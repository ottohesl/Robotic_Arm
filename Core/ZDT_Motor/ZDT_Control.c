/**
  ******************************************************************************
  * @file           : ZDT_CONTROL.c
  * @brief          : ZDT系列电机CAN通信控制驱动实现文件
  * @author         : ottohesl
  * @date           : 2026-02-01
  * @version        : V1.0
  * @note           : 实现ZDT电机各类控制指令的CAN帧封装与发送，及反馈帧解析
  ******************************************************************************
  */
#include "ZDT_Control.h"
#include <stdbool.h>
#include "fdcan.h"
#include "ZDT_MOTOR_LOG.h"

/* 全局FDCAN句柄：绑定到hfdcan2 */
FDCAN_HandleTypeDef *Motor_hfdcan=&hfdcan2;

/**
 * @brief  CAN数据分包发送函数
 * @details 支持长数据分包发送，自动处理CAN帧ID、数据长度及分包逻辑
 * @param  hfdcan: FDCAN句柄指针
 * @param  data: 待发送的数据缓冲区（格式：地址+功能码+参数+校验）
 * @param  len: 待发送数据总长度
 * @retval 无
 * @note   数据格式要求：data[0]=地址，data[1]=功能码，后续为参数，最后为校验字节
 */
void Send_Can(FDCAN_HandleTypeDef *hfdcan,uint8_t* data , uint8_t len) {
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0};
    uint8_t Data_Length=0, Now_Length=0, Flag=0;
    uint8_t PackNum = 0;

    /* 参数合法性检查 */
    if (len<2||data==NULL) {
        return;
    }

    /* 初始化CAN发送头参数 */
    TxHeader.Identifier = 0x00;                // 标准ID（初始值）
    TxHeader.IdType = FDCAN_EXTENDED_ID;       // 扩展帧模式
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;   // 数据帧
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;   // 默认8字节长度
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;    // 关闭位速率切换
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;     // 经典CAN模式（非FD）
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    /* 计算有效数据长度（排除地址和功能码） */
    Data_Length = len - 2;

    /* 分包发送逻辑 */
    while (Flag<Data_Length) {
        Now_Length = Data_Length - Flag;    // 剩余未发送数据长度
        TxHeader.Identifier=((uint32_t)data[0]<<8) | (uint32_t)PackNum; // 构造分包ID
        TxData[0]=data[1];  // 填充功能码

        /* 数据填充逻辑 */
        if (Now_Length<7) {
            // 剩余数据不足7字节，按实际长度填充
            for (int i = 0 ; i < Now_Length ; i++ , Flag++) {
                TxData[i+1]=data[Flag+2];
            }
            // 根据实际数据长度设置CAN帧长度
            switch((Now_Length + 1)) { // +1是因为TxData[0]已填充功能码
                case 1: TxHeader.DataLength = FDCAN_DLC_BYTES_1; break;
                case 2: TxHeader.DataLength = FDCAN_DLC_BYTES_2; break;
                case 3: TxHeader.DataLength = FDCAN_DLC_BYTES_3; break;
                case 4: TxHeader.DataLength = FDCAN_DLC_BYTES_4; break;
                case 5: TxHeader.DataLength = FDCAN_DLC_BYTES_5; break;
                case 6: TxHeader.DataLength = FDCAN_DLC_BYTES_6; break;
                case 7: TxHeader.DataLength = FDCAN_DLC_BYTES_7; break;
                default: TxHeader.DataLength = FDCAN_DLC_BYTES_8; break;
            }
        }else {
            // 剩余数据≥7字节，整包8字节发送
            for (int i = 0 ; i < 7 ; i++, Flag++) {
                TxData[i+1]=data[Flag+2];
            }
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
        }

        /* 发送CAN帧 */
        HAL_StatusTypeDef state=HAL_FDCAN_AddMessageToTxFifoQ(hfdcan,&TxHeader,TxData);

        /* 调试模式：打印发送状态 */
#if deuge_mode
        switch (state) {
            case HAL_OK:        OTTO_uart(&huart_debug,"发送can命令成功");break;
            case HAL_BUSY:      OTTO_uart(&huart_debug,"can帧busy");break;
            case HAL_ERROR:     OTTO_uart(&huart_debug,"can帧error");break;
            case HAL_TIMEOUT:   OTTO_uart(&huart_debug,"can帧timeout");break;
            default:break;
        }
#endif

        PackNum++; // 分包序号自增
    }
}

/**
 * @brief  电机位置清零指令发送
 * @param  hfdcan: FDCAN句柄指针
 * @param  addr: 电机地址
 * @retval 无
 * @note   指令格式：地址+0x0A(功能码)+0x6D(辅助码)+0x6B(校验)
 */
void ZDT_Control_Pos_Clear(FDCAN_HandleTypeDef *hfdcan,uint8_t addr) {
    uint8_t cmd[16]={0};

    // 装载位置清零指令
    cmd[0] =  addr;                       // 电机地址
    cmd[1] =  0x0A;                       // 位置清零功能码
    cmd[2] =  0x6D;                       // 辅助码
    cmd[3] =  0x6B;                       // 校验字节

    // 发送CAN指令
    Send_Can(hfdcan,cmd,4);
}

/**
 * @brief  电机使能/禁用指令发送
 * @param  hfdcan: FDCAN句柄指针
 * @param  addr: 电机地址
 * @param  state: 使能状态：true-使能，false-禁用
 * @param  sync: 同步标志：SIN-单电机，ALL-多电机
 * @retval 无
 * @note   指令格式：地址+0xF3(功能码)+0xAB(辅助码)+使能状态+同步标志+校验
 */
void ZDT_Control_Enable_Motor(FDCAN_HandleTypeDef *hfdcan,uint8_t addr, bool state, Sync sync)
{
    uint8_t cmd[16] = {0};

    // 装载使能指令
    cmd[0] =  addr;                       // 电机地址
    cmd[1] =  0xF3;                       // 使能功能码
    cmd[2] =  0xAB;                       // 辅助码
    cmd[3] =  (uint8_t)state;             // 使能状态
    cmd[4] =  sync;                       // 多机同步标志
    cmd[5] =  0x6B;                       // 校验字节

    // 发送CAN指令
    Send_Can(hfdcan,cmd, 6);
}

/**
 * @brief  电机力矩模式控制指令发送
 * @param  hfdcan: FDCAN句柄指针
 * @param  addr: 电机地址
 * @param  dir: 旋转方向：CW-顺时针，CCW-逆时针
 * @param  ele_slope: 力矩斜率（加速度）
 * @param  ele: 目标力矩值
 * @param  sync: 同步标志：SIN-单电机，ALL-多电机
 * @retval 无
 * @note   指令格式：地址+0xF5(功能码)+方向+力矩斜率(2字节)+力矩值(2字节)+同步标志+校验
 */
void ZDT_Control_Tor_Mode(FDCAN_HandleTypeDef *hfdcan,uint8_t addr,Dir dir, uint16_t ele_slope,uint16_t ele, Sync sync) {
    uint8_t cmd[16] = {0};

    // 装载力矩控制指令
    cmd[0] =  addr;                       // 电机地址
    cmd[1] =  0xF5;                       // 力矩模式功能码
    cmd[2] =  dir;                        // 旋转方向
    cmd[3] =  (uint8_t)(ele_slope>>8);    // 力矩斜率高8位
    cmd[4] =  (uint8_t)(ele_slope>>0);    // 力矩斜率低8位
    cmd[5] =  (uint8_t)(ele>>8);          // 力矩值高8位
    cmd[6] =  (uint8_t)(ele>>0);          // 力矩值低8位
    cmd[7] =  sync;                       // 多机同步标志
    cmd[8] =  0x6B;                       // 校验字节

    // 发送CAN指令
    Send_Can(hfdcan,cmd, 9);
}

/**
 * @brief  电机速度模式控制指令发送
 * @param  hfdcan: FDCAN句柄指针
 * @param  addr: 电机地址
 * @param  dir: 旋转方向：CW-顺时针，CCW-逆时针
 * @param  vel_slope: 速度斜率（加速度）
 * @param  vel: 目标速度（RPM）
 * @param  sync: 同步标志：SIN-单电机，ALL-多电机
 * @retval 无
 * @note   指令格式：地址+0xF6(功能码)+方向+速度(2字节)+加速度+同步标志+校验
 */
void ZDT_Control_Vel_Mode(FDCAN_HandleTypeDef *hfdcan,uint8_t addr,Dir dir, uint16_t vel_slope,uint16_t vel, Sync sync) {
    uint8_t cmd[16] = {0};

    // 装载速度控制指令
    cmd[0] =  addr;                       // 电机地址
    cmd[1] =  0xF6;                       // 速度模式功能码
    cmd[2] =  dir;                        // 旋转方向
    cmd[3] =  (uint8_t)(vel >> 8);        // 速度高8位
    cmd[4] =  (uint8_t)(vel >> 0);        // 速度低8位
    cmd[5] =  vel_slope;                  // 加速度（0为直接启动）
    cmd[6] =  sync;                       // 多机同步标志
    cmd[7] =  0x6B;                       // 校验字节

    // 发送CAN指令
    Send_Can(hfdcan,cmd, 8);
}

/**
 * @brief  电机梯形曲线位置模式控制指令发送
 * @param  hfdcan: FDCAN句柄指针
 * @param  addr: 电机地址
 * @param  dir: 旋转方向：CW-顺时针，CCW-逆时针
 * @param  acc_vel: 加速度
 * @param  max_vel: 最大速度
 * @param  pos: 目标位置脉冲数
 * @param  loca: 位置类型：RELATIVE_POS-相对，ABSOLUTE_POS-绝对
 * @param  sync: 同步标志：SIN-单电机，ALL-多电机
 * @retval 无
 * @note   指令格式：地址+0xFD(功能码)+方向+最大速度(2字节)+加速度+位置(4字节)+位置类型+同步标志+校验
 */
void ZDT_Control_Trape_Pos_Mode(FDCAN_HandleTypeDef *hfdcan,uint8_t addr,Dir dir,uint16_t acc_vel, uint16_t max_vel, uint16_t pos, Loca_Manage loca,Sync sync) {
    uint8_t cmd[16] = {0};

    // 装载梯形位置控制指令
    cmd[0] =  addr;                       // 电机地址
    cmd[1] =  0xFD;                       // 梯形位置模式功能码
    cmd[2] =  dir;                        // 旋转方向
    cmd[3] =  (uint8_t)(max_vel>>8);      // 最大速度高8位
    cmd[4] =  (uint8_t)(max_vel>>0);      // 最大速度低8位
    cmd[5] =  acc_vel;                    // 加速度（0为直接启动）
    cmd[6] =  (uint8_t)(pos >> 24);       // 位置脉冲数bit24-31
    cmd[7] =  (uint8_t)(pos >> 16);       // 位置脉冲数bit16-23
    cmd[8] =  (uint8_t)(pos >> 8);        // 位置脉冲数bit8-15
    cmd[9] =  (uint8_t)(pos >> 0);        // 位置脉冲数bit0-7
    cmd[10] = loca;                       // 位置管理类型
    cmd[11] = sync;                       // 多机同步标志
    cmd[12] =  0x6B;                      // 校验字节

    // 发送CAN指令
    Send_Can(hfdcan,cmd, 13);
}

/**
 * @brief  电机直通限速位置模式控制指令发送
 * @param  hfdcan: FDCAN句柄指针
 * @param  addr: 电机地址
 * @param  dir: 旋转方向：CW-顺时针，CCW-逆时针
 * @param  vel: 限速值
 * @param  pos: 目标位置脉冲数
 * @param  loca: 位置类型：RELATIVE_POS-相对，ABSOLUTE_POS-绝对
 * @param  sync: 同步标志：SIN-单电机，ALL-多电机
 * @retval 无
 * @note   指令格式：地址+0xFB(功能码)+方向+限速(2字节)+位置(4字节)+位置类型+同步标志+校验
 */
void ZDT_Control_Direct_Pos_Mode(FDCAN_HandleTypeDef *hfdcan,uint8_t addr,Dir dir,uint16_t vel, uint16_t pos, Loca_Manage loca, Sync sync) {
    uint8_t cmd[16] = {0};

    // 装载直通位置控制指令
    cmd[0] =  addr;                       // 电机地址
    cmd[1] =  0xFB;                       // 直通位置模式功能码
    cmd[2] =  dir;                        // 旋转方向
    cmd[3] =  (uint8_t)(vel>>8);          // 限速值高8位
    cmd[4] =  (uint8_t)(vel>>0);          // 限速值低8位
    cmd[5] =  (uint8_t)(pos >> 24);       // 位置脉冲数bit24-31
    cmd[6] =  (uint8_t)(pos >> 16);       // 位置脉冲数bit16-23
    cmd[7] =  (uint8_t)(pos >> 8);        // 位置脉冲数bit8-15
    cmd[8] =  (uint8_t)(pos >> 0);        // 位置脉冲数bit0-7
    cmd[9] = loca;                        // 位置管理类型
    cmd[10] = sync;                       // 多机同步标志
    cmd[11] =  0x6B;                      // 校验字节

    // 发送CAN指令
    Send_Can(hfdcan,cmd, 12);
}

/**
 * @brief  电机立即停止指令发送
 * @param  hfdcan: FDCAN句柄指针
 * @param  addr: 电机地址
 * @param  sync: 同步标志：SIN-单电机，ALL-多电机
 * @retval 无
 * @note   指令格式：地址+0xFE(功能码)+0x98(辅助码)+同步标志+校验
 */
void ZDT_Control_Stop_Motor(FDCAN_HandleTypeDef *hfdcan,uint8_t addr,Sync sync) {
    uint8_t cmd[16] = {0};

    // 装载停止指令
    cmd[0] =  addr;                       // 电机地址
    cmd[1] =  0xFE;                       // 停止功能码
    cmd[2] =  0x98;                       // 辅助码
    cmd[3] =  sync;                       // 多机同步标志
    cmd[4] =  0x6B;                       // 校验字节

    // 发送CAN指令
    Send_Can(hfdcan,cmd, 5);
}

/**
 * @brief  多机同步使能指令发送
 * @param  hfdcan: FDCAN句柄指针
 * @param  addr: 电机地址
 * @retval 无
 * @note   指令格式：地址+0xFF(功能码)+0x66(辅助码)+校验
 */
void ZDT_Control_Sync_Mode(FDCAN_HandleTypeDef *hfdcan,uint8_t addr) {
    uint8_t cmd[16] = {0};

    // 装载同步使能指令
    cmd[0] =  addr;                       // 电机地址
    cmd[1] =  0xFF;                       // 同步功能码
    cmd[2] =  0x66;                       // 辅助码
    cmd[3] =  0x6B;                       // 校验字节

    // 发送CAN指令
    Send_Can(hfdcan,cmd, 4);
}

/**
 * @brief  修改电机回零参数指令发送
 * @param  hfdcan: FDCAN句柄指针
 * @param  addr: 电机地址
 * @param  svF: 存储标志：true-存储，false-不存储
 * @param  o_mode: 回零模式（参考O_Mode枚举）
 * @param  o_dir: 回零方向：0-CW，其他-CCW
 * @param  o_vel: 回零速度（RPM）
 * @param  o_tm: 回零超时时间（ms）
 * @param  sl_vel: 碰撞回零检测转速（RPM）
 * @param  sl_ma: 碰撞回零检测电流（mA）
 * @param  sl_ms: 碰撞回零检测时间（ms）
 * @param  potF: 上电自动回零：true-使能，false-禁用
 * @retval 无
 */
void ZDT_Control_Origin_Modify_Params(FDCAN_HandleTypeDef *hfdcan,uint8_t addr, bool svF, O_Mode o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
    uint8_t cmd[32] = {0};

    // 装载回零参数修改指令
    cmd[0] =  addr;                       // 电机地址
    cmd[1] =  0x4C;                       // 回零参数修改功能码
    cmd[2] =  0xAE;                       // 辅助码
    cmd[3] =  svF;                        // 存储标志
    cmd[4] =  o_mode;                     // 回零模式
    cmd[5] =  o_dir;                      // 回零方向
    cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度高8位
    cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度低8位
    cmd[8]  =  (uint8_t)(o_tm >> 24);     // 超时时间bit24-31
    cmd[9]  =  (uint8_t)(o_tm >> 16);     // 超时时间bit16-23
    cmd[10] =  (uint8_t)(o_tm >> 8);      // 超时时间bit8-15
    cmd[11] =  (uint8_t)(o_tm >> 0);      // 超时时间bit0-7
    cmd[12] =  (uint8_t)(sl_vel >> 8);    // 检测转速高8位
    cmd[13] =  (uint8_t)(sl_vel >> 0);    // 检测转速低8位
    cmd[14] =  (uint8_t)(sl_ma >> 8);     // 检测电流高8位
    cmd[15] =  (uint8_t)(sl_ma >> 0);     // 检测电流低8位
    cmd[16] =  (uint8_t)(sl_ms >> 8);     // 检测时间高8位
    cmd[17] =  (uint8_t)(sl_ms >> 0);     // 检测时间低8位
    cmd[18] =  potF;                      // 上电自动回零标志
    cmd[19] =  0x6B;                      // 校验字节

    // 发送CAN指令
    Send_Can(hfdcan,cmd, 20);
}

/**
 * @brief  触发电机回零指令发送
 * @param  hfdcan: FDCAN句柄指针
 * @param  addr: 电机地址
 * @param  o_mode: 回零模式（参考O_Mode枚举）
 * @param  sync: 同步标志：SIN-单电机，ALL-多电机
 * @retval 无
 * @note   指令格式：地址+0x9A(功能码)+回零模式+同步标志+校验
 */
void ZDT_Control_Origin_Trigger_Return(FDCAN_HandleTypeDef *hfdcan,uint8_t addr, uint8_t o_mode, Sync sync)
{
    uint8_t cmd[16] = {0};

    // 装载触发回零指令
    cmd[0] =  addr;                       // 电机地址
    cmd[1] =  0x9A;                       // 触发回零功能码
    cmd[2] =  o_mode;                     // 回零模式
    cmd[3] =  sync;                       // 多机同步标志
    cmd[4] =  0x6B;                       // 校验字节

    // 发送CAN指令
    Send_Can(hfdcan,cmd,5);
}

/**
 * @brief  中断退出电机回零指令发送
 * @param  hfdcan: FDCAN句柄指针
 * @param  addr: 电机地址
 * @retval 无
 * @note   指令格式：地址+0x9C(功能码)+0x48(辅助码)+校验
 */
void ZDT_Control_Origin_Interrupt(FDCAN_HandleTypeDef *hfdcan,uint8_t addr)
{
    uint8_t cmd[16] = {0};

    // 装载中断回零指令
    cmd[0] =  addr;                       // 电机地址
    cmd[1] =  0x9C;                       // 中断回零功能码
    cmd[2] =  0x48;                       // 辅助码
    cmd[3] =  0x6B;                       // 校验字节

    // 发送CAN指令
   Send_Can(hfdcan,cmd,4);
}

/**
 * @brief  读取电机系统参数指令发送
 * @param  hfdcan: FDCAN句柄指针
 * @param  addr: 电机地址
 * @param  s: 待读取的参数类型（参考SysParams_t枚举）
 * @retval 无
 */
void ZDT_Control_Read_Sys_Params(FDCAN_HandleTypeDef *hfdcan,uint8_t addr, SysParams_t s)
{
    uint8_t i = 0;
    uint8_t cmd[16] = {0};

    // 装载电机地址
    cmd[i] = addr; ++i;

    // 根据参数类型装载功能码
    switch(s)
    {
        case S_VER  : cmd[i] = 0x1F; ++i; break;
        case S_RL   : cmd[i] = 0x20; ++i; break;
        case S_PID  : cmd[i] = 0x21; ++i; break;
        case S_VBUS : cmd[i] = 0x24; ++i; break;
        case S_CPHA : cmd[i] = 0x27; ++i; break;
        case S_ENCL : cmd[i] = 0x31; ++i; break;
        case S_TPOS : cmd[i] = 0x33; ++i; break;
        case S_VEL  : cmd[i] = 0x35; ++i; break;
        case S_CPOS : cmd[i] = 0x36; ++i; break;
        case S_PERR : cmd[i] = 0x37; ++i; break;
        case S_FLAG : cmd[i] = 0x3A; ++i; break;
        case S_ORG  : cmd[i] = 0x3B; ++i; break;
        case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
        case S_STATE: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
        default: break;
    }

    // 装载校验字节
    cmd[i] = 0x6B; ++i;

    // 发送CAN指令
     Send_Can(hfdcan,cmd, i);
}

/**
 * @brief  读取电机全部系统参数指令发送
 * @param  hfdcan: FDCAN句柄指针
 * @param  addr: 电机地址
 * @retval 无
 * @note   指令格式：地址+0x43(功能码)+0x7A(辅助码)+校验
 */
void ZDT_Control_Read_ALL_Params(FDCAN_HandleTypeDef *hfdcan,uint8_t addr) {
    uint8_t cmd[16] = {0};

    // 装载读取全部参数指令
    cmd[0] = addr;                        // 电机地址
    cmd[1] = 0x43;                        // 读取系统状态功能码
    cmd[2] = 0x7A;                        // 辅助码
    cmd[3] = 0x6B;                        // 校验字节

    // 发送CAN指令
    Send_Can(hfdcan,cmd, 4);
}

/**
 * @brief  解析电机CAN反馈帧数据
 * @param  motor: 电机反馈数据结构体指针（输出参数）
 * @param  rxdata: 接收的CAN数据缓冲区（输入参数）
 * @param  id: 电机地址ID
 * @retval 无
 * @note   根据反馈帧功能码解析对应参数，填充到motor结构体中
 */
void ZDT_Control_Analyze_FDBack(ZDT_FBpara_t *motor,const uint8_t *rxdata,const uint32_t id) {
    if (rxdata==NULL) {
        return;
    }

    // 初始化状态参数
    motor->Motor_Status.ERROR=0;
    motor->IS_Receive = 0;
    motor->vaild = 0;
    motor->id = id;

    // 根据功能码解析反馈数据
    switch(rxdata[0]) {
        /******************************读取电机参数表**************************************/
        case S_PID: // PID参数
            if (rxdata[13]==CHECK_SUM) {
                motor->S_pid.Pos_kp=(rxdata[1]<<24)|(rxdata[2]<<16)|(rxdata[3]<<8)|(rxdata[4]<<0);
                motor->S_pid.Pos_ki=(rxdata[5]<<24)|(rxdata[6]<<16)|(rxdata[7]<<8)|(rxdata[8]<<0);
                motor->S_pid.Pos_kd=(rxdata[9]<<24)|(rxdata[10]<<16)|(rxdata[11]<<8)|(rxdata[12]<<0);
                motor->vaild=1;
            }
            break;

        case S_CPHA: // 相电流
            if (rxdata[3]==CHECK_SUM) {
                motor->S_Cpha = (rxdata[1]<<8)|rxdata[2];
                motor->vaild=1;
            }
            break;

        case S_VBUS: // 总线电压
            if (rxdata[3]==CHECK_SUM) {
                motor->S_vbus = (rxdata[1]<<8)|rxdata[2];
                motor->vaild=1;
            }
            break;

        case S_TPOS: // 目标位置
            if (rxdata[6]==CHECK_SUM) {
                if (rxdata[1] == CCW)
                    motor->S_Tpos = (-1)*(float)((rxdata[2]<<24)|(rxdata[3]<<16)|(rxdata[4]<<8)|(rxdata[5]<<0))*360.0f/65536.0f;
                else
                    motor->S_Tpos=(float)(((rxdata[2]<<24)|(rxdata[3]<<16)|(rxdata[4]<<8)|(rxdata[5]<<0)))*360.0f/65536.0f;
                motor->vaild=1;
            }
            break;

        case S_CPOS: // 当前位置
            if (rxdata[6]==CHECK_SUM) {
                if (rxdata[1]==CCW)
                    motor->S_Cpos=(-1)*(float)(((rxdata[2]<<24)|(rxdata[3]<<16)|(rxdata[4]<<8)|(rxdata[5]<<0)))*360.0f/65536.0f;
                else
                    motor->S_Cpos=(float)(((rxdata[2]<<24)|(rxdata[3]<<16)|(rxdata[4]<<8)|(rxdata[5]<<0)))*360.0f/65536.0f;
                motor->vaild=1;
            }
            break;

        case S_VEL: // 转速
            if (rxdata[4]==CHECK_SUM) {
                if (rxdata[1]==CCW) {
                    motor->S_Vel.Vel_RPM = (-1)*(float)((rxdata[2]<<8)|rxdata[3]);
                    motor->S_Vel.Vel_RPS = (-1)*(float)((rxdata[2]<<8)|rxdata[3])/60.0f;
                }else {
                    motor->S_Vel.Vel_RPM = (float)((rxdata[2]<<8)|rxdata[3]);
                    motor->S_Vel.Vel_RPS = (float)((rxdata[2]<<8)|rxdata[3])/60.0f;
                }
                motor->vaild=1;
            }
            break;

        case S_PERR: // 位置误差
            if (rxdata[6]==CHECK_SUM) {
                if (rxdata[1]==CCW)
                    motor->S_Perr= (-1)*(float)((rxdata[2]<<24)|(rxdata[3]<<16)|(rxdata[4]<<8)|(rxdata[5]<<0))*360.0f/65536.0f;
                else
                    motor->S_Perr=(float)((rxdata[2]<<24)|(rxdata[3]<<16)|(rxdata[4]<<8)|(rxdata[5]<<0))*360.0f/65536.0f;
                motor->vaild=1;
            }
            break;

        case S_TEMP: // 驱动器温度
            if (rxdata[3]==CHECK_SUM) {
                if (rxdata[1]==CCW)
                    motor->S_Temp= (-1)*(float)rxdata[2];
                else
                    motor->S_Temp=(float)rxdata[2];
                motor->vaild=1;
            }
            break;

        case S_FLAG: // 状态标志位
            if (rxdata[2]==CHECK_SUM) {
                motor->S_Flag.IS_ENABLE=rxdata[1]&0x01;
                motor->S_Flag.IS_INPLACE=rxdata[1]&0x02;
                motor->S_Flag.IS_LOCKED=rxdata[1]&0x04;
                motor->S_Flag.IS_SAVE_LOCKED=rxdata[1]&0x08;
                motor->vaild=1;
            }
            break;

        case S_STATE: // 系统状态（全参数）
            if (rxdata[29]==CHECK_SUM) {
                // 总线电压
                motor->S_vbus =  rxdata[3]<<8|rxdata[4];
                // 相电流
                motor->S_Cpha = (rxdata[5]<<8)|rxdata[6];
                // 目标位置
                if (rxdata[9]==CCW)
                    motor->S_Tpos= (-1)*(float)((rxdata[10]<<24)|(rxdata[11]<<16)|(rxdata[12]<<8)|(rxdata[13]<<0))*360.0f/65536.0f;
                else
                    motor->S_Tpos=(float)((rxdata[10]<<24)|(rxdata[11]<<16)|(rxdata[12]<<8)|(rxdata[13]<<0))*360.0f/65536.0f;
                // 转速
                if (rxdata[14]==CCW) {
                    motor->S_Vel.Vel_RPM = (-1)*(float)((rxdata[15]<<8)|rxdata[16]);
                    motor->S_Vel.Vel_RPS = (-1)*(float)((rxdata[15]<<8)|rxdata[16])/60.0f;
                }else {
                    motor->S_Vel.Vel_RPM = (float)((rxdata[15]<<8)|rxdata[16]);
                    motor->S_Vel.Vel_RPS = (float)((rxdata[15]<<8)|rxdata[16])/60.0f;
                }
                // 当前位置
                if (rxdata[17]==CCW)
                    motor->S_Cpos=(-1)*(float)((rxdata[18]<<24)|(rxdata[19]<<16)|(rxdata[20]<<8)|(rxdata[21]<<0))*360.0f/65536.0f;
                else
                    motor->S_Cpos=(float)((rxdata[18]<<24)|(rxdata[19]<<16)|(rxdata[20]<<8)|(rxdata[21]<<0))*360.0f/65536.0f;
                // 位置误差
                if (rxdata[22]==CCW)
                    motor->S_Perr= (-1)*(float)((rxdata[23]<<24)|(rxdata[24]<<16)|(rxdata[25]<<8)|(rxdata[26]<<0))*360.0f/65536.0f;
                else
                    motor->S_Perr=(float)((rxdata[23]<<24)|(rxdata[24]<<16)|(rxdata[25]<<8)|(rxdata[26]<<0))*360.0f/65536.0f;
                // 状态标志位
                motor->S_Flag.IS_ENCL=rxdata[27]&0x01;
                motor->S_Flag.IS_RIGHT=rxdata[27]&0x02;
                motor->S_Flag.IS_RETURN=rxdata[27]&0x04;
                motor->S_Flag.IS_RETURN_FAULT=rxdata[27]&0x08;
                motor->S_Flag.IS_ENABLE=rxdata[28]&0x01;
                motor->S_Flag.IS_INPLACE=rxdata[28]&0x02;
                motor->S_Flag.IS_LOCKED=rxdata[28]&0x04;
                motor->S_Flag.IS_SAVE_LOCKED=rxdata[28]&0x08;

                motor->vaild = 1;
            }
            break;

        /*******************************读取电机命令反馈值**************************************/
        case ZDT_FUNC_ENABLE_MOTOR: // 使能指令反馈
            if (rxdata[1]==0x02&&rxdata[2]==0x6B)
                motor->Motor_Status.EN_status=Receive_Success;
            if (rxdata[1]==0xE2&&rxdata[2]==0x6B)
                motor->Motor_Status.EN_status=Error_Parameter;
            motor->IS_Receive = 1;
            break;

        case ZDT_FUNC_TOR_MODE: // 力矩模式指令反馈
            if (rxdata[1]==0x02&&rxdata[2]==0x6B)
                motor->Motor_Status.Tor_Mode_status=Receive_Success;
            if (rxdata[1]==0xE2&&rxdata[2]==0x6B)
                motor->Motor_Status.Tor_Mode_status=Error_Parameter;
            motor->IS_Receive = 1;
            break;

        case ZDT_FUNC_VEL_MODE: // 速度模式指令反馈
            if (rxdata[1]==0x02&&rxdata[2]==0x6B)
                motor->Motor_Status.Vel_Mode_status=Receive_Success;
            if (rxdata[1]==0xE2&&rxdata[2]==0x6B)
                motor->Motor_Status.Vel_Mode_status=Error_Parameter;
            motor->IS_Receive = 1;
            break;

        case ZDT_FUNC_TRAPE_POS_MODE: // 梯形位置模式指令反馈
            if (rxdata[1]==0x02&&rxdata[2]==0x6B)
                motor->Motor_Status.Trape_Pos_Val_Mode_status=Receive_Success;
            if (rxdata[1]==0xE2&&rxdata[2]==0x6B)
                motor->Motor_Status.Trape_Pos_Val_Mode_status=Error_Parameter;
            motor->IS_Receive = 1;
            break;

        case ZDT_FUNC_DIRECT_POS_MODE: // 直通位置模式指令反馈
            if (rxdata[1]==0x02&&rxdata[2]==0x6B)
                motor->Motor_Status.Direct_Pos_Mode_status=Receive_Success;
            if (rxdata[1]==0xE2&&rxdata[2]==0x6B)
                motor->Motor_Status.Direct_Pos_Mode_status=Error_Parameter;
            motor->IS_Receive = 1;
            break;

        case ZDT_FUNC_STOP_MOTOR: // 停止指令反馈
            if (rxdata[1]==0x02&&rxdata[2]==0x6B)
                motor->Motor_Status.Stop_Motor_status=Receive_Success;
            if (rxdata[1]==0xE2&&rxdata[2]==0x6B)
                motor->Motor_Status.Stop_Motor_status=Error_Parameter;
            motor->IS_Receive = 1;
            break;

        case ZDT_FUNC_SYNC_MODE: // 同步模式指令反馈
            if (rxdata[1]==0x02&&rxdata[2]==0x6B)
                motor->Motor_Status.Sync_Mode_status=Receive_Success;
            if (rxdata[1]==0xE2&&rxdata[2]==0x6B)
                motor->Motor_Status.Sync_Mode_status=Error_Parameter;
            motor->IS_Receive = 1;
            break;

        default: // 未知指令/错误指令
            if (rxdata[0]==0x00&&rxdata[1]==0XEE&&rxdata[2]==0X6B)
                motor->Motor_Status.ERROR=Error_Command;
            motor->IS_Receive = 1;
            break;
    }
}

/***************************************************基本控制函数******************************************************/

/**
 * @brief  电机梯形位置模式快捷控制函数
 * @param  addr: 电机地址
 * @param  dir: 旋转方向：CW-顺时针，CCW-逆时针
 * @param  acc: 加速度
 * @param  vel_RPS: 目标速度（转/秒）
 * @param  angle: 目标角度（°）
 * @retval 无
 * @note   自动转换速度单位（转/秒→转/分钟）和角度→脉冲数，简化调用
 */
void ZDT_MOTOR_POSITION(uint8_t addr,Dir dir,uint16_t acc, uint16_t vel_RPS, float angle) {
    uint16_t pos=P(angle);          // 角度转脉冲数
    uint16_t vel=vel_RPS*60.0f;     // 转/秒 → 转/分钟

    // 发送梯形位置模式指令
    ZDT_Control_Trape_Pos_Mode(Motor_hfdcan,addr,dir,acc,vel,pos,ABSOLUTE_POS,SIN);
    osDelay(200); // 延时确保指令发送完成
}

/**
 * @brief  电机速度模式快捷控制函数
 * @param  addr: 电机地址
 * @param  dir: 旋转方向：CW-顺时针，CCW-逆时针
 * @param  acc: 加速度
 * @param  vel_RPS: 目标速度（转/秒）
 * @retval 无
 * @note   自动转换速度单位（转/秒→转/分钟），简化调用
 */
void ZDT_MOTOR_VEL(uint8_t addr,Dir dir,uint16_t acc, uint16_t vel_RPS) {
    uint16_t vel=vel_RPS*60.0f;     // 转/秒 → 转/分钟

    // 发送速度模式指令
    ZDT_Control_Vel_Mode(Motor_hfdcan,addr,dir,acc,vel,SIN);
}