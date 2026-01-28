#include "ZDT_Control.h"
#include <stdbool.h>
#include "fdcan.h"
#include "ZDT_MOTOR_LOG.h"
FDCAN_HandleTypeDef *Motor_hfdcan=&hfdcan2;
void Send_Can(FDCAN_HandleTypeDef *hfdcan,uint8_t* data , uint8_t len) {
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0};
    uint8_t Data_Length=0, Now_Length=0, Flag=0;
    uint8_t PackNum = 0;
    if (len<2||data==NULL) {
        return;
    }
    TxHeader.Identifier = 0x00;                // 标准ID
    TxHeader.IdType = FDCAN_EXTENDED_ID;       // 扩展帧
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;   // 数据帧
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;   // 先默认8字节，后续根据分包调整
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;    // 普通CAN模式，关闭位速率切换
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;     // 经典CAN模式（非FD模式）
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    Data_Length = len - 2;
    while (Flag<Data_Length) {
        Now_Length = Data_Length - Flag;    //剩下数据的数据
        TxHeader.Identifier=((uint32_t)data[0]<<8) | (uint32_t)PackNum;
        TxData[0]=data[1];  //功能码填写
        //构造can帧
        if (Now_Length<7) {
            //依据实际字节发送数据
            for (int i = 0 ; i < Now_Length ; i++ , Flag++) {
                TxData[i+1]=data[Flag+2];
            }
            switch((Now_Length + 1)) { // +1是因为TxData[0]是功能码
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
            //整包发送8字节数据
            for (int i = 0 ; i < 7 ; i++, Flag++) {
                TxData[i+1]=data[Flag+2];
            }
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
        }
        //发送can帧
        HAL_StatusTypeDef state=HAL_FDCAN_AddMessageToTxFifoQ(hfdcan,&TxHeader,TxData);
#if deuge_mode
        switch (state) {
            case HAL_OK:        OTTO_uart(&huart_debug,"发送can命令成功");break;
            case HAL_BUSY:      OTTO_uart(&huart_debug,"can帧busy");break;
            case HAL_ERROR:     OTTO_uart(&huart_debug,"can帧error");break;
            case HAL_TIMEOUT:   OTTO_uart(&huart_debug,"can帧timeout");break;
            default:break;
        }
        #endif
        PackNum++;
    }
}
//位置清零
void ZDT_Control_Pos_Clear(FDCAN_HandleTypeDef *hfdcan,uint8_t addr) {
    uint8_t cmd[16]={0};
    // 装载命令
    cmd[0] =  addr;                       // 地址
    cmd[1] =  0x0A;                       // 功能码
    cmd[2] =  0x6D;                       // 辅助码
    cmd[3] =  0x6B;                       // 校验字节

    // 发送命令
    Send_Can(hfdcan,cmd,4);
}

void ZDT_Control_Enable_Motor(FDCAN_HandleTypeDef *hfdcan,uint8_t addr, bool state, Sync sync)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  addr;                       // 地址
    cmd[1] =  0xF3;                       // 功能码
    cmd[2] =  0xAB;                       // 辅助码
    cmd[3] =  (uint8_t)state;             // 使能状态
    cmd[4] =  sync;                        // 多机同步运动标志
    cmd[5] =  0x6B;                       // 校验字节

    // 发送命令
    Send_Can(hfdcan,cmd, 6);
}
//力矩控制模式
void ZDT_Control_Tor_Mode(FDCAN_HandleTypeDef *hfdcan,uint8_t addr,Dir dir, uint16_t ele_slope,uint16_t ele, Sync sync) {
    uint8_t cmd[16] = {0};
    cmd[0] =  addr;
    cmd[1] =  0xF5;
    cmd[2] =  dir;
    cmd[3] =  (uint8_t)(ele_slope>>8);
    cmd[4] =  (uint8_t)(ele_slope>>0);
    cmd[5] =  (uint8_t)(ele>>8);
    cmd[6] =  (uint8_t)(ele>>0);
    cmd[7] = sync;
    cmd[8] =  0x6B;
    Send_Can(hfdcan,cmd, 9);
}
//速度模式控制
void ZDT_Control_Vel_Mode(FDCAN_HandleTypeDef *hfdcan,uint8_t addr,Dir dir, uint16_t vel_slope,uint16_t vel, Sync sync) {
    uint8_t cmd[16] = {0};
    cmd[0] =  addr;                       // 地址
    cmd[1] =  0xF6;                       // 功能码
    cmd[2] =  dir;                        // 方向
    cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
    cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
    cmd[5] =  vel_slope;                      // 加速度，注意：0是直接启动
    cmd[6] =  sync;                        // 多机同步运动标志
    cmd[7] =  0x6B;                       // 校验字节

    Send_Can(hfdcan,cmd, 8);
}
//梯形曲线位置模式控制
void ZDT_Control_Trape_Pos_Mode(FDCAN_HandleTypeDef *hfdcan,uint8_t addr,Dir dir,uint16_t acc_vel, uint16_t max_vel, uint16_t pos, Loca_Manage loca,Sync sync) {
    uint8_t cmd[16] = {0};
    cmd[0] =  addr;
    cmd[1] =  0xFD;
    cmd[2] =  dir;                      //对位置的方向
    cmd[3] =  (uint8_t)(max_vel>>8);
    cmd[4] =  (uint8_t)(max_vel>>0);
    cmd[5] =  acc_vel;            // 加速度，注意：0是直接启动
    cmd[6] =  (uint8_t)(pos >> 24);      // 脉冲数(bit24 - bit31)
    cmd[7] =  (uint8_t)(pos >> 16);      // 脉冲数(bit16 - bit23)
    cmd[8] =  (uint8_t)(pos >> 8);       // 脉冲数(bit8  - bit15)
    cmd[9] =  (uint8_t)(pos >> 0);       // 脉冲数(bit0  - bit7 )
    cmd[10] = loca;                         //位置管理
    cmd[11] = sync;
    cmd[12] =  0x6B;
    Send_Can(hfdcan,cmd, 13);
}
//直通限速位置模式控制
void ZDT_Control_Direct_Pos_Mode(FDCAN_HandleTypeDef *hfdcan,uint8_t addr,Dir dir,uint16_t vel, uint16_t pos, Loca_Manage loca, Sync sync) {
    uint8_t cmd[16] = {0};
    cmd[0] =  addr;
    cmd[1] =  0xFB;
    cmd[2] =  dir;                      //对位置的方向
    cmd[3] =  (uint8_t)(vel>>8);
    cmd[4] =  (uint8_t)(vel>>0);
    cmd[5] =  (uint8_t)(pos >> 24);      // 脉冲数(bit24 - bit31)
    cmd[6] =  (uint8_t)(pos >> 16);      // 脉冲数(bit16 - bit23)
    cmd[7] =  (uint8_t)(pos >> 8);       // 脉冲数(bit8  - bit15)
    cmd[8] =  (uint8_t)(pos >> 0);       // 脉冲数(bit0  - bit7 )
    cmd[9] = loca;                         //位置管理
    cmd[10] = sync;
    cmd[11] =  0x6B;
    Send_Can(hfdcan,cmd, 12);
}
//立即停止电机
void ZDT_Control_Stop_Motor(FDCAN_HandleTypeDef *hfdcan,uint8_t addr,Sync sync) {
    uint8_t cmd[16] = {0};
    cmd[0] =  addr;
    cmd[1] =  0xFE;
    cmd[2] =  0x98;
    cmd[3] =  sync;
    cmd[4] =  0x6B;
    Send_Can(hfdcan,cmd, 5);
}
//多机同步使能
void ZDT_Control_Sync_Mode(FDCAN_HandleTypeDef *hfdcan,uint8_t addr) {
    uint8_t cmd[16] = {0};
    cmd[0] =  addr;
    cmd[1] =  0xFF;
    cmd[2] =  0x66;
    cmd[3] =  0x6B;
    Send_Can(hfdcan,cmd, 4);
}
/**
  * @brief    修改回零参数
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    o_dir  ：回零方向，0为CW，其余值为CCW
  * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
  * @param    o_tm   ：回零超时时间，单位：毫秒
  * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
  * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
  * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
  * @param    potF   ：上电自动触发回零，false为不使能，true为使能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_Control_Origin_Modify_Params(FDCAN_HandleTypeDef *hfdcan,uint8_t addr, bool svF, O_Mode o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
    uint8_t cmd[32] = {0};

    // 装载命令
    cmd[0] =  addr;                       // 地址
    cmd[1] =  0x4C;                       // 功能码
    cmd[2] =  0xAE;                       // 辅助码
    cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
    cmd[4] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    cmd[5] =  o_dir;                      // 回零方向
    cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位字节
    cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位字节
    cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
    cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
    cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
    cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
    cmd[12] =  (uint8_t)(sl_vel >> 8);    // 无限位碰撞回零检测转速(RPM)高8位字节
    cmd[13] =  (uint8_t)(sl_vel >> 0);    // 无限位碰撞回零检测转速(RPM)低8位字节
    cmd[14] =  (uint8_t)(sl_ma >> 8);     // 无限位碰撞回零检测电流(Ma)高8位字节
    cmd[15] =  (uint8_t)(sl_ma >> 0);     // 无限位碰撞回零检测电流(Ma)低8位字节
    cmd[16] =  (uint8_t)(sl_ms >> 8);     // 无限位碰撞回零检测时间(Ms)高8位字节
    cmd[17] =  (uint8_t)(sl_ms >> 0);     // 无限位碰撞回零检测时间(Ms)低8位字节
    cmd[18] =  potF;                      // 上电自动触发回零，false为不使能，true为使能
    cmd[19] =  0x6B;                      // 校验字节

    // 发送命令
    Send_Can(hfdcan,cmd, 20);
}
/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    sync   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_Control_Origin_Trigger_Return(FDCAN_HandleTypeDef *hfdcan,uint8_t addr, uint8_t o_mode, Sync sync)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  addr;                       // 地址
    cmd[1] =  0x9A;                       // 功能码
    cmd[2] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    cmd[3] =  sync;                        // 多机同步运动标志
    cmd[4] =  0x6B;                       // 校验字节

    // 发送命令
    Send_Can(hfdcan,cmd,5);
}
/**
  * @brief    强制中断并退出回零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_Control_Origin_Interrupt(FDCAN_HandleTypeDef *hfdcan,uint8_t addr)
{
    uint8_t cmd[16] = {0};
    // 装载命令
    cmd[0] =  addr;                       // 地址
    cmd[1] =  0x9C;                       // 功能码
    cmd[2] =  0x48;                       // 辅助码
    cmd[3] =  0x6B;                       // 校验字节
    // 发送命令
   Send_Can(hfdcan,cmd,4);
}

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_Control_Read_Sys_Params(FDCAN_HandleTypeDef *hfdcan,uint8_t addr, SysParams_t s)
{
    uint8_t i = 0;
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[i] = addr; ++i;                   // 地址

    switch(s)                             // 功能码
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
        case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
        default: break;
    }

    cmd[i] = 0x6B; ++i;                   // 校验字节

    // 发送命令
     Send_Can(hfdcan,cmd, i);
}
//解析反馈帧
void ZDT_Control_Analyze_FDBack(ZDT_FBpara_t *motor,const uint8_t *rxdata,const uint32_t id) {
    if (rxdata==NULL) {
        return;
    }
    memset(motor, 0, sizeof(ZDT_FBpara_t));
    motor->Motor_Status.ERROR=0;
    motor->vaild = 0;
    motor->id = id;
    switch(rxdata[0]) {
        /******************************读取电机参数表**************************************/
        case S_PID:
            if (rxdata[13]==CHECK_SUM) {
                motor->S_pid.Pos_kp=(rxdata[1]<<24)|(rxdata[2]<<16)|(rxdata[3]<<8)|(rxdata[4]<<0);
                motor->S_pid.Pos_ki=(rxdata[5]<<24)|(rxdata[6]<<16)|(rxdata[7]<<8)|(rxdata[8]<<0);
                motor->S_pid.Pos_kd=(rxdata[9]<<24)|(rxdata[10]<<16)|(rxdata[11]<<8)|(rxdata[12]<<0);
                motor->vaild=1;
            }
            break;
        case S_CPHA:
            if (rxdata[3]==CHECK_SUM) {
                motor->S_Cpha = (rxdata[1]<<8)|rxdata[2];
                motor->vaild=1;
            }
        case S_VBUS:
            if (rxdata[3]==CHECK_SUM) {
                motor->S_vbus = (rxdata[1]<<8)|rxdata[2];
                motor->vaild=1;
            }
        case S_TPOS:
            if (rxdata[6]==CHECK_SUM) {
                if (rxdata[1] == CCW) motor->S_Tpos = (-1)*(float)((rxdata[2]<<24)|(rxdata[3]<<16)|(rxdata[4]<<8)|(rxdata[5]<<0))*360.0f/65536.0f;
                else motor->S_Tpos=(float)(((rxdata[2]<<24)|(rxdata[3]<<16)|(rxdata[4]<<8)|(rxdata[5]<<0)))*360.0f/65536.0f;
                motor->vaild=1;
            }
        case S_CPOS:
            if (rxdata[6]==CHECK_SUM) {
                if (rxdata[1]==CCW) motor->S_Cpos=(-1)*(float)(((rxdata[2]<<24)|(rxdata[3]<<16)|(rxdata[4]<<8)|(rxdata[5]<<0)))*360.0f/65536.0f;
                else motor->S_Cpos=(float)(((rxdata[2]<<24)|(rxdata[3]<<16)|(rxdata[4]<<8)|(rxdata[5]<<0)))*360.0f/65536.0f;
                motor->vaild=1;
            }
            break;
        case S_VEL:
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
        case S_PERR:
            if (rxdata[6]==CHECK_SUM) {
                if (rxdata[1]==CCW) motor->S_Perr= (-1)*(float)((rxdata[2]<<24)|(rxdata[3]<<16)|(rxdata[4]<<8)|(rxdata[5]<<0))*360.0f/65536.0f;
                else (float)((rxdata[2]<<24)|(rxdata[3]<<16)|(rxdata[4]<<8)|(rxdata[5]<<0))*360.0f/65536.0f;
                motor->vaild=1;
            }
        case S_TEMP:
            if (rxdata[3]==CHECK_SUM) {
                if (rxdata[1]==CCW) motor->S_Temp= (-1)*(float)rxdata[2];
                else motor->S_Temp=(float)rxdata[2];
                motor->vaild=1;
            }
        case S_FLAG:
            if (rxdata[2]==CHECK_SUM) {
                motor->S_Flag.IS_ENABLE=rxdata[1]&0x01;
                motor->S_Flag.IS_INPLACE=rxdata[1]&0x02;
                motor->S_Flag.IS_LOCKED=rxdata[1]&0x04;
                motor->S_Flag.IS_SAVE_LOCKED=rxdata[1]&0x08;
                motor->vaild = 1;
            }
       /*******************************读取电机命令反馈值**************************************/
        case ZDT_FUNC_ENABLE_MOTOR:
            if (rxdata[1]==0x02||rxdata[2]==0x6B) motor->Motor_Status.EN_status=Receive_Success;
            if (rxdata[1]==0xE2||rxdata[2]==0x6B) motor->Motor_Status.EN_status=Error_Parameter;
            break;
        case ZDT_FUNC_TOR_MODE:
            if (rxdata[1]==0x02||rxdata[2]==0x6B) motor->Motor_Status.Tor_Mode_status=Receive_Success;
            if (rxdata[1]==0xE2||rxdata[2]==0x6B) motor->Motor_Status.Tor_Mode_status=Error_Parameter;
            break;
        case ZDT_FUNC_VEL_MODE:
            if (rxdata[1]==0x02||rxdata[2]==0x6B) motor->Motor_Status.Vel_Mode_status=Receive_Success;
            if (rxdata[1]==0xE2||rxdata[2]==0x6B) motor->Motor_Status.Vel_Mode_status=Error_Parameter;
            break;
        case ZDT_FUNC_TRAPE_POS_MODE:
            if (rxdata[1]==0x02||rxdata[2]==0x6B) motor->Motor_Status.Trape_Pos_Val_Mode_status=Receive_Success;
            if (rxdata[1]==0xE2||rxdata[2]==0x6B) motor->Motor_Status.Trape_Pos_Val_Mode_status=Error_Parameter;
            break;
        case ZDT_FUNC_DIRECT_POS_MODE:
            if (rxdata[1]==0x02||rxdata[2]==0x6B) motor->Motor_Status.Direct_Pos_Mode_status=Receive_Success;
            if (rxdata[1]==0xE2||rxdata[2]==0x6B) motor->Motor_Status.Direct_Pos_Mode_status=Error_Parameter;
            break;
        case ZDT_FUNC_STOP_MOTOR:
            if (rxdata[1]==0x02||rxdata[2]==0x6B) motor->Motor_Status.Stop_Motor_status=Receive_Success;
            if (rxdata[1]==0xE2||rxdata[2]==0x6B) motor->Motor_Status.Stop_Motor_status=Error_Parameter;
            break;
        case ZDT_FUNC_SYNC_MODE:
            if (rxdata[1]==0x02||rxdata[2]==0x6B) motor->Motor_Status.Sync_Mode_status=Receive_Success;
            if (rxdata[1]==0xE2||rxdata[2]==0x6B) motor->Motor_Status.Sync_Mode_status=Error_Parameter;
            break;
        default:
            if (rxdata[0]==0x00&&rxdata[1]==0XEE&&rxdata[2]==0X6B)  motor->Motor_Status.ERROR=Error_Command;
            break;
    }
}
/***************************************************基本控制函数******************************************************/
//位置模式
void ZDT_MOTOR_POSITION(uint8_t addr,Dir dir,uint16_t acc, uint16_t vel_RPS, float angle) {
    uint16_t pos=P(angle);
    uint16_t vel=vel_RPS*60.0f;
    ZDT_Control_Trape_Pos_Mode(Motor_hfdcan,addr,dir,acc,vel,pos,ABSOLUTE_POS,SIN);
    //ZDT_Log_PrintCmdResult(addr, ZDT_FUNC_TRAPE_POS_MODE, 1);
}
//速度模式
void ZDT_MOTOR_VEL(uint8_t addr,Dir dir,uint16_t acc, uint16_t vel_RPS) {
    uint16_t vel=vel_RPS*60.0f;
    ZDT_Control_Vel_Mode(Motor_hfdcan,addr,dir,acc,vel,SIN);
}