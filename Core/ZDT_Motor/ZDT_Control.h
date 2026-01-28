#ifndef ZDT_CONTROL_H
#define ZDT_CONTROL_H
#include <stdbool.h>
#include "ottohesl.h"
#include "stm32h7xx_hal.h"

#define deuge_mode 0
#define   T          3200               //转一圈所需脉冲数
#define   P(angle)   T/360.0f*angle     //转到angle度所需的脉冲
#define ZDT_MOTOR1  0x01
#define ZDT_MOTOR2  0x02
#define ZDT_MOTOR3  0x03
#define ZDT_ALL     0x00
#define CHECK_SUM   0x6B
/**********************电机枚举定义*******************************/
//电机参数读取表
typedef enum {
    S_VER   = 0x1F,			/* 读取固件版本和对应的硬件版本 */
    S_RL    = 0x20,			/* 读取读取相电阻和相电感 */
    S_PID   = 0x21,			/* 读取PID参数 */
    S_VBUS  = 0x24,			/* 读取总线电压 */
    S_CPHA  = 0x27,			/* 读取相电流 */
    S_ENCL  = 0x31,			/* 读取经过线性化校准后的编码器值 */
    S_TPOS  = 0x33,			/* 读取电机目标位置角度 */
    S_VEL   = 0x35,			/* 读取电机实时转速(RPM) */
    S_CPOS  = 0x36,			/* 读取电机实时位置角度 */
    S_PERR  = 0x37,			/* 读取电机位置误差角度 */
    S_TEMP  = 0x39,         /* 读取电机驱动器温度 */
    S_FLAG  = 0x3A,			/* 读取使能/到位/堵转状态标志位 */
    S_Conf  = 0x42,			/* 读取驱动参数 */
    S_State = 0x43,			/* 读取系统状态参数 */
    S_ORG   = 0x3B,           /* 读取正在回零/回零失败状态标志位 */
}SysParams_t;
//电机控制参数表
typedef enum {
    // 基础控制类
    ZDT_FUNC_POS_CLEAR        = 0x0A,    // 位置清零
    ZDT_FUNC_ENABLE_MOTOR     = 0xF3,    // 电机使能/禁用
    ZDT_FUNC_TOR_MODE         = 0xF5,    // 力矩控制模式
    ZDT_FUNC_VEL_MODE         = 0xF6,    // 速度模式控制
    ZDT_FUNC_TRAPE_POS_MODE   = 0xFD,    // 梯形曲线位置模式
    ZDT_FUNC_DIRECT_POS_MODE  = 0xFB,    // 直通限速位置模式
    ZDT_FUNC_STOP_MOTOR       = 0xFE,    // 立即停止电机
    ZDT_FUNC_SYNC_MODE        = 0xFF,    // 多机同步使能

    // 回零相关类
    ZDT_FUNC_ORIGIN_MODIFY_PARAMS = 0x4C, // 修改回零参数
    ZDT_FUNC_ORIGIN_TRIGGER_RETURN = 0x9A, // 触发回零
    ZDT_FUNC_ORIGIN_INTERRUPT     = 0x9C, // 中断退出回零

    // 读取参数类（与SysParams_t一一对应）
    ZDT_FUNC_READ_VER      = 0x1F,    // 读取版本信息
    ZDT_FUNC_READ_RL       = 0x20,    // 读取限位信息
    ZDT_FUNC_READ_PID      = 0x21,    // 读取PID参数
    ZDT_FUNC_READ_VBUS     = 0x24,    // 读取总线电压
    ZDT_FUNC_READ_CPHA     = 0x27,    // 读取编码器相位
    ZDT_FUNC_READ_ENCL     = 0x31,    // 读取编码器线数
    ZDT_FUNC_READ_TPOS     = 0x33,    // 读取目标位置
    ZDT_FUNC_READ_VEL      = 0x35,    // 读取速度
    ZDT_FUNC_READ_CPOS     = 0x36,    // 读取当前位置
    ZDT_FUNC_READ_PERR     = 0x37,    // 读取位置误差
    ZDT_FUNC_READ_FLAG     = 0x3A,    // 读取状态标志
    ZDT_FUNC_READ_ORG      = 0x3B,    // 读取回零状态
    ZDT_FUNC_READ_CONF     = 0x42,    // 读取配置参数
    ZDT_FUNC_READ_STATE    = 0x43     // 读取运行状态
} ZDT_FuncCode_t;
//正、负方向定义枚举
typedef enum {
    CW  = 0x00,   //正
    CCW = 0x01,  //负
}Dir;
//单电机运转与多电机运转枚举定义
typedef enum {
    SIN = 0x00,     //单个电机
    ALL = 0x01,     //多个电机
}Sync;
//电机位置参数枚举定义
typedef enum {
    RELATIVE_POS = 0x00,        //相对位置
    ABSOLUTE_POS = 0x01,        //绝对位置
}Loca_Manage;
//返回参数命令枚举
typedef enum {
    Receive_Success = 1,         //命令接受成功
    Error_Command = 2,           //命令错误
    Error_Parameter = 3,         //参数条件错误
}Command_t;
//电机回零参数枚举
typedef enum {
    SIN_NEAR = 0,           //单圈就近回零
    SIN_DIRE = 1,           //单圈方向回零
    MIU_COLLISION = 2,      //多圈无限位碰撞回零
    MIU_LIMIT_SWITCH = 3,   //多圈有限位开关回零
}O_Mode;

/**********************电机结构体定义*******************************/
//位置环反馈帧结构体
typedef struct {
    int Pos_kp;         //位置环kp
    int Pos_ki;         //位置环ki
    int Pos_kd;         //位置环kd
}FB_pid;
//电机标志位读取结构体
typedef struct {
    bool IS_ENABLE;         //是否使能
    bool IS_INPLACE;        //是否就位
    bool IS_LOCKED;         //是否堵转
    bool IS_SAVE_LOCKED;    //是否堵转保护
}Motor_Flag;
//电机各模式状态结构体
typedef struct {
    uint8_t ERROR;
    uint8_t EN_status;
    uint8_t Vel_Mode_status;
    uint8_t Tor_Mode_status;
    uint8_t Direct_Pos_Mode_status;         //直通限速位置模式
    uint8_t Trape_Pos_Val_Mode_status;
    uint8_t Stop_Motor_status;
    uint8_t Sync_Mode_status;
}Motor_Status;
//接受反馈帧储存转速单位结构体
typedef struct {
    float Vel_RPM;      //单位 转/分钟
    float Vel_RPS;      //单位 转/秒
}S_Vel;
//电机反馈帧结构体
typedef struct {
    uint32_t id;                //电机地址
    FB_pid S_pid;                //pid参数
    int32_t S_vbus;             //总线电压,单位mv
    int32_t S_Cpha;             //相电流，单位ma
    float S_Tpos;             //目标位置角度
    S_Vel S_Vel;              //当前转速
    float S_Cpos;             //当前位置角度
    float S_Perr;             //位置误差角度
    float S_Temp;              //驱动器温度
    Motor_Flag S_Flag;           //读取使能/到位/堵转状态标志位
    bool vaild;             //反馈帧是否有效
    Motor_Status Motor_Status;
}ZDT_FBpara_t;


void Send_Can(FDCAN_HandleTypeDef *hfdcan,uint8_t* data , uint8_t len);
void ZDT_Control_Pos_Clear(FDCAN_HandleTypeDef *hfdcan,uint8_t addr);
void ZDT_Control_Enable_Motor(FDCAN_HandleTypeDef *hfdcan,uint8_t addr, bool state, Sync sync);
void ZDT_Control_Tor_Mode(FDCAN_HandleTypeDef *hfdcan,uint8_t addr,Dir dir, uint16_t ele_slope,uint16_t ele, Sync sync);
void ZDT_Control_Vel_Mode(FDCAN_HandleTypeDef *hfdcan,uint8_t addr,Dir dir, uint16_t vel_slope,uint16_t vel, Sync sync);
void ZDT_Control_Trape_Pos_Mode(FDCAN_HandleTypeDef *hfdcan,uint8_t addr,Dir dir,uint16_t acc_vel, uint16_t max_vel, uint16_t pos, Loca_Manage loca,Sync sync);
void ZDT_Control_Direct_Pos_Mode(FDCAN_HandleTypeDef *hfdcan,uint8_t addr,Dir dir,uint16_t vel, uint16_t pos, Loca_Manage loca, Sync sync);
void ZDT_Control_Stop_Motor(FDCAN_HandleTypeDef *hfdcan,uint8_t addr,Sync sync);
void ZDT_Control_Sync_Mode(FDCAN_HandleTypeDef *hfdcan,uint8_t addr) ;
void ZDT_Control_Origin_Modify_Params(FDCAN_HandleTypeDef *hfdcan,uint8_t addr, bool svF, O_Mode o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
void ZDT_Control_Origin_Trigger_Return(FDCAN_HandleTypeDef *hfdcan,uint8_t addr, uint8_t o_mode, Sync sync);
void ZDT_Control_Origin_Interrupt(FDCAN_HandleTypeDef *hfdcan,uint8_t addr);
void ZDT_Control_Read_Sys_Params(FDCAN_HandleTypeDef *hfdcan,uint8_t addr, SysParams_t s);
void ZDT_Control_Analyze_FDBack(ZDT_FBpara_t *motor,const uint8_t *rxdata,const uint32_t id);

void ZDT_MOTOR_POSITION(uint8_t addr,Dir dir,uint16_t acc, uint16_t vel, float angle) ;
void ZDT_MOTOR_VEL(uint8_t addr,Dir dir,uint16_t acc, uint16_t vel_RPS);

#endif //ZDT_CONTROL_H
