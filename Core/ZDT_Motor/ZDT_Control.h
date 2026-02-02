/**
  ******************************************************************************
  * @file           : ZDT_CONTROL.h
  * @brief          : ZDT系列电机CAN通信控制驱动头文件
  * @author         : ottohesl
  * @date           : 2026-02-01
  * @version        : V1.0
  * @note           : 该文件包含ZDT电机的控制指令定义、数据结构及函数声明，
  *                   支持位置、速度、力矩等多种控制模式及回零、参数读取等功能
  */
#ifndef ZDT_CONTROL_H
#define ZDT_CONTROL_H
#define OTTOHESL_H_Vision 7         //更改宏定义版本以适配不同芯片
#include <stdbool.h>
#include "ottohesl.h"

#if (OTTOHESL_H_Vision==1)
#include "stm32f1xx_hal.h"
#endif
#if (OTTOHESL_H_Vision==4)
#include "stm32f4xx_hal.h"
#endif
#if (OTTOHESL_H_Vision==7)
#include "stm32h7xx_hal.h"
#endif

/*******************************************************************************
 * 宏定义区
 ******************************************************************************/
#define deuge_mode      0                       /* 调试模式开关：0-关闭，1-开启 */
#define T               51200                   /* 电机转一圈所需脉冲数（电机最大脉冲为65536） */
#define MAX_ANGLE       460.8                   /* 电机角度最大值，超过该角度复位 */
#define P(angle)        T/360.0f*angle         /* 角度转脉冲数：angle-目标角度(°)，返回对应脉冲数 */
#define ZDT_MOTOR1      0x01                   /* 电机1地址 */
#define ZDT_MOTOR2      0x02                   /* 电机2地址 */
#define ZDT_MOTOR3      0x03                   /* 电机3地址 */
#define ZDT_ALL         0x00                   /* 所有电机地址 */
#define CHECK_SUM       0x6B                   /* CAN通信校验字节固定值 */

/*******************************************************************************
 * 枚举类型定义区
 ******************************************************************************/

/**
 * @brief 电机系统参数读取指令枚举
 * @note  用于指定读取电机的各类状态/参数类型
 */
typedef enum {
    S_VER   = 0x1F,        /* 读取固件版本和对应的硬件版本 */
    S_RL    = 0x20,        /* 读取相电阻和相电感 */
    S_PID   = 0x21,        /* 读取PID参数 */
    S_VBUS  = 0x24,        /* 读取总线电压 */
    S_CPHA  = 0x27,        /* 读取相电流 */
    S_ENCL  = 0x31,        /* 读取经过线性化校准后的编码器值 */
    S_TPOS  = 0x33,        /* 读取电机目标位置角度 */
    S_VEL   = 0x35,        /* 读取电机实时转速(RPM) */
    S_CPOS  = 0x36,        /* 读取电机实时位置角度 */
    S_PERR  = 0x37,        /* 读取电机位置误差角度 */
    S_TEMP  = 0x39,        /* 读取电机驱动器温度 */
    S_FLAG  = 0x3A,        /* 读取使能/到位/堵转状态标志位 */
    S_Conf  = 0x42,        /* 读取驱动参数 */
    S_STATE = 0x43,        /* 读取系统状态参数 */
    S_ORG   = 0x3B,        /* 读取正在回零/回零失败状态标志位 */
}SysParams_t;

/**
 * @brief 电机控制功能码枚举
 * @note  用于指定电机的各类控制指令类型
 */
typedef enum {
    // 基础控制类
    ZDT_FUNC_POS_CLEAR        = 0x0A,    /* 位置清零 */
    ZDT_FUNC_ENABLE_MOTOR     = 0xF3,    /* 电机使能/禁用 */
    ZDT_FUNC_TOR_MODE         = 0xF5,    /* 力矩控制模式 */
    ZDT_FUNC_VEL_MODE         = 0xF6,    /* 速度模式控制 */
    ZDT_FUNC_TRAPE_POS_MODE   = 0xFD,    /* 梯形曲线位置模式 */
    ZDT_FUNC_DIRECT_POS_MODE  = 0xFB,    /* 直通限速位置模式 */
    ZDT_FUNC_STOP_MOTOR       = 0xFE,    /* 立即停止电机 */
    ZDT_FUNC_SYNC_MODE        = 0xFF,    /* 多机同步使能 */

    // 回零相关类
    ZDT_FUNC_ORIGIN_MODIFY_PARAMS = 0x4C, /* 修改回零参数 */
    ZDT_FUNC_ORIGIN_TRIGGER_RETURN = 0x9A, /* 触发回零 */
    ZDT_FUNC_ORIGIN_INTERRUPT     = 0x9C, /* 中断退出回零 */

} ZDT_FuncCode_t;

/**
 * @brief 电机旋转方向枚举
 */
typedef enum {
    CW  = 0x00,   /* 顺时针方向(正方向) */
    CCW = 0x01,   /* 逆时针方向(负方向) */
}Dir;

/**
 * @brief 电机控制同步类型枚举
 * @note  区分单电机/多电机控制
 */
typedef enum {
    SIN = 0x00,     /* 单个电机控制 */
    ALL = 0x01,     /* 多个电机同步控制 */
}Sync;

/**
 * @brief 电机位置管理类型枚举
 * @note  区分相对位置/绝对位置控制
 */
typedef enum {
    RELATIVE_POS = 0x00,        /* 相对位置模式 */
    ABSOLUTE_POS = 0x01,        /* 绝对位置模式 */
}Loca_Manage;

/**
 * @brief 电机指令执行状态返回枚举
 */
typedef enum {
    Receive_Success = 1,         /* 命令接收成功 */
    Error_Command = 2,           /* 命令错误 */
    Error_Parameter = 3,         /* 参数条件错误 */
}Command_t;

/**
 * @brief 电机回零模式枚举
 */
typedef enum {
    SIN_NEAR = 0,           /* 单圈就近回零 */
    SIN_DIRE = 1,           /* 单圈方向回零 */
    MIU_COLLISION = 2,      /* 多圈无限位碰撞回零 */
    MIU_LIMIT_SWITCH = 3,   /* 多圈有限位开关回零 */
}O_Mode;

/*******************************************************************************
 * 结构体类型定义区
 ******************************************************************************/

/**
 * @brief PID参数反馈结构体
 * @note  存储位置环PID参数
 */
typedef struct {
    int Pos_kp;         /* 位置环比例系数(Kp) */
    int Pos_ki;         /* 位置环积分系数(Ki) */
    int Pos_kd;         /* 位置环微分系数(Kd) */
}FB_pid;

/**
 * @brief 电机状态标志位结构体
 * @note  存储电机各类状态标志
 */
typedef struct {
    bool IS_ENABLE;         /* 是否使能：true-使能，false-未使能 */
    bool IS_INPLACE;        /* 是否就位：true-到位，false-未到位 */
    bool IS_LOCKED;         /* 是否堵转：true-堵转，false-正常 */
    bool IS_SAVE_LOCKED;    /* 是否堵转保护：true-已保护，false-未保护 */
    bool IS_ENCL;           /* 是否编码器就位：true-就位，false-未就位 */
    bool IS_RIGHT;          /* 是否校准表就位：true-就位，false-未就位 */
    bool IS_RETURN;         /* 是否正在回零：true-回零中，false-未回零 */
    bool IS_RETURN_FAULT;   /* 是否回零失败：true-失败，false-成功/未执行 */
}Motor_Flag;

/**
 * @brief 电机工作模式状态结构体
 * @note  存储各控制模式的执行状态
 */
typedef struct {
    uint8_t ERROR;                          /* 通用错误码 */
    uint8_t EN_status;                      /* 使能指令执行状态 */
    uint8_t Vel_Mode_status;                /* 速度模式指令执行状态 */
    uint8_t Tor_Mode_status;                /* 力矩模式指令执行状态 */
    uint8_t Direct_Pos_Mode_status;         /* 直通限速位置模式指令执行状态 */
    uint8_t Trape_Pos_Val_Mode_status;      /* 梯形曲线位置模式指令执行状态 */
    uint8_t Stop_Motor_status;              /* 停止电机指令执行状态 */
    uint8_t Sync_Mode_status;               /* 多机同步模式指令执行状态 */
}Motor_Status;

/**
 * @brief 电机转速单位转换结构体
 * @note  存储不同单位的转速值
 */
typedef struct {
    float Vel_RPM;      /* 转速(转/分钟) */
    float Vel_RPS;      /* 转速(转/秒) */
}S_Vel;

/**
 * @brief 电机反馈数据总结构体
 * @note  整合所有电机反馈参数
 */
typedef struct {
    uint32_t id;                /* 电机地址ID */
    FB_pid S_pid;               /* PID参数 */
    int32_t S_vbus;             /* 总线电压(单位:mv) */
    int32_t S_Cpha;             /* 相电流(单位:ma) */
    float S_Tpos;               /* 目标位置角度(°) */
    S_Vel S_Vel;                /* 当前转速 */
    float S_Cpos;               /* 当前位置角度(°) */
    float S_Perr;               /* 位置误差角度(°) */
    float S_Temp;               /* 驱动器温度(℃) */
    Motor_Flag S_Flag;          /* 电机状态标志位 */
    bool vaild;                 /* 反馈帧是否有效：true-有效，false-无效 */
    bool IS_Receive;            /* 反馈帧是否接收完成：true-已接收，false-未接收 */
    Motor_Status Motor_Status;  /* 电机各模式执行状态 */
}ZDT_FBpara_t;

/*******************************************************************************
 * 函数声明区
 ******************************************************************************/

/**************************************基本服务函数***************************************/
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
void ZDT_Control_Read_ALL_Params(FDCAN_HandleTypeDef *hfdcan,uint8_t addr);
void ZDT_Control_Analyze_FDBack(ZDT_FBpara_t *motor,const uint8_t *rxdata,const uint32_t id);
/**************************************基本控制函数***************************************/
void ZDT_MOTOR_POSITION(uint8_t addr,Dir dir,uint16_t acc, float vel_RPS, float angle) ;
void ZDT_MOTOR_VEL(uint8_t addr,Dir dir,uint16_t acc, uint16_t vel_RPS);

#endif //ZDT_CONTROL_H