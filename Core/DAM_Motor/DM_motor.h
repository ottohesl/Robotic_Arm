#ifndef DM_MOTOR_H
#define DM_MOTOR_H
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "fdcan.h"
#include "usart.h"

/**
 * @brief 主机地址
 * 如果要自定义地址，上位机连接电机改地址就行
 */
#define MASTER  0x000
/**
 * @brief 电机控制模式常量定义
 * 建议使用mit模式，可以涵盖位置速度的控制模式
 * 位置速度控制模式有问题，mit模式够用了
 */
#define mit_mode 0      ///< MIT控制模式
#define pos_mode 256      ///< 位置控制模式
#define spd_mode 512      ///< 速度控制模式
#define psi_mode 768      ///< 电流控制模式

/**
 * @brief 电机参数范围默认值定义
 */
#define PMAX_DEFAULT 6.28f    ///< 默认位置最大值(rad)
#define VMAX_DEFAULT 30.0f    ///< 默认速度最大值(rad/s)
#define TMAX_DEFAULT 10.0f    ///< 默认扭矩最大值(N·m)
#define KP_MIN 0.0f           ///< Kp最小值
#define KP_MAX 500.0f         ///< Kp最大值
#define KD_MIN 0.0f           ///< Kd最小值
#define KD_MAX 5.0f           ///< Kd最大值
/**
 * @brief 从机地址
 * 如果要自定义地址，上位机连接电机改地址就行
 */
typedef enum {
    MOTOR1 = 0x01,
    MOTOR2 = 0x02,
    MOTOR3 = 0x03,
    MOTOR4 = 0x04,
}motor_name;
/**
 * @brief 电机工作模式枚举
 * @note CAN ID偏移量定义
 */
typedef enum {
    MIT_MODE = 0x000,    ///< MIT模式，CAN ID偏移0
    POS_MODE = 0x100,    ///< 位置模式，CAN ID偏移0x100
    SPD_MODE = 0x200,    ///< 速度模式，CAN ID偏移0x200
    PSI_MODE = 0x300     ///< 电流模式，CAN ID偏移0x300
} motor_mode_t;
/**
 * @brief 电机状态枚举
 * @note 定义枚举
 */
typedef enum{
    MOTOR_ENABLE = 1,
    MOTOR_DISABLE = 0,
}motor_state;
/**
 * @brief 电机反馈参数结构体
 * 存储从电机接收到的状态反馈数据
 */
typedef struct {
    int id;                ///< 电机CAN ID
    int state;             ///< 电机状态标志
    int p_int;             ///< 整型位置原始数据
    int v_int;             ///< 整型速度原始数据
    int t_int;             ///< 整型扭矩原始数据
    int kp_int;            ///< 整型Kp原始数据
    int kd_int;            ///< 整型Kd原始数据
    float pos;             ///< 解析后的位置值(rad)
    float vel;             ///< 解析后的速度值(rad/s)
    float tor;             ///< 解析后的扭矩值(N·m)
    float Kp;              ///< 解析后的Kp参数
    float Kd;              ///< 解析后的Kd参数
    float Tmos;            ///< 驱动器MOS温度(℃)
    float Tcoil;           ///< 电机线圈温度(℃)
} motor_fbpara_t;

/**
 * @brief 电机控制参数结构体
 * 存储发送给电机的控制指令参数
 */
typedef struct {
    uint32_t mode;          ///< 当前控制模式
    float pos_set;         ///< 目标位置设定值(rad)
    float vel_set;         ///< 目标速度设定值(rad/s)
    float tor_set;         ///< 目标扭矩设定值(N·m)
    float cur_set;         ///< 目标电流设定值(A)
    float kp_set;          ///< Kp参数设定值
    float kd_set;          ///< Kd参数设定值
} motor_ctrl_t;

/**
 * @brief 电机完整控制结构体
 * 包含电机的所有控制状态和参数
 */
typedef struct {
    uint16_t id;           ///< 电机CAN通信ID
    uint16_t mst_id;       ///< 主控制器ID，用于标识
    motor_fbpara_t para;   ///< 电机反馈参数
    motor_ctrl_t ctrl;     ///< 电机控制参数
    float P_MAX;           ///< 位置控制最大范围(rad)
    float V_MAX;           ///< 速度控制最大范围(rad/s)
    float T_MAX;           ///< 扭矩控制最大范围(N·m)
} motor_t;

/**
 * @brief 通过FDCAN发送数据到指定设备
 * @param hfdcan FDCAN句柄指针
 * @param id 目标CAN ID
 * @param data 要发送的数据缓冲区
 * @param len 数据长度(字节)
 * @return uint8_t 发送状态: 0-成功, 1-失败
 */
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);

/**
 * @brief 解析电机反馈数据
 * @param motor 电机控制结构体指针
 * @param rx_data 接收到的原始数据缓冲区
 */
void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data);

/**
 * @brief 浮点数转无符号整型
 * @param x_float 输入的浮点数
 * @param x_min 数值范围最小值
 * @param x_max 数值范围最大值
 * @param bits 输出整型的位数
 * @return int 转换后的整型数值
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits);

/**
 * @brief 无符号整型转浮点数
 * @param x_int 输入的整型数
 * @param x_min 数值范围最小值
 * @param x_max 数值范围最大值
 * @param bits 输入整型的位数
 * @return float 转换后的浮点数值
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits);

/**
 * @brief 使能指定模式的电机控制
 * @param hcan FDCAN句柄指针
 * @param motor_id 电机ID
 * @param mode_id 控制模式ID
 */
void enable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);

/**
 * @brief 失能指定模式的电机控制
 * @param hcan FDCAN句柄指针
 * @param motor_id 电机ID
 * @param mode_id 控制模式ID
 */
void disable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);

/**
 * @brief 保存当前位置为零点
 * @param hcan FDCAN句柄指针
 * @param motor_id 电机ID
 * @param mode_id 控制模式ID
 */
void save_pos_zero(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);

/**
 * @brief 清除电机错误状态
 * @param hcan FDCAN句柄指针
 * @param motor_id 电机ID
 * @param mode_id 控制模式ID
 */
void clear_err(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);

/**
 * @brief 使能电机(根据当前设置的模式)
 * @param hcan FDCAN句柄指针
 * @param motor 电机控制结构体指针
 */
void dm_motor_enable(FDCAN_HandleTypeDef* hcan, motor_t *motor);

/**
 * @brief 失能电机并清除控制参数
 * @param hcan FDCAN句柄指针
 * @param motor 电机控制结构体指针
 */
void dm_motor_disable(FDCAN_HandleTypeDef* hcan, motor_t *motor);

/**
 * @brief 清除电机控制参数(归零)
 * @param motor 电机控制结构体指针
 */
void dm_motor_clear_para(motor_t *motor);

/**
 * @brief 清除电机错误状态(根据当前模式)
 * @param hcan FDCAN句柄指针
 * @param motor 电机控制结构体指针
 */
void dm_motor_clear_err(FDCAN_HandleTypeDef* hcan, motor_t *motor);

/**
 * @brief 发送电机控制命令(根据当前模式)
 * @param hcan FDCAN句柄指针
 * @param motor 电机控制结构体指针
 */
void dm_motor_ctrl_send(FDCAN_HandleTypeDef* hcan, motor_t *motor);

/**
 * @brief MIT模式控制命令发送
 * @param hcan FDCAN句柄指针
 * @param motor 电机控制结构体指针
 * @param motor_id 电机ID
 * @param pos 目标位置(rad)
 * @param vel 目标速度(rad/s)
 * @param kp 位置比例系数
 * @param kd 位置微分系数
 * @param tor 前馈扭矩(N·m)
 */
void mit_ctrl(FDCAN_HandleTypeDef* hcan, motor_t *motor, uint16_t motor_id, float pos, float vel, float kp, float kd, float tor);

/**
 * @brief 位置模式控制命令发送
 * @param hcan FDCAN句柄指针
 * @param motor_id 电机ID
 * @param pos 目标位置(rad)
 * @param vel 限制速度(rad/s)
 */
void pos_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel);

/**
 * @brief 速度模式控制命令发送
 * @param hcan FDCAN句柄指针
 * @param motor_id 电机ID
 * @param vel 目标速度(rad/s)
 */
void spd_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float vel);

/**
 * @brief 电流模式控制命令发送
 * @param hcan FDCAN句柄指针
 * @para
 * @param pos 目标位置(rad)
 * @param vel 限制速度(rad/s)
 * @param cur 目标电流(A)
 */
void psi_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel, float cur);
void redefine_motor(motor_t *motor,motor_name ser,float position, float tor);
//extern motor_t motor1, motor2, motor3, motor4;

#endif //DM_MOTOR_H