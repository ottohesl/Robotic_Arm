#include "DM_motor.h"

char message[128];
/**
 * @brief 通过FDCAN发送数据到指定设备
 * @param hfdcan FDCAN句柄指针
 * @param id 目标CAN ID
 * @param data 要发送的数据缓冲区
 * @param len 数据长度(字节)
 * @return uint8_t 发送状态: 0-成功, 1-失败
 * @note 支持标准CAN帧和FDCAN帧格式，自动处理数据长度编码
 */
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef txHeader;

    // 达妙电机使用标准CAN帧，只支持8字节数据
    if (len > 8) {
        int msg_len = sprintf(message, "Error: Data length %lu too long!\n", len);
        HAL_UART_Transmit(&huart_debug, (uint8_t *)message, msg_len, 100);
        return 1;
    }

    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;  // 固定8字节
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;   // 重要：关闭波特率切换，使用标准CAN
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;    // 重要：使用经典CAN格式
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    // 如果数据不足8字节，用0填充
    uint8_t send_data[8] = {0};
    memcpy(send_data, data, len);

    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, send_data);

    if(status != HAL_OK) {
        return 1;
    }
    return 0;
}

/**
 * @brief 解析电机反馈数据
 * @param motor 电机控制结构体指针
 * @param rx_data 接收到的原始数据缓冲区
 * @note 按照达妙电机通信协议解析8字节反馈数据包
 */
void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data)
{
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, -motor->P_MAX, motor->P_MAX, 16);
    motor->para.vel = uint_to_float(motor->para.v_int, -motor->V_MAX, motor->V_MAX, 12);
    motor->para.tor = uint_to_float(motor->para.t_int, -motor->T_MAX, motor->T_MAX, 12);
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
}

/**
 * @brief 浮点数转无符号整型
 * @param x_float 输入的浮点数
 * @param x_min 数值范围最小值
 * @param x_max 数值范围最大值
 * @param bits 输出整型的位数
 * @return int 转换后的整型数值
 * @note 用于将浮点控制参数转换为CAN通信所需的整型格式
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief 无符号整型转浮点数
 * @param x_int 输入的整型数
 * @param x_min 数值范围最小值
 * @param x_max 数值范围最大值
 * @param bits 输入整型的位数
 * @return float 转换后的浮点数值
 * @note 用于将CAN通信接收的整型数据转换为实际物理量
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 使能指定模式的电机控制
 * @param hcan FDCAN句柄指针
 * @param motor_id 电机ID
 * @param mode_id 控制模式ID
 * @note 发送特定数据包(0xFF...0xFC)使能电机对应模式
 */
void enable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;

    fdcanx_send_data(hcan, id, data, 8);
}

/**
 * @brief 失能指定模式的电机控制
 * @param hcan FDCAN句柄指针
 * @param motor_id 电机ID
 * @param mode_id 控制模式ID
 * @note 发送特定数据包(0xFF...0xFD)失能电机对应模式
 */
void disable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;

    fdcanx_send_data(hcan, id, data, 8);
}

/**
 * @brief 保存当前位置为零点
 * @param hcan FDCAN句柄指针
 * @param motor_id 电机ID
 * @param mode_id 控制模式ID
 * @note 发送特定数据包(0xFF...0xFE)设置当前位置为编码器零点
 */
void save_pos_zero(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFE;

    fdcanx_send_data(hcan, id, data, 8);
}

/**
 * @brief 清除电机错误状态
 * @param hcan FDCAN句柄指针
 * @param motor_id 电机ID
 * @param mode_id 控制模式ID
 * @note 发送特定数据包(0xFF...0xFB)清除电机故障状态
 */
void clear_err(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFB;

    fdcanx_send_data(hcan, id, data, 8);
}

/**
 * @brief 使能电机(根据当前设置的模式)
 * @param hcan FDCAN句柄指针
 * @param motor 电机控制结构体指针
 * @note 根据motor->ctrl.mode选择对应的使能命令
 */
void dm_motor_enable(FDCAN_HandleTypeDef* hcan, motor_t *motor)
{
    switch(motor->ctrl.mode)
    {
        case mit_mode:
            enable_motor_mode(hcan, motor->id, MIT_MODE);
            break;
        case pos_mode:
            enable_motor_mode(hcan, motor->id, POS_MODE);
            break;
        case spd_mode:
            enable_motor_mode(hcan, motor->id, SPD_MODE);
            break;
        case psi_mode:
            enable_motor_mode(hcan, motor->id, PSI_MODE);
            break;
    }
}

/**
 * @brief 失能电机并清除控制参数
 * @param hcan FDCAN句柄指针
 * @param motor 电机控制结构体指针
 * @note 失能当前模式并重置所有控制参数为零
 */
void dm_motor_disable(FDCAN_HandleTypeDef* hcan, motor_t *motor)
{
    switch(motor->ctrl.mode)
    {
        case mit_mode:
            disable_motor_mode(hcan, motor->id, MIT_MODE);
            break;
        case pos_mode:
            disable_motor_mode(hcan, motor->id, POS_MODE);
            break;
        case spd_mode:
            disable_motor_mode(hcan, motor->id, SPD_MODE);
            break;
        case psi_mode:
            disable_motor_mode(hcan, motor->id, PSI_MODE);
            break;
    }
    dm_motor_clear_para(motor);
}

/**
 * @brief 清除电机控制参数(归零)
 * @param motor 电机控制结构体指针
 * @note 将所有控制设定值重置为零，用于安全停止
 */
void dm_motor_clear_para(motor_t *motor)
{
    motor->ctrl.pos_set = 0;
    motor->ctrl.vel_set = 0;
    motor->ctrl.tor_set = 0;
    motor->ctrl.cur_set = 0;
    motor->ctrl.kp_set = 0;
    motor->ctrl.kd_set = 0;
}

/**
 * @brief 清除电机错误状态(根据当前模式)
 * @param hcan FDCAN句柄指针
 * @param motor 电机控制结构体指针
 * @note 根据当前控制模式发送对应的清除错误命令
 */
void dm_motor_clear_err(FDCAN_HandleTypeDef* hcan, motor_t *motor)
{
    switch(motor->ctrl.mode)
    {
        case mit_mode:
            clear_err(hcan, motor->id, MIT_MODE);
            break;
        case pos_mode:
            clear_err(hcan, motor->id, POS_MODE);
            break;
        case spd_mode:
            clear_err(hcan, motor->id, SPD_MODE);
            break;
        case psi_mode:
            clear_err(hcan, motor->id, PSI_MODE);
            break;
    }
}

/**
 * @brief 发送电机控制命令(根据当前模式)
 * @param hcan FDCAN句柄指针
 * @param motor 电机控制结构体指针
 * @note 根据设置的控制模式调用对应的控制函数发送命令
 */
void dm_motor_ctrl_send(FDCAN_HandleTypeDef* hcan, motor_t *motor)
{
    switch(motor->ctrl.mode)
    {
        case mit_mode:
            mit_ctrl(hcan, motor, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
            break;
        case pos_mode:
            pos_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
            break;
        case spd_mode:
            spd_ctrl(hcan, motor->id, motor->ctrl.vel_set);
            break;
        case psi_mode:
            psi_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.cur_set);
            break;
    }
}

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
 * @note MIT模式提供完整的PD位置控制+前馈扭矩控制
 */
void mit_ctrl(FDCAN_HandleTypeDef* hcan, motor_t *motor, uint16_t motor_id, float pos, float vel, float kp, float kd, float tor)
{
    uint8_t data[8];
    uint16_t id = motor_id + MIT_MODE;

    // 将浮点参数转换为CAN通信所需的整型格式
    uint16_t pos_tmp = float_to_uint(pos, -motor->P_MAX, motor->P_MAX, 16);
    uint16_t vel_tmp = float_to_uint(vel, -motor->V_MAX, motor->V_MAX, 12);
    uint16_t tor_tmp = float_to_uint(tor, -motor->T_MAX, motor->T_MAX, 12);
    uint16_t kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);

    // 按照达妙MIT模式协议打包数据
    data[0] = (pos_tmp >> 8) & 0xFF;
    data[1] = (pos_tmp & 0xFF);
    data[2] = ((vel_tmp >> 4)& 0xFF);
    data[3] = ((vel_tmp & 0xF) << 4) | ((kp_tmp >> 8) & 0xF);
    data[4] = (kp_tmp & 0xFF);
    data[5] = ((kd_tmp >> 4) & 0xFF);
    data[6] = ((kd_tmp & 0xF) << 4) | ((tor_tmp >> 8) & 0xF);
    data[7] = (tor_tmp & 0xFF);

    fdcanx_send_data(hcan, id, data, 8);
}

/**
 * @brief 位置模式控制命令发送
 * @param hcan FDCAN句柄指针
 * @param motor_id 电机ID
 * @param pos 目标位置(rad)
 * @param vel 限制速度(rad/s)
 * @note 位置模式下使用浮点数直接传输，支持梯形加减速
 */
void pos_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel)
{
    uint16_t id;
    uint8_t *pbuf, *vbuf;
    uint8_t data[8];

    id = motor_id + POS_MODE;
    pbuf = (uint8_t*)&pos;
    vbuf = (uint8_t*)&vel;

    // 浮点数按小端序转换为字节流
    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);

    data[4] = *vbuf;
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);

    fdcanx_send_data(hcan, id, data, 8);
}

/**
 * @brief 速度模式控制命令发送
 * @param hcan FDCAN句柄指针
 * @param motor_id 电机ID
 * @param vel 目标速度(rad/s)
 * @note 速度模式下只发送速度设定值，使用4字节浮点数
 */
void spd_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float vel)
{
    uint16_t id;
    uint8_t *vbuf;
    uint8_t data[4];

    id = motor_id + SPD_MODE;
    vbuf = (uint8_t*)&vel;

    // 速度值按小端序转换为字节流
    data[0] = *vbuf;
    data[1] = *(vbuf + 1);
    data[2] = *(vbuf + 2);
    data[3] = *(vbuf + 3);

    fdcanx_send_data(hcan, id, data, 4);
}

/**
 * @brief 电流模式控制命令发送
 * @param hcan FDCAN句柄指针
 * @param motor_id 电机ID
 * @param pos 目标位置(rad)
 * @param vel 限制速度(rad/s)
 * @param cur 目标电流(A)
 * @note 电流模式下位置和速度为限制条件，电流为主要控制量
 */
void psi_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel, float cur)
{
    uint16_t id;
    uint8_t *pbuf, *vbuf, *ibuf;
    uint8_t data[8];

    // 速度和电流进行缩放转换为整型
    uint16_t u16_vel = vel * 100;
    uint16_t u16_cur = cur * 10000;

    id = motor_id + PSI_MODE;
    pbuf = (uint8_t*)&pos;
    vbuf = (uint8_t*)&u16_vel;
    ibuf = (uint8_t*)&u16_cur;

    // 位置使用浮点数，速度和电流使用整型
    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);

    data[4] = *vbuf;
    data[5] = *(vbuf + 1);

    data[6] = *ibuf;
    data[7] = *(ibuf + 1);

    fdcanx_send_data(hcan, id, data, 8);
}
/**
 * @brief 电机初始化与参数重配置函数
 * @param motor 句柄指针
 * @param ser 电机地址枚举
 * @param position 目标位置(角度制)
 * @param tor 力矩
 * @note 只针对位置和力矩的重定义，如要设置kp，kd或者修改速度等，可直接进入函数配置；亦或者重新添加参数列表
 * 默认为mit模式，对于机械臂设置，就mit模式即可
 * kp=0的时候,设置位置无法生效，如果速度不为0，那么电机会直接旋转；所以设置为位置模式的时候必须给kp一个值
 * kp影响精度,kp越大那么电机振动越明显且力矩越大，但是会越接近设置的位置角度
 */
// void redefine_motor(motor_t *motor,motor_name ser,float position, float tor) {
//     float rad = position * M_PI / 180;
//     motor->id = ser;
//     motor->mst_id = MASTER;
//     motor->ctrl.pos_set  = rad;
//     motor->ctrl.tor_set  = tor;
//     motor->ctrl.vel_set  = 0.1;
//     motor->ctrl.kp_set   = 8.0f;
//     motor->ctrl.kd_set   = 1.0f;
//     motor->ctrl.cur_set  = 0.0f;
//     motor->P_MAX = PMAX_DEFAULT;
//     motor->V_MAX = VMAX_DEFAULT;
//     motor->T_MAX = TMAX_DEFAULT;
//     motor->ctrl.mode = MIT_MODE;
// }
void redefine_motor(motor_t *motor,motor_name ser,float position, float tor) {
    float rad = position * M_PI / 180;
    motor->id = ser;
    motor->mst_id = MASTER;
    motor->ctrl.pos_set  =rad;
    motor->ctrl.tor_set  = tor;
    motor->ctrl.vel_set  = 2.0f;
    motor->ctrl.kp_set   = 1.0f;
    motor->ctrl.kd_set   = 0.0f;
    motor->ctrl.cur_set  = 0.0f;
    motor->P_MAX = PMAX_DEFAULT;
    motor->V_MAX = VMAX_DEFAULT;
    motor->T_MAX = TMAX_DEFAULT;
    motor->ctrl.mode = SPD_MODE;
}