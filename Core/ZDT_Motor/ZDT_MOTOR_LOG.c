#include "ZDT_MOTOR_LOG.h"


/************************* 私有函数声明 *************************/
static void ZDT_Log_PrintFuncCode(uint8_t func_code);
static void ZDT_Log_PrintControlStatus(ZDT_FBpara_t *motor);
static void ZDT_Log_PrintSysParams(ZDT_FBpara_t *motor);
static void ZDT_Log_CheckSafetyStatus(ZDT_FBpara_t *motor);
static void ZDT_Log_UART_Printf(const char *format, ...);

/************************* 串口日志核心函数 *************************/
/**
 * @brief  FreeRTOS安全的串口打印函数（带互斥锁）
 * @param  format: 格式化字符串
 * @param  ...: 可变参数
 */
static void ZDT_Log_UART_Printf(const char *format, ...)
{
    if (uart_log_mutexHandle == NULL) return;
    static char message[128];
    // 获取串口互斥锁
    osMutexAcquire(uart_log_mutexHandle, osWaitForever);
    va_list args;
    va_start(args, format);
    int len = vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    /* 2. 检查格式化结果：长度异常则发送错误提示 */
    if (len < 0 || len >= sizeof(message)) {
        HAL_UART_Transmit(&huart_debug, (uint8_t *)"length error\n", strlen("length error\n"), 100);
        return;
    }

    /* 3. 末尾添加换行符（确保缓存有剩余空间） */
    if (len + 1 < sizeof(message)) {
        message[len] = '\n';
        len++;  // 换行符计入最终发送长度
    }
    // 释放互斥锁
    HAL_UART_Transmit(&huart_debug, (uint8_t *) message, len, 100);
    osMutexRelease(uart_log_mutexHandle);
}

/**
 * @brief  功能码转文字描述（核心映射）
 * @param  func_code: 功能码（枚举值）
 */
static void ZDT_Log_PrintFuncCode(uint8_t func_code)
{
    switch(func_code)
    {
        case ZDT_FUNC_POS_CLEAR:          ZDT_Log_UART_Printf("位置清零"); break;
        case ZDT_FUNC_ENABLE_MOTOR:       ZDT_Log_UART_Printf("电机使能/禁用"); break;
        case ZDT_FUNC_TOR_MODE:           ZDT_Log_UART_Printf("力矩控制模式"); break;
        case ZDT_FUNC_VEL_MODE:           ZDT_Log_UART_Printf("速度模式控制"); break;
        case ZDT_FUNC_TRAPE_POS_MODE:     ZDT_Log_UART_Printf("梯形曲线位置模式"); break;
        case ZDT_FUNC_DIRECT_POS_MODE:    ZDT_Log_UART_Printf("直通限速位置模式"); break;
        case ZDT_FUNC_STOP_MOTOR:         ZDT_Log_UART_Printf("立即停止电机"); break;
        case ZDT_FUNC_SYNC_MODE:          ZDT_Log_UART_Printf("多机同步使能"); break;
        case ZDT_FUNC_ORIGIN_TRIGGER_RETURN: ZDT_Log_UART_Printf("触发回零"); break;
        default:                          ZDT_Log_UART_Printf("未知功能(0x%02X)", func_code); break;
    }
}

/**
 * @brief  打印控制命令执行状态
 * @param  motor: 电机反馈结构体指针
 */
static void ZDT_Log_PrintControlStatus(ZDT_FBpara_t *motor)
{
    //if (!motor->vaild) return;

    ZDT_Log_UART_Printf("========= 电机[0x%02X]控制状态 =========", motor->id);

    // 遍历控制状态枚举，输出对应状态
    if (motor->Motor_Status.EN_status == Receive_Success) {
        ZDT_Log_UART_Printf("✅ %s: 执行成功", "电机使能/禁用");
    } else if (motor->Motor_Status.EN_status == Error_Parameter) {
        ZDT_Log_UART_Printf("❌ %s: 参数错误", "电机使能/禁用");
    }

    if (motor->Motor_Status.Vel_Mode_status == Receive_Success) {
        ZDT_Log_UART_Printf("✅ %s: 执行成功", "速度模式控制");
    } else if (motor->Motor_Status.Vel_Mode_status == Error_Parameter) {
        ZDT_Log_UART_Printf("❌ %s: 参数错误", "速度模式控制");
    }

    if (motor->Motor_Status.Trape_Pos_Val_Mode_status == Receive_Success) {
        ZDT_Log_UART_Printf("✅ %s: 执行成功", "位置模式");
    } else if (motor->Motor_Status.Trape_Pos_Val_Mode_status == Error_Parameter) {
        ZDT_Log_UART_Printf("❌ %s: 参数错误", "位置模式");
    }

    if (motor->Motor_Status.ERROR == Error_Command) {
        ZDT_Log_UART_Printf("❌ 电机[0x%02X]: 接收到错误命令", motor->id);
    }
}

/**
 * @brief  打印电机系统参数（转速/位置/电压等）
 * @param  motor: 电机反馈结构体指针
 */
static void ZDT_Log_PrintSysParams(ZDT_FBpara_t *motor)
{
    //if (!motor->vaild) return;

    ZDT_Log_UART_Printf("========= 电机[0x%02X]系统参数 =========", motor->id);

    // 总线电压
    ZDT_Log_UART_Printf("🔋 总线电压: %.1f V", motor->S_vbus / 1000.0f);

    // 转速（RPM/RPS）
    ZDT_Log_UART_Printf("⚡ 实时转速: %.2f RPM (%.2f RPS)",
                       motor->S_Vel.Vel_RPM, motor->S_Vel.Vel_RPS);

    // 当前位置
    ZDT_Log_UART_Printf("📍 当前位置: %.2f °", motor->S_Cpos);

    // 目标位置
    ZDT_Log_UART_Printf("🎯 目标位置: %.2f °", motor->S_Tpos);

    // 位置误差
    ZDT_Log_UART_Printf("📏 位置误差: %.2f °", motor->S_Perr);

    // 驱动器温度
    ZDT_Log_UART_Printf("🌡️ 驱动器温度: %.1f °C", motor->S_Temp);

    // PID参数
    ZDT_Log_UART_Printf("🎛️ PID参数 - KP:%d, KI:%d, KD:%d",
                       motor->S_pid.Pos_kp, motor->S_pid.Pos_ki, motor->S_pid.Pos_kd);
}

/**
 * @brief  安全状态检测（堵转/高温告警）
 * @param  motor: 电机反馈结构体指针
 */
static void ZDT_Log_CheckSafetyStatus(ZDT_FBpara_t *motor)
{
    //if (!motor->vaild) return;

    ZDT_Log_UART_Printf("========= 电机[0x%02X]安全状态 =========", motor->id);

    // 堵转检测
    if (motor->S_Flag.IS_LOCKED) {
        ZDT_Log_UART_Printf("⚠️ 【告警】电机堵转！");
    } else {
        ZDT_Log_UART_Printf("✅ 无堵转");
    }

    // 堵转保护
    if (motor->S_Flag.IS_SAVE_LOCKED) {
        ZDT_Log_UART_Printf("⚠️ 【告警】电机触发堵转保护！");
    }

    // 温度告警
    if (motor->S_Temp > TEMP_WARN_THRESHOLD) {
        ZDT_Log_UART_Printf("⚠️ 【告警】驱动器温度过高(%.1f°C)！", motor->S_Temp);
    } else {
        ZDT_Log_UART_Printf("✅ 温度正常");
    }

    // 使能状态
    ZDT_Log_UART_Printf("🔌 电机使能状态: %s", motor->S_Flag.IS_ENABLE ? "已使能" : "未使能");

    // 到位状态
    ZDT_Log_UART_Printf("🎯 电机到位状态: %s", motor->S_Flag.IS_INPLACE ? "已到位" : "未到位");
}

/************************* 对外接口函数 *************************/

/**
 * @brief  打印单台电机的完整状态（控制+参数+安全）
 * @param  motor: 电机反馈结构体指针
 */
void ZDT_Log_PrintMotorStatus(ZDT_FBpara_t *motor)
{
    if (motor == NULL) return;

    ZDT_Log_PrintControlStatus(motor);  // 控制状态
    ZDT_Log_PrintSysParams(motor);      // 系统参数
    ZDT_Log_CheckSafetyStatus(motor);   // 安全状态
    ZDT_Log_UART_Printf("=====================================");
    motor->vaild=0;
}

/**
 * @brief  打印控制命令执行结果（发送命令后调用）
 * @param  motor_id: 电机地址
 * @param  func_code: 功能码（枚举）
 * @param  status: 执行状态（Receive_Success/Error_Parameter等）
 */
void ZDT_Log_PrintCmdResult(uint8_t motor_id, ZDT_FuncCode_t func_code, Command_t status)
{
    ZDT_Log_UART_Printf("📢 电机:0x%02X ", motor_id);
    ZDT_Log_PrintFuncCode(func_code);

    if (status == Receive_Success) {
        ZDT_Log_UART_Printf(": 执行成功 ✅");
    } else if (status == Error_Parameter) {
        ZDT_Log_UART_Printf(": 参数错误 ❌");
    } else if (status == Error_Command) {
        ZDT_Log_UART_Printf(": 命令错误 ❌");
    }
}

