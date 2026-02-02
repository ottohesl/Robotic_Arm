
#ifndef ZDT_MOTOR_LOG_H
#define ZDT_MOTOR_LOG_H
#include "ZDT_Control.h"
#include "FreeRTOS.h"
#include "main.h"
// 电机日志配置（可根据需求调整）
#define LOG_TASK_STACK_SIZE    1024
#define LOG_TASK_PRIORITY      2
#define LOG_REFRESH_INTERVAL   50    // 日志刷新间隔(ms)
#define TEMP_WARN_THRESHOLD    60.0f   // 温度告警阈值(℃)
#define VELOCITY_PRINT_PRECISION 2     // 转速打印精度
#define POSITION_PRINT_PRECISION 2     // 位置打印精度
void ZDT_Log_PrintMotorStatus(ZDT_FBpara_t *motor);
void ZDT_Log_PrintCmdResult(ZDT_FBpara_t *motor);
#endif //ZDT_MOTOR_LOG_H
