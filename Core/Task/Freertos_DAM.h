
#ifndef FREERTOS_DAM_H
#define FREERTOS_DAM_H
#include <stdbool.h>
#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"

#define DEBUG_MODE 0
#define DAM_MOTOR_NUM 3
#define ZDT_MOTOR_NUM 3
#define delata        50
// typedef struct {
//     Pose Target_Pos;        //目标位置
//     uint32_t time_tamp;     //时间戳
//     bool Valid_Pos;         //有效位置标识
// }Target_Pose_Msg;
//
// typedef struct {
//     JointAngles angles;     //6轴关节角度
//     bool Valid_Angles;      //关节角度是否有效
//     bool Valid_Solve;       //运动学是否求解成功
// }Joint_Angle_Msg;

typedef struct {
    float actual_joint[6];     // 电机实际反馈的关节角度（度，方便调试查看）
    float motor_temp[6];       // 6轴电机线圈温度
    uint8_t motor_state[6];    // 6轴电机工作状态（0：故障，1：正常）
    uint8_t solve_success;     // 对应的运动学求解状态（透传，方便调试关联）
} Motor_State_Msg;









#endif //FREERTOS_DAM_H
