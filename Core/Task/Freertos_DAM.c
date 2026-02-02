#include "Freertos_DAM.h"
#include "DM_Motor_LOG.h"
#include "fdcan.h"
#include "ZDT_Control.h"
#include "ZDT_MOTOR_LOG.h"

void Motor_Control(void *argument)
{
    int16_t control_motor;
    int16_t now_motor_state;
    /* USER CODE BEGIN Motor_Control */
    /* Infinite loop */
    //ZDT_MOTOR_POSITION(ZDT_MOTOR1,CW,6, 10, 19.10);
    ZDT_MOTOR_POSITION(ZDT_MOTOR3,CW,0.1, 0.5,38.79 );
    //save_pos_zero(&hfdcan_dam, MOTOR3, mit_mode);
    // DAM_MOTOR_MIT(MOTOR1 , 10, 1, 1);
    // DAM_MOTOR_MIT(MOTOR2 , 10, 1, 1);
    // DAM_MOTOR_MIT(MOTOR3 , 100, 1, 1);
    //ZDT_MOTOR_VEL(ZDT_MOTOR1,CW,2, 10);
    for(;;)
    {

        //DAM_MOTOR_MIT(MOTOR3 , 260, 0.2, 0.4);
        // if (osMessageQueueGet(&Target_PosHandle,&control_motor,0,osWaitForever)==osOK) {
        //     //获取到了具体角度\方向
        //     //控制电机运动到指定位置
        //     osMessageQueuePut(&Motor_StateHandle,&now_motor_state,0,osWaitForever);
        // }

        osDelay(500);
    }
    /* USER CODE END Motor_Control */
}

void Solve(void *argument)
{
    int16_t get_pos;
    int16_t control_motor;
    char put_message;


    /* USER CODE BEGIN Solve */
    /* Infinite loop */
    for(;;)
    {
        // if (osMessageQueueGet(&Solve_AngleHandle,&get_pos,0,osWaitForever)==osOK) {
        //     //传递目标了
        //     //开始计算
        //     osMessageQueuePut(&Target_PosHandle,&put_message,0,osWaitForever);
        // }
        osDelay(50);
    }
    /* USER CODE END Solve */
}

void Camera_Data(void *argument)
{
    int16_t pos;
    /* USER CODE BEGIN Camera_Data */
    /* Infinite loop */
    for(;;)
    {
        // osMessageQueuePut(&Solve_AngleHandle,&pos,0,osWaitForever);
        osDelay(100);
    }
    /* USER CODE END Camera_Data */
}

void Debug(void *argument)
{
    int16_t now_motor_state;
    /* USER CODE BEGIN Debug */
    /* Infinite loop */
    for(;;)
    {
        // if (osMessageQueueGet(&Motor_StateHandle,&now_motor_state,0,osWaitForever)==osOK) {}
        osDelay(200);
    }
    /* USER CODE END Debug */
}
void Log(void *argument)
{
    /* USER CODE BEGIN Log */
/**********************************获取电机反馈帧*********************************/
    ZDT_FBpara_t* Z[3] = {
        get_motor1(),
        get_motor2(),
        get_motor3()
    };
    motor_t* D[3]={
        DAM_get_motor1(),
        DAM_get_motor2(),
        DAM_get_motor3()
    };
    // 初始化延迟
    osDelay(100);

    OTTO_uart(&huart_debug,"✅/*******************电机日志任务启动*******************/");
#if DEBUG_MODE
    OTTO_uart(&huart_debug,"✅ 调试模式");
#else
    OTTO_uart(&huart_debug,"✅ 仅查看电机转速与位置");
#endif
    /* Infinite loop */
    for(;;)
    {
/**********************************步进电机状态打印*********************************/
#if DEBUG_MODE
        // 遍历所有电机，读取参数并打印日志
        for (uint8_t i = 0; i<1 ; i++)
        {
            uint8_t motor_id = Z[2]->id;

             ZDT_Control_Read_Sys_Params(&hfdcan_zdt, motor_id, S_VEL);    // 转速
             osDelay(delata);
             ZDT_Control_Read_Sys_Params(&hfdcan_zdt, motor_id, S_CPOS);   // 当前位置
             osDelay(delata);
             ZDT_Control_Read_Sys_Params(&hfdcan_zdt, motor_id, S_TEMP);  // 温度
             osDelay(delata);
             ZDT_Control_Read_Sys_Params(&hfdcan_zdt, motor_id, S_FLAG);  // 状态标志
             osDelay(delata);
             ZDT_Control_Read_Sys_Params(&hfdcan_zdt, motor_id, S_TPOS);
             osDelay(delata);
             ZDT_Control_Read_Sys_Params(&hfdcan_zdt, motor_id, S_PERR);
             osDelay(delata);
            ZDT_Control_Read_Sys_Params(&hfdcan_zdt, motor_id, S_VBUS);
             osDelay(delata);
             ZDT_Log_PrintMotorStatus(Z[2]);
        }
#else
        // 遍历所有电机，读取参数并打印日志
        for (uint8_t i = 0; i<ZDT_MOTOR_NUM ; i++)
        {
            ZDT_Log_PrintCmdResult(Z[2]);
        }
#endif
/********************************达妙电机状态打印*********************************/
        for (uint8_t i = 0; i<DAM_MOTOR_NUM ; i++)
        {
            DAM_Motor_ALL_State(D[2]);
        }
        //周期性延迟（固定频率执行）
        osDelay(10);
    }
    /* USER CODE END Log */
}
