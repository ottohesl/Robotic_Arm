#include "Freertos_DAM.h"
void Motor_Control(void *argument)
{
    int16_t control_motor;
    int16_t now_motor_state;
    float pos=0;
    /* USER CODE BEGIN Motor_Control */
    /* Infinite loop */

    for(;;)
    {
        // if (i<60) {
        //     ZDT_MOTOR_POSITION(ZDT_MOTOR2,CW,0.2, 0.1, i);
        //     i++;
        // }
        // DAM_MOTOR_POS(MOTOR1 ,-60, 0.1);
        // DAM_MOTOR_POS(MOTOR2 ,-30, 0.1);
        // DAM_MOTOR_POS(MOTOR3 ,-50, 0.1);
        // osDelay(1000);
        //DAM_MOTOR_MIT(MOTOR3 , 260, 0.2, 0.4);
        // if (osMessageQueueGet(&Target_PosHandle,&control_motor,0,osWaitForever)==osOK) {
        //     //获取到了具体角度\方向
        //     //控制电机运动到指定位置
        //     osMessageQueuePut(&Motor_StateHandle,&now_motor_state,0,osWaitForever);
        // }

        osDelay(50);
    }
    /* USER CODE END Motor_Control */
}
void Solve(void *argument)
{
      //Joints_FK(63.43, 40.47 ,-70.16   ,0,29.96,-63.66);
    //  Joints_FK(90.00,17.23,-24.89,0,7.67,0);
    // Joints_IK_Debug(0.0f, 200, 200.0f, 0.0f, 0.0f, 0.0f);
    //  //Joints_FK(0, 38.97,0,0,-38.97,0);
    //Joints_FK(0,60,-90,0,60,0);        //定义初始位置
   // osDelay(3000);
    //Joints_IK_Scan();
    int i=0;
    for(;;)
    {
        // Joints_IK_BatchTest(-200.0f, 200.0f, 50.0f,   // X范围：-200~200mm，步长50mm
        //          -200.0f, 200.0f, 50.0f,   // Y范围：-200~200mm，步长50mm
        //          100.0f,  280.0f, 50.0f,   // Z范围：100~300mm，步长50mm
        //          0.0f, 0.0f, 0.0f);        // 目标姿态为0°
        Joints_IK(100,200,280);
        // if (i<=200) {
        //     Joints_IK(i,200,200);
        //     i++;
        // }
       // Joints_IK(0,200,200);
        //Joints_IK_Continuous_Test(&huart_debug);
        //Joints_FK(0,-20,10,12,0,0);
        // Joints_FK(60,60,-90,30,-60,0);
        osDelay(200);

    }
}

void Camera_Data(void *argument)
{
    int16_t pos;
    /* USER CODE BEGIN Camera_Data */
    /* Infinite loop */
    for(;;)
    {
        // osMessageQueuePut(&Solve_AngleHandle,&pos,0,osWaitForever);
        osDelay(1000);
    }
    /* USER CODE END Camera_Data */
}

void OLED(void *argument){
    /* USER CODE BEGIN Debug */
    /* Infinite loop */
    for(;;)
    {
        oled_loop();
        osDelay(20);
        /* USER CODE END Debug */
    }
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

    //OTTO_uart(&huart_debug,"✅/*******************电机日志任务启动*******************/");
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
        for (uint8_t i = 0; i<ZDT_MOTOR_NUM; i++)
        {
            //ZDT_Log_PrintCmdResult(Z[i]);
        }
#endif
/********************************达妙电机状态打印*********************************/
        for (uint8_t i = 0; i<DAM_MOTOR_NUM ; i++)
        {
           // DAM_Motor_PV_State(D[i]);
        }
        //周期性延迟（固定频率执行）
        osDelay(100);
    }
    /* USER CODE END Log */
}
