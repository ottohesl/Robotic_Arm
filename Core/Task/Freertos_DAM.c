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
     // Joints_FK(-111.80 ,-45.62,-20.94 ,180.00 ,-66.55 ,-68.20 );
    // Joints_IK_Debug(0.0f, 200, 200.0f, 0.0f, 0.0f, 0.0f);
    //Joints_IK(0,250,380);
   // Joints_FK( -90.00, -32.55  ,-40.79  ,0.00 ,73.34 , 90.00 );
    //ZDT_Control_Origin_Trigger_Return(&hfdcan_zdt,ZDT_MOTOR1,00,SIN);
   // Joints_FK(0,0,0,0,0,0);
   Joints_FK(0,60,-90,0,-60,0);        //定义初始位置
    osDelay(800);
    Joints_FK(0,60,-90,0,-60,0);
   // Joints_FK(10,40,-80,20,-20,20);
   // osDelay(3000);
    //Joints_IK_Scan();
    // Joints_IK_BatchTest(-200.0f, 200.0f, 10.0f,   // X范围：-200~200mm，步长50mm
    //      250.0f, 250.0f, 10.0f,   // Y范围：-200~200mm，步长50mm
    //      380.0f,  380.0f, 10.0f,   // Z范围：100~300mm，步长50mm
    //      0.0f, 0.0f, 0.0f);        // 目标姿态为0°
    float i=-100;
    int time=800;
    //
    for(;;)
    {
       // Joints_FK(0,60,-90,0,-60,0);
        //Joints_FK(0,30,-60,60,-60,0);
        //Joints_FK(0,60,-90,0,-60,0);
        //Joints_FK(0,60,-90,0,-60,0);
         // Joints_IK(-100,250,380);
         // osDelay(time);
         // Joints_IK(-60,250,380);
         // osDelay(time);
         // Joints_IK(-30,250,380);
         // osDelay(time);
         // Joints_IK(0,250,380);
         // osDelay(time);
         // Joints_IK(30,250,380);
         // osDelay(time);
         // Joints_IK(60,250,380);
         // osDelay(time);
         // Joints_IK(100,250,380);
         // osDelay(time);
         // //回
         // Joints_IK(60,250,380);
         // osDelay(time);
         // Joints_IK(30,250,380);
         // osDelay(time);
         // Joints_IK(0,250,380);
         // osDelay(time);
         // Joints_IK(-30,250,380);
         // osDelay(time);
         // Joints_IK(-60,250,380);
         // osDelay(time);
         // Joints_IK(100,200,280);
         // osDelay(200);
        //Joints_FK(10, 40 ,-70.16   ,0,29.96,-63.66);
        // if (i<=200) {
        //     Joints_IK(i,250,380);
        //     i=i+10;
        //     osDelay(100);
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
/**********************************发送关节角度到mqtt*********************************/
        Joint6D_t *current_joints = pvPortMalloc(sizeof(Joint6D_t));
        // 必须判断是否分配成功！防止内存不足死机
        if(current_joints == NULL)
        {
            // 分配失败处理
            osDelay(1);
            continue;
        }
        Joint6D_ReadFromMotor(current_joints);
        SendCurrentJointsToESP32(current_joints);
        vPortFree(current_joints);
        OTTO_uart(&huart_debug, "dam实时位置:%.2f °,%.2f °,%.2f °",current_joints->a[0],current_joints->a[1],current_joints->a[2]);
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
           //DAM_Motor_PV_State(D[i]);
        }
        //周期性延迟（固定频率执行）
        osDelay(100);
    }
    /* USER CODE END Log */
}
