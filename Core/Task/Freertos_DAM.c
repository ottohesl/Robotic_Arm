#include "Freertos_DAM.h"
#include "DM_Motor_LOG.h"
#include "fdcan.h"
#include "ZDT_Control.h"
#include "ZDT_MOTOR_LOG.h"
#include "6DOF_Control.h"
void Motor_Control(void *argument)
{
    int16_t control_motor;
    int16_t now_motor_state;
    float pos=0;
    /* USER CODE BEGIN Motor_Control */
    /* Infinite loop */
    //ZDT_MOTOR_POSITION(ZDT_MOTOR1,CW,0.2, 0.5, pos);
    // ZDT_MOTOR_POSITION(ZDT_MOTOR2,CW,0.2, 0.5, 0);
    // ZDT_MOTOR_POSITION(ZDT_MOTOR3,CW,0.2, 0.5,pos);
    // ZDT_MOTOR_POSITION(ZDT_MOTOR1,CW,0.2, 0.5,80);

    //save_pos_zero(&hfdcan_dam, MOTOR3, mit_mode);
    //ZDT_MOTOR_VEL(ZDT_MOTOR1,CW,2, 10);
    for(;;)
    {
        // DAM_MOTOR_POS(MOTOR1 ,0, 0.1);
        // DAM_MOTOR_POS(MOTOR2 ,0, 0.1);
        // DAM_MOTOR_POS(MOTOR3 ,0, 0.1);
        // osDelay(3000);
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

        osDelay(500);
    }
    /* USER CODE END Motor_Control */
}
void Solve(void *argument)
{

    for(;;)
    {
        Joints_FK(10,10,10,-20,30,0);
        Joints_IK(141.05  ,9.59,414.06);
        Joints_FK(10.01 ,9.97 ,-10.05  ,-13.06, 49.30 ,-8.91 );
        osDelay(10000);
        OTTO_uart(&huart_debug, "\n🔄 重复正逆解测试...");
        // 重新执行步骤3-7（可封装为函数）
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
        osDelay(100);
    }
    /* USER CODE END Camera_Data */
}

void Debug(void *argument) {
    CAN_Rx_Msg_t can1_rx_msg;
    CAN_Rx_Msg_t can2_rx_msg;
    /* USER CODE BEGIN Debug */
    /* Infinite loop */
    for(;;)
    {
        //解析反馈帧
        if (osMessageQueueGet(CAN1RX_DataHandle, &can1_rx_msg, NULL, osWaitForever) == osOK) {
            uint8_t addr = can1_rx_msg.rx_data[0] & 0xF;
            motor_t *motor = NULL;
            switch(addr) {
                case MOTOR1: motor = DAM_get_motor1(); break;
                case MOTOR2: motor = DAM_get_motor2(); break;
                case MOTOR3: motor = DAM_get_motor3(); break;
                default:
                    OTTO_uart(&huart_debug,"DAM地址错误：0x%02X",addr);
                    break;
            }

            if (motor != NULL) {
                dm_motor_fbdata(motor, can1_rx_msg.rx_data);
            }
        }

        if (osMessageQueueGet(CAN2RX_DataHandle, &can2_rx_msg, NULL, osWaitForever) == osOK) {
            // 解析电机反馈数据（反馈帧ID为0）
            uint8_t addr = (can2_rx_msg.rx_header.Identifier>>8)&0xFF;
            ZDT_FBpara_t *motor = NULL;
            switch(addr) {
                case ZDT_MOTOR1:
                    motor = get_motor1();break;
                case ZDT_MOTOR2:
                    motor = get_motor2();break;
                case ZDT_MOTOR3:
                    motor = get_motor3();break;
                default:
                    OTTO_uart(&huart_debug,"ZDT地址错误：0x%04X",addr); break;
            }
            if (motor != NULL) {
                // 解析反馈数据
                ZDT_Control_Analyze_FDBack(motor, can2_rx_msg .rx_data, addr);

            }
            osDelay(100);
        }
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
        for (uint8_t i = 0; i<ZDT_MOTOR_NUM; i++)
        {
            ZDT_Log_PrintCmdResult(Z[i]);
        }
#endif
/********************************达妙电机状态打印*********************************/
        for (uint8_t i = 0; i<DAM_MOTOR_NUM ; i++)
        {
            DAM_Motor_PV_State(D[i]);
        }
        //周期性延迟（固定频率执行）
        osDelay(10000);
    }
    /* USER CODE END Log */
}
