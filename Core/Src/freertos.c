/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Motor_Control_T */
osThreadId_t Motor_Control_THandle;
const osThreadAttr_t Motor_Control_T_attributes = {
  .name = "Motor_Control_T",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Solve_Task */
osThreadId_t Solve_TaskHandle;
const osThreadAttr_t Solve_Task_attributes = {
  .name = "Solve_Task",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Camera_Data_Tas */
osThreadId_t Camera_Data_TasHandle;
const osThreadAttr_t Camera_Data_Tas_attributes = {
  .name = "Camera_Data_Tas",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Debug_Task */
osThreadId_t Debug_TaskHandle;
const osThreadAttr_t Debug_Task_attributes = {
  .name = "Debug_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Log_Task */
osThreadId_t Log_TaskHandle;
const osThreadAttr_t Log_Task_attributes = {
  .name = "Log_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow3,
};
/* Definitions for Solve_Angle */
osMessageQueueId_t Solve_AngleHandle;
const osMessageQueueAttr_t Solve_Angle_attributes = {
  .name = "Solve_Angle"
};
/* Definitions for Motor_State */
osMessageQueueId_t Motor_StateHandle;
const osMessageQueueAttr_t Motor_State_attributes = {
  .name = "Motor_State"
};
/* Definitions for Target_Pos */
osMessageQueueId_t Target_PosHandle;
const osMessageQueueAttr_t Target_Pos_attributes = {
  .name = "Target_Pos"
};
/* Definitions for CAN1RX_Data */
osMessageQueueId_t CAN1RX_DataHandle;
const osMessageQueueAttr_t CAN1RX_Data_attributes = {
  .name = "CAN1RX_Data"
};
/* Definitions for CAN2RX_Data */
osMessageQueueId_t CAN2RX_DataHandle;
const osMessageQueueAttr_t CAN2RX_Data_attributes = {
  .name = "CAN2RX_Data"
};
/* Definitions for uart_log_mutex */
osMutexId_t uart_log_mutexHandle;
const osMutexAttr_t uart_log_mutex_attributes = {
  .name = "uart_log_mutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Motor_Control(void *argument);
void Solve(void *argument);
void Camera_Data(void *argument);
void Debug(void *argument);
void Log(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of uart_log_mutex */
  uart_log_mutexHandle = osMutexNew(&uart_log_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Solve_Angle */
  Solve_AngleHandle = osMessageQueueNew (16, sizeof(uint16_t), &Solve_Angle_attributes);

  /* creation of Motor_State */
  Motor_StateHandle = osMessageQueueNew (16, sizeof(uint16_t), &Motor_State_attributes);

  /* creation of Target_Pos */
  Target_PosHandle = osMessageQueueNew (16, sizeof(uint16_t), &Target_Pos_attributes);

  /* creation of CAN1RX_Data */
  CAN1RX_DataHandle = osMessageQueueNew (16, sizeof(CAN_Rx_Msg_t*), &CAN1RX_Data_attributes);

  /* creation of CAN2RX_Data */
  CAN2RX_DataHandle = osMessageQueueNew (16, sizeof(CAN_Rx_Msg_t*), &CAN2RX_Data_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Motor_Control_T */
  Motor_Control_THandle = osThreadNew(Motor_Control, NULL, &Motor_Control_T_attributes);

  /* creation of Solve_Task */
  Solve_TaskHandle = osThreadNew(Solve, NULL, &Solve_Task_attributes);

  /* creation of Camera_Data_Tas */
  Camera_Data_TasHandle = osThreadNew(Camera_Data, NULL, &Camera_Data_Tas_attributes);

  /* creation of Debug_Task */
  Debug_TaskHandle = osThreadNew(Debug, NULL, &Debug_Task_attributes);

  /* creation of Log_Task */
  Log_TaskHandle = osThreadNew(Log, NULL, &Log_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Motor_Control */
/**
  * @brief  Function implementing the Motor_Control_T thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Motor_Control */
__weak void Motor_Control(void *argument)
{
  /* USER CODE BEGIN Motor_Control */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Motor_Control */
}

/* USER CODE BEGIN Header_Solve */
/**
* @brief Function implementing the Solve_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Solve */
__weak void Solve(void *argument)
{
  /* USER CODE BEGIN Solve */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Solve */
}

/* USER CODE BEGIN Header_Camera_Data */
/**
* @brief Function implementing the Camera_Data_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Camera_Data */
__weak void Camera_Data(void *argument)
{
  /* USER CODE BEGIN Camera_Data */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Camera_Data */
}

/* USER CODE BEGIN Header_Debug */
/**
* @brief Function implementing the Debug_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Debug */
__weak void Debug(void *argument)
{
  /* USER CODE BEGIN Debug */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Debug */
}

/* USER CODE BEGIN Header_Log */
/**
* @brief Function implementing the Log_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Log */
__weak void Log(void *argument)
{
  /* USER CODE BEGIN Log */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Log */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

