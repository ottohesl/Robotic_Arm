/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>

#include "DM_motor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
motor_t motor1,motor2, motor3, motor4;
ZDT_FBpara_t z_motor1,z_motor2,z_motor3;
FDCAN_RxHeaderTypeDef RxHeader_ZDT;
FDCAN_RxHeaderTypeDef RxHeader_DAM;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t DAM_MOTOR_RxData[20];
uint8_t ZDT_MOTOR_rxData[32]={0};
uint8_t loopback_rx_data[32] = {0};
FDCAN_RxHeaderTypeDef loopback_rx_header;
FDCAN_RxHeaderTypeDef loopback_rx_header_DAM;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
#if check_num
  if (hfdcan->Instance == FDCAN1) {
    // 获取接收到的消息
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader_DAM, DAM_MOTOR_RxData) == HAL_OK) {
      // 解析电机反馈数据（反馈帧ID为0）
      uint8_t addr = DAM_MOTOR_RxData[0] & 0xF;
      motor_t *motor = NULL;
      switch(addr) {
        case MOTOR1:
          motor=DAM_get_motor1();
          break;
        case MOTOR2:
          motor=DAM_get_motor2();
          break;
        case MOTOR3:
          motor=DAM_get_motor3();
          break;
        default:
          OTTO_uart(&huart_debug,"DAM地址错误：0x%04X",addr);
          break;
      }
      if (motor != NULL) {
        // 解析反馈数据
        dm_motor_fbdata(motor, DAM_MOTOR_RxData);
      }
    }
  }
#else
  if (hfdcan->Instance == FDCAN1) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &loopback_rx_header_DAM, loopback_rx_data) == HAL_OK) {
      // 串口打印帧头信息（扩展帧、ID、长度）
      OTTO_uart(&huart_debug, "===== FDCAN1回环帧（发送的命令）=====");
      OTTO_uart(&huart_debug, "帧类型：扩展帧（29位）");
      OTTO_uart(&huart_debug, "帧ID：0x%04X", loopback_rx_header_DAM.Identifier); // 扩展帧ID用8位十六进制
      OTTO_uart(&huart_debug, "数据长度：%d 字节", loopback_rx_header_DAM.DataLength ); // DLC转字节数
      OTTO_uart(&huart_debug, "数据内容（十六进制）：");
      // 打印数据内容（逐字节）
      for (int i = 0; i < loopback_rx_header_DAM.DataLength ; i++) {
        OTTO_uart(&huart_debug, "0x%02X ", loopback_rx_data[i]);
      }
      OTTO_uart(&huart_debug, "=====================================");
    }
  }

#endif

#if check_num
    if (hfdcan->Instance == FDCAN2) {
      //获取接收到的消息
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader_ZDT, ZDT_MOTOR_rxData) == HAL_OK) {
        // 解析电机反馈数据（反馈帧ID为0）
         uint8_t addr = (RxHeader_ZDT.Identifier>>8)&0xFF;
        ZDT_FBpara_t *motor = NULL;
        switch(addr) {
          case ZDT_MOTOR1:
            motor = get_motor1();
            break;
          case ZDT_MOTOR2:
            motor = get_motor2();
            break;
          case ZDT_MOTOR3:
            motor = get_motor3();
            break;
          default:
            OTTO_uart(&huart_debug,"ZDT地址错误：0x%04X",addr);
            break;
        }
        if (motor != NULL) {
          // 解析反馈数据
          ZDT_Control_Analyze_FDBack(motor, ZDT_MOTOR_rxData, addr);
        }
      }
    }
#else check_num
if (hfdcan->Instance == FDCAN2) {
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &loopback_rx_header, loopback_rx_data) == HAL_OK) {
    // 串口打印帧头信息（扩展帧、ID、长度）
    OTTO_uart(&huart_debug, "帧ID：0x%04X", loopback_rx_header.Identifier); // 扩展帧ID用8位十六进制
    OTTO_uart(&huart_debug, "数据长度：%d 字节", loopback_rx_header.DataLength ); // DLC转字节数
     //打印数据内容（逐字节）
     for (int i = 0; i < loopback_rx_header.DataLength ; i++) {
       OTTO_uart(&huart_debug, "0x%02X ", loopback_rx_data[i]);
     }
    OTTO_uart(&huart_debug, "=====================================");
  }
}
#endif
}
//获取电机反馈
ZDT_FBpara_t* get_motor1() {
  return &z_motor1;
}
ZDT_FBpara_t*get_motor2() {
  return &z_motor2;
}
ZDT_FBpara_t* get_motor3() {
  return &z_motor3;
}
motor_t* DAM_get_motor1() {
  return &motor1;
}
motor_t* DAM_get_motor2() {
  return &motor2;
}
motor_t* DAM_get_motor3() {
  return &motor3;
}
void ZDT_Motors_Init(void)
{
  // 初始化电机1
  z_motor1.id = ZDT_MOTOR1;
  z_motor1.vaild = 0;  // 初始化为无效数据
  z_motor1.S_Flag.IS_ENABLE = false;
  z_motor1.S_Flag.IS_INPLACE = false;
  z_motor1.S_Flag.IS_LOCKED = false;
  z_motor1.S_Flag.IS_SAVE_LOCKED = false;

  // 其他参数初始化为0
  z_motor1.S_vbus = 0;
  z_motor1.S_Cpos = 0.0f;
  z_motor1.S_Tpos = 0.0f;
  z_motor1.S_Perr = 0.0f;
  z_motor1.S_Temp = 0.0f;
  z_motor1.S_Vel.Vel_RPM = 0.0f;
  z_motor1.S_Vel.Vel_RPS = 0.0f;
  // 初始化电机2
  z_motor2.id = ZDT_MOTOR2;
  z_motor2.vaild = 0;  // 初始化为无效数据
  z_motor2.S_Flag.IS_ENABLE = false;
  z_motor2.S_Flag.IS_INPLACE = false;
  z_motor2.S_Flag.IS_LOCKED = false;
  z_motor2.S_Flag.IS_SAVE_LOCKED = false;

  // 其他参数初始化为0
  z_motor2.S_vbus = 0;
  z_motor2.S_Cpos = 0.0f;
  z_motor2.S_Tpos = 0.0f;
  z_motor2.S_Perr = 0.0f;
  z_motor2.S_Temp = 0.0f;
  z_motor2.S_Vel.Vel_RPM = 0.0f;
  z_motor2.S_Vel.Vel_RPS = 0.0f;
  // 初始化电机3
  z_motor3.id = ZDT_MOTOR3;
  z_motor3.vaild = 0;  // 初始化为无效数据
  z_motor3.S_Flag.IS_ENABLE = false;
  z_motor3.S_Flag.IS_INPLACE = false;
  z_motor3.S_Flag.IS_LOCKED = false;
  z_motor3.S_Flag.IS_SAVE_LOCKED = false;

  // 其他参数初始化为0
  z_motor3.S_vbus = 0;
  z_motor3.S_Cpos = 0.0f;
  z_motor3.S_Tpos = 0.0f;
  z_motor3.S_Perr = 0.0f;
  z_motor3.S_Temp = 0.0f;
  z_motor3.S_Vel.Vel_RPM = 0.0f;
  z_motor3.S_Vel.Vel_RPS = 0.0f;
}
void enable_motor(motor_t *motor) {
  dm_motor_enable(&hfdcan_dam, motor);
}
void write_pos_mode(motor_name motor) {
  write_motor_data(motor,10, pos_mode, 0, 0, 0);
  HAL_Delay(200);
  save_motor_data(motor, 10);
  HAL_Delay(200);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /**************************开启fdcan和fifo中断*********************************/
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  /**************************初始化达妙电机/张大头步进电机***************************/
  DAM_Motor_Init(&motor1,MOTOR1,20,10);
  DAM_Motor_Init(&motor2,MOTOR2,20,10);
  DAM_Motor_Init(&motor3,MOTOR3,20,10);
  ZDT_Motors_Init();
  /************************初次调用，程序运行后无需再次调用***************************/
  write_pos_mode(MOTOR1);
  write_pos_mode(MOTOR2);
  write_pos_mode(MOTOR3);
  /*********************************使能达妙电机**********************************/
  enable_motor(&motor1);
  enable_motor(&motor2);
  enable_motor(&motor3);
  /*****************************************************************************/
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM23 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM23)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
