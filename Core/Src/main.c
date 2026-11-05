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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_uart.h"
#include "stm32h7xx_hal.h"
#include "app_vision.h"
#include "app_chassis.h"
#include "app_roboarm.h"
#include "app_imu.h"
#include "app_imu_uart_debug.h"
#include "alg_vision_align.h"
#include "bsp_rgb.h"
#include "app_chassis_closeloop.h"
#include "app_chassis_motion.h"
#include "stm32h7xx_hal_tim.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//此为重要参数，角度环的周期时钟基础，修改后需要重新调试角度环
#define CHASSIS_MOTION_UPDATE_PERIOD 1U   //每10ms更新一次
#define CHASSIS_MOTION_TEST_SLOWDOWN 1000U   //测试用，每隔5s修改一次state，重新设置一条motion命令

/* 1：独立QR坐标验证，使用USART10控制机械臂，暂停路线和IMU串口日志。 */
#ifndef VISION_COORD_DEBUG_ENABLE
#define VISION_COORD_DEBUG_ENABLE 0
#endif

#ifndef CHASSIS_TCP_CONFIRM_DEBUG
#define CHASSIS_TCP_CONFIRM_DEBUG 0
#endif

#define CORRECT_TCP_QR_X 205
#define CORRECT_TCP_QR_Y -13

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart10;

/* USER CODE BEGIN PV */
int s_imu_ret;
double testangle[3] = {0.0, 0.0, 0.0};
double yawangle = 0.0;
double temperature = 0.0;
float distance = 0.0f;
volatile uint8_t chassis_motion_count = 0U;

int s_imu_uart_debug_ret;  /* 调试器可查看 RET_OK / RET_BUSY / 错误码 */

/* 每次测试只修改这组XYZ/pitch，控制机械臂与坐标换算共用同一份姿态。 */
//End_Effector_Pos_t vision_arm_pose = {170.0, 0.0,  25.0,  0.0};//11月4日测试用观察人质
//End_Effector_Pos_t vision_arm_pose = {0.0, 260.0, 60.0, 0.0};//观察二维码
End_Effector_Pos_t vision_arm_pose = {154.0, 0.0,   26.0, -1.0};//观察爆炸物
//End_Effector_Pos_t vision_arm_pose = {180.0, 0.0, 25.0, -0.0};//观察恐怖分子
//End_Effector_Pos_t vision_arm_pose = {133.537333615, 1.525593299, 82.971289222, -0.338};//观察人质
int vision_arm_move_ret = RET_NOTREADY;
volatile VisionResult_t vision_result;       /* K230原始结果，xyz_mm为TCP系坐标 */
volatile float qr_position_body[3];          /* QR中心相对基座，X右/Y前/Z上，mm */
volatile float arm_origin_relative_qr[3];    /* 基座相对QR中心的位移，用同一组B轴表达 */
volatile uint32_t vision_count = 0U;
volatile uint8_t vision_valid = 0U;

volatile uint8_t chassis_fix_onlyonce = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART7_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART10_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

void vision_confirm_function(const proto_vision_frame_t *frame);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void vision_confirm_function(const proto_vision_frame_t *frame)
{
  if (frame == NULL || frame->cmd != VISION_CMD_RESULT) {
    return;
  }

  vision_valid = 0U;
  VisionResult_t result;
  if (!proto_vision_decode_result(frame->payload, frame->len, &result)) {
    return;
  }
  vision_result = result;
  if (result.task_cmd != VISION_CMD_QR || result.slot_id != 0U ||
      (result.status != VISION_SUCCESS && result.status != VISION_OBSERVATION) ||
      (result.flags & VISION_POSITION) == 0U || vision_arm_move_ret != RET_OK) {
    return;
  }

  VisionTransform_t transform;
  if (!alg_vision_transform_from_arm_pose(&vision_arm_pose, &transform)) {
    return;
  }

  /* K230已完成相机到TCP的手眼变换，这里计算 p_B = R_B_E*p_E + t_B_E。 */
  float position[3];
  for (unsigned i = 0U; i < 3U; ++i) {
    position[i] = transform.translation[i];
    for (unsigned j = 0U; j < 3U; ++j) {
      position[i] += transform.rotation[i][j] * result.xyz_mm[j];
    }
    if (!isfinite(position[i])) {
      return;
    }
  }
  for (unsigned i = 0U; i < 3U; ++i) {
    qr_position_body[i] = position[i];
    arm_origin_relative_qr[i] = -position[i];
  }
  ++vision_count;
  vision_valid = 1U; /* 在下面return处打断点，查看arm_origin_relative_qr[0..2]。 */

  chassis_fix_onlyonce = 1;
  return;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_UART7_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_USART10_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  bsp_rgb_init();
  bsp_rgb_set(40, 0, 0);   //上电后先亮红灯，表示系统初始化中
  //除mcu外其他部分供电使能
  HAL_GPIO_WritePin(Power_OUT2_EN_GPIO_Port,Power_OUT2_EN_Pin,GPIO_PIN_SET);
  /* OUT5V_EN直接控制激光，启动时保持关闭。 */
  HAL_GPIO_WritePin(OUT5V_EN_GPIO_Port,OUT5V_EN_Pin,GPIO_PIN_RESET);
  HAL_Delay(2000);    //等待其他设备稳定
  //视觉链路初始化
  if (app_vision_init(&huart7) != RET_OK)
  {
    Error_Handler();
  }
  /* 注册视觉通信回调 */
  //app_vision_set_rx_cb(vision_on_frame);
  //底盘链路初始化
  if (app_chassis_init(&huart1) != RET_OK)
  {
    Error_Handler();
  }
  app_chassis_move(0.0f, 0.0f, 0.0f);       //初始化底盘模块
  /* 机械臂链路初始化（USART10：PE3=TX / PE2=RX）*/
#if VISION_COORD_DEBUG_ENABLE || !APP_IMU_UART_DEBUG_ENABLE
  if (app_roboarm_init(&huart10) != RET_OK)
  {
    Error_Handler();
  }
#endif

  for (uint32_t i = 0U; i < 3U; i++)
  {
    s_imu_ret = app_imu_init(&hspi2);
    //s_imu_ret = RET_OK;
    if (s_imu_ret == RET_OK)
    {
      break;
    }
    HAL_Delay(50);   /* 失败就等一会儿重来（整个初始化会重跑一遍） */
  }
  if (s_imu_ret != RET_OK)
  {
    Error_Handler();
  }
  (void)app_imu_poll();
  app_chassis_motion_init();
  task_init();   //任务初始化
  HAL_TIM_Base_Start_IT(&htim6);  //闭环开启区域
  bsp_rgb_set(0, 40, 0);

#if VISION_COORD_DEBUG_ENABLE
  app_vision_set_rx_cb(vision_confirm_function);
  HAL_Delay(3000);
  vision_arm_move_ret = app_roboarm_servo_move_to(vision_arm_pose.x, vision_arm_pose.y,
                                                vision_arm_pose.z, vision_arm_pose.pitch);
  HAL_Delay(3000);
  bsp_uart_flush_rx(BSP_UART_VISION); /* 丢弃运动期间积压的数据。 */
  (void)app_vision_send(VISION_CMD_QR, NULL, 0U); /* 让脚本清空旧样本并重新开始采样。 */
#elif APP_IMU_UART_DEBUG_ENABLE
  /* 调试期间 USART10 只发 CSV，不下发舵机二进制指令。 */
  s_imu_uart_debug_ret = app_imu_uart_debug_init(&huart10, HAL_GetTick());
  if (s_imu_uart_debug_ret != RET_OK)
  {
    Error_Handler();
  }
#else
  HAL_Delay(3000);
  app_roboarm_servo_move_to(0.0,250.0,100.0,-0.174533);
  HAL_Delay(3000);
#endif


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    (void)app_imu_poll();
    app_vision_poll();
    app_chassis_poll();
    distance = app_chassis_get_distance();
    temperature = app_imu_get_temp();
    app_imu_get_euler(testangle);
    yawangle = testangle[2];
    /* 一秒一次，由模块内部限频；主循环持续轮询 IMU，不插入发送延时。 */
#if VISION_COORD_DEBUG_ENABLE
    /* app_vision_poll内的回调逐包完成坐标验证；此模式不运行路线或IMU串口日志。 */
#elif APP_IMU_UART_DEBUG_ENABLE
    int debug_ret = app_imu_uart_debug_poll(HAL_GetTick());
    if (debug_ret != RET_IDLE) {
      s_imu_uart_debug_ret = debug_ret;
    }
#else
    task_poll();   //任务流：时间驱动的状态机

    /* 规划先更新目标，闭环随后启动非阻塞的串口中断发送。 */
    if (chassis_motion_count >= CHASSIS_MOTION_UPDATE_PERIOD) {
      chassis_motion_count = 0U;
      app_chassis_motion_update(0.01f);
      app_chassis_closeloop_update();
    }
#endif

#if CHASSIS_TCP_CONFIRM_DEBUG
    if (chassis_fix_onlyonce) {
      chassis_fix_onlyonce = 0;
      /* 根据TCP结果计算移动距离和方向下放底盘。 */
      float distance = ((arm_origin_relative_qr[0] - CORRECT_TCP_QR_X) * (arm_origin_relative_qr[0] - CORRECT_TCP_QR_X)) 
                        +((arm_origin_relative_qr[1] - CORRECT_TCP_QR_Y) * (arm_origin_relative_qr[1] - CORRECT_TCP_QR_Y));
      distance = sqrtf(distance);
      float direction = atan(fabsf(arm_origin_relative_qr[0] - CORRECT_TCP_QR_X) / fabsf(arm_origin_relative_qr[1] - CORRECT_TCP_QR_Y));
      if((arm_origin_relative_qr[0] - CORRECT_TCP_QR_X)<0&&(arm_origin_relative_qr[1]-CORRECT_TCP_QR_Y)<0)direction = -direction;
      if((arm_origin_relative_qr[0] - CORRECT_TCP_QR_X)<0&&(arm_origin_relative_qr[1]-CORRECT_TCP_QR_Y)>0)direction = direction - 3.1415926535f;
      if((arm_origin_relative_qr[0] - CORRECT_TCP_QR_X)>0&&(arm_origin_relative_qr[1]-CORRECT_TCP_QR_Y)>0)direction = 3.1415926535f-direction;
      if((arm_origin_relative_qr[0] - CORRECT_TCP_QR_X)>0&&(arm_origin_relative_qr[1]-CORRECT_TCP_QR_Y)<0)direction = direction;
      app_chassis_motion_linear_move(distance,direction,100.0f,200.0f,200.0f);
      //(100.0f, -1.5707963f, 80.0f, 200.0f, 200.0f);
    }
    if (chassis_motion_count >= CHASSIS_MOTION_UPDATE_PERIOD) {
      chassis_motion_count = 0U;
      app_chassis_motion_update(0.01f);
      app_chassis_closeloop_update();
    }
#endif
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_ENABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 639;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART10_UART_Init(void)
{

  /* USER CODE BEGIN USART10_Init 0 */

  /* USER CODE END USART10_Init 0 */

  /* USER CODE BEGIN USART10_Init 1 */

  /* USER CODE END USART10_Init 1 */
  huart10.Instance = USART10;
  huart10.Init.BaudRate = 9600;
  huart10.Init.WordLength = UART_WORDLENGTH_8B;
  huart10.Init.StopBits = UART_STOPBITS_1;
  huart10.Init.Parity = UART_PARITY_NONE;
  huart10.Init.Mode = UART_MODE_TX_RX;
  huart10.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart10.Init.OverSampling = UART_OVERSAMPLING_16;
  huart10.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart10.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart10.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart10, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart10, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART10_Init 2 */

  /* USER CODE END USART10_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Power_OUT2_EN_Pin|SPI2_CS0_Pin|SPI2_CS1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT5V_EN_GPIO_Port, OUT5V_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Power_OUT1_EN_GPIO_Port, Power_OUT1_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LASER_Pin|RGB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Power_OUT2_EN_Pin Power_OUT1_EN_Pin OUT5V_EN_Pin */
  GPIO_InitStruct.Pin = Power_OUT2_EN_Pin|Power_OUT1_EN_Pin|OUT5V_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_CS0_Pin SPI2_CS1_Pin */
  GPIO_InitStruct.Pin = SPI2_CS0_Pin|SPI2_CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LASER_Pin */
  GPIO_InitStruct.Pin = LASER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LASER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RGB_Pin */
  GPIO_InitStruct.Pin = RGB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RGB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU__INT1_Pin IMU_INT3_Pin */
  GPIO_InitStruct.Pin = IMU__INT1_Pin|IMU_INT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
