/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Power_OUT2_EN_Pin GPIO_PIN_13
#define Power_OUT2_EN_GPIO_Port GPIOC
#define Power_OUT1_EN_Pin GPIO_PIN_14
#define Power_OUT1_EN_GPIO_Port GPIOC
#define SPI2_CS0_Pin GPIO_PIN_0
#define SPI2_CS0_GPIO_Port GPIOC
#define SPI2_CS1_Pin GPIO_PIN_3
#define SPI2_CS1_GPIO_Port GPIOC
#define RGB_Pin GPIO_PIN_7
#define RGB_GPIO_Port GPIOA
#define Camera_RX_Pin GPIO_PIN_7
#define Camera_RX_GPIO_Port GPIOE
#define Camera_TX_Pin GPIO_PIN_8
#define Camera_TX_GPIO_Port GPIOE
#define IMU__INT1_Pin GPIO_PIN_10
#define IMU__INT1_GPIO_Port GPIOE
#define IMU_INT3_Pin GPIO_PIN_12
#define IMU_INT3_GPIO_Port GPIOE
#define Chassis_TX_Pin GPIO_PIN_9
#define Chassis_TX_GPIO_Port GPIOA
#define Chassis_RX_Pin GPIO_PIN_10
#define Chassis_RX_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
