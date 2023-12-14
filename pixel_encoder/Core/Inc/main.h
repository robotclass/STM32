/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define CI_Pin GPIO_PIN_0
#define CI_GPIO_Port GPIOA
#define DI_Pin GPIO_PIN_1
#define DI_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_2
#define SW2_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_4
#define SW1_GPIO_Port GPIOA
#define BTN_Pin GPIO_PIN_5
#define BTN_GPIO_Port GPIOA
#define BTN_EXTI_IRQn EXTI4_15_IRQn
#define SW3_Pin GPIO_PIN_1
#define SW3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define TRANSFER_DIR_WRITE      0x1
#define TRANSFER_DIR_READ       0x0

#define I2C_ADDRESS_SLIDER		0x20
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
