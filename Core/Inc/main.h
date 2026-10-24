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
#include "stm32f1xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define US_F_TRIG_Pin GPIO_PIN_6
#define US_F_TRIG_GPIO_Port GPIOA
#define US_F_ECHO_Pin GPIO_PIN_7
#define US_F_ECHO_GPIO_Port GPIOA
#define US_F_ECHO_EXTI_IRQn EXTI9_5_IRQn
#define US_L_TRIG_Pin GPIO_PIN_0
#define US_L_TRIG_GPIO_Port GPIOB
#define US_L_ECHO_Pin GPIO_PIN_1
#define US_L_ECHO_GPIO_Port GPIOB
#define US_L_ECHO_EXTI_IRQn EXTI1_IRQn
#define AIN1_Pin GPIO_PIN_12
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_13
#define AIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_14
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_15
#define BIN2_GPIO_Port GPIOB
#define US_R_TRIG_Pin GPIO_PIN_8
#define US_R_TRIG_GPIO_Port GPIOB
#define US_R_ECHO_Pin GPIO_PIN_9
#define US_R_ECHO_GPIO_Port GPIOB
#define US_R_ECHO_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
