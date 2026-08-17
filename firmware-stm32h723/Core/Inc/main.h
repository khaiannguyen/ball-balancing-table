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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN2_Pin GPIO_PIN_2
#define BTN2_GPIO_Port GPIOE
#define BTN2_EXTI_IRQn EXTI2_IRQn
#define BTN3_Pin GPIO_PIN_3
#define BTN3_GPIO_Port GPIOE
#define BTN3_EXTI_IRQn EXTI3_IRQn
#define BTN4_Pin GPIO_PIN_4
#define BTN4_GPIO_Port GPIOE
#define BTN4_EXTI_IRQn EXTI4_IRQn
#define BTN5_Pin GPIO_PIN_5
#define BTN5_GPIO_Port GPIOE
#define BTN5_EXTI_IRQn EXTI9_5_IRQn
#define BTN6_Pin GPIO_PIN_6
#define BTN6_GPIO_Port GPIOE
#define BTN6_EXTI_IRQn EXTI9_5_IRQn
#define BTN1_Pin GPIO_PIN_1
#define BTN1_GPIO_Port GPIOF
#define BTN1_EXTI_IRQn EXTI1_IRQn
#define IMU_INT_Pin GPIO_PIN_0
#define IMU_INT_GPIO_Port GPIOC
#define IMU_INT_EXTI_IRQn EXTI0_IRQn
#define IMU_CS_Pin GPIO_PIN_3
#define IMU_CS_GPIO_Port GPIOC
#define TFT_DC_Pin GPIO_PIN_6
#define TFT_DC_GPIO_Port GPIOA
#define TFT_CS_Pin GPIO_PIN_14
#define TFT_CS_GPIO_Port GPIOE
#define TFT_RST_Pin GPIO_PIN_8
#define TFT_RST_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
