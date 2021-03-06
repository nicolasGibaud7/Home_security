/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

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
#define BTN_USER_Pin GPIO_PIN_0
#define BTN_USER_GPIO_Port GPIOA
#define BTN_USER_EXTI_IRQn EXTI0_IRQn
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOA
#define KEYPAD_COLUMN_1_Pin GPIO_PIN_7
#define KEYPAD_COLUMN_1_GPIO_Port GPIOE
#define KEYPAD_ROW_1_Pin GPIO_PIN_8
#define KEYPAD_ROW_1_GPIO_Port GPIOE
#define KEYPAD_COLUMN_2_Pin GPIO_PIN_9
#define KEYPAD_COLUMN_2_GPIO_Port GPIOE
#define KEYPAD_ROW_2_Pin GPIO_PIN_10
#define KEYPAD_ROW_2_GPIO_Port GPIOE
#define KEYPAD_COLUMN_3_Pin GPIO_PIN_11
#define KEYPAD_COLUMN_3_GPIO_Port GPIOE
#define KEYPAD_ROW_3_Pin GPIO_PIN_12
#define KEYPAD_ROW_3_GPIO_Port GPIOE
#define KEYPAD_COLUMN_4_Pin GPIO_PIN_13
#define KEYPAD_COLUMN_4_GPIO_Port GPIOE
#define KEYPAD_ROW_4_Pin GPIO_PIN_14
#define KEYPAD_ROW_4_GPIO_Port GPIOE
#define BTN_BLACK_Pin GPIO_PIN_14
#define BTN_BLACK_GPIO_Port GPIOB
#define BTN_BLACK_EXTI_IRQn EXTI15_10_IRQn
#define BTN_BLUE_Pin GPIO_PIN_8
#define BTN_BLUE_GPIO_Port GPIOD
#define BTN_BLUE_EXTI_IRQn EXTI9_5_IRQn
#define BTN_POWER_Pin GPIO_PIN_9
#define BTN_POWER_GPIO_Port GPIOD
#define BTN_POWER_EXTI_IRQn EXTI9_5_IRQn
#define BTN_POWER_LED_Pin GPIO_PIN_10
#define BTN_POWER_LED_GPIO_Port GPIOD
#define BTN_YELLOW_Pin GPIO_PIN_11
#define BTN_YELLOW_GPIO_Port GPIOD
#define BTN_YELLOW_EXTI_IRQn EXTI15_10_IRQn
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_GREEN_GPIO_Port GPIOD
#define LED_ORANGE_Pin GPIO_PIN_13
#define LED_ORANGE_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOD
#define LED_BLUE_Pin GPIO_PIN_15
#define LED_BLUE_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
