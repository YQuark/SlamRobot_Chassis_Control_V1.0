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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOC
#define ADC_VBAT_Pin GPIO_PIN_0
#define ADC_VBAT_GPIO_Port GPIOA
#define ADC_CUR_L_Pin GPIO_PIN_1
#define ADC_CUR_L_GPIO_Port GPIOA
#define ADC_CUR_R_Pin GPIO_PIN_2
#define ADC_CUR_R_GPIO_Port GPIOA
#define PS2_CLK_Pin GPIO_PIN_4
#define PS2_CLK_GPIO_Port GPIOA
#define PS2_CS_Pin GPIO_PIN_5
#define PS2_CS_GPIO_Port GPIOA
#define PS2_CMD_Pin GPIO_PIN_6
#define PS2_CMD_GPIO_Port GPIOA
#define PS2_DAT_Pin GPIO_PIN_7
#define PS2_DAT_GPIO_Port GPIOA
#define L_A_Pin GPIO_PIN_12
#define L_A_GPIO_Port GPIOD
#define L_B_Pin GPIO_PIN_13
#define L_B_GPIO_Port GPIOD
#define R_A_Pin GPIO_PIN_6
#define R_A_GPIO_Port GPIOC
#define R_B_Pin GPIO_PIN_7
#define R_B_GPIO_Port GPIOC
#define PWM_R_1_Pin GPIO_PIN_8
#define PWM_R_1_GPIO_Port GPIOC
#define PWM_R_2_Pin GPIO_PIN_9
#define PWM_R_2_GPIO_Port GPIOC
#define PWM_L_1_Pin GPIO_PIN_8
#define PWM_L_1_GPIO_Port GPIOA
#define PWM_L_2_Pin GPIO_PIN_11
#define PWM_L_2_GPIO_Port GPIOA
#define IMU_INT_Pin GPIO_PIN_5
#define IMU_INT_GPIO_Port GPIOB
#define IMU_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
