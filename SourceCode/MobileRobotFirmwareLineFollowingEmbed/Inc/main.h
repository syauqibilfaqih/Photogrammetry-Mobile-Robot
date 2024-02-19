/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define s0_Pin GPIO_PIN_1
#define s0_GPIO_Port GPIOC
#define s1_Pin GPIO_PIN_3
#define s1_GPIO_Port GPIOC
#define s2_Pin GPIO_PIN_1
#define s2_GPIO_Port GPIOA
#define s3_Pin GPIO_PIN_5
#define s3_GPIO_Port GPIOA
#define M1_D1_Pin GPIO_PIN_15
#define M1_D1_GPIO_Port GPIOB
#define M1_D2_Pin GPIO_PIN_9
#define M1_D2_GPIO_Port GPIOD
#define M0_D1_Pin GPIO_PIN_10
#define M0_D1_GPIO_Port GPIOD
#define M0_D2_Pin GPIO_PIN_11
#define M0_D2_GPIO_Port GPIOD
#define lcd_db6_Pin GPIO_PIN_14
#define lcd_db6_GPIO_Port GPIOD
#define lcd_db7_Pin GPIO_PIN_15
#define lcd_db7_GPIO_Port GPIOD
#define lcd_db5_Pin GPIO_PIN_6
#define lcd_db5_GPIO_Port GPIOC
#define lcd_db4_Pin GPIO_PIN_8
#define lcd_db4_GPIO_Port GPIOC
#define lcd_e_Pin GPIO_PIN_9
#define lcd_e_GPIO_Port GPIOC
#define lcd_rw_Pin GPIO_PIN_8
#define lcd_rw_GPIO_Port GPIOA
#define lcd_rs_Pin GPIO_PIN_11
#define lcd_rs_GPIO_Port GPIOA
#define buzzer_Pin GPIO_PIN_12
#define buzzer_GPIO_Port GPIOA
#define M0_PWM_Pin GPIO_PIN_15
#define M0_PWM_GPIO_Port GPIOA
#define M1_PWM_Pin GPIO_PIN_3
#define M1_PWM_GPIO_Port GPIOB
#define brd_led_Pin GPIO_PIN_1
#define brd_led_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
