/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f3xx_hal.h"
#include "stm32f3xx_ll_adc.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task_resource.h"
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
#define MOTOR1_ENC_TIM TIM1
#define MOTOR_PWM_TIM TIM2
#define MOTOR2_ENC_TIM TIM3
#define ENC_UPDATE_TIM TIM7
#define KEY_CHECK_TIM TIM17
#define ANALOG1_Pin LL_GPIO_PIN_0
#define ANALOG1_GPIO_Port GPIOA
#define INA2_Pin LL_GPIO_PIN_1
#define INA2_GPIO_Port GPIOA
#define INB1_Pin LL_GPIO_PIN_2
#define INB1_GPIO_Port GPIOA
#define ANALOG2_Pin LL_GPIO_PIN_3
#define ANALOG2_GPIO_Port GPIOA
#define MOTOR2_ENC_B_Pin LL_GPIO_PIN_4
#define MOTOR2_ENC_B_GPIO_Port GPIOA
#define INA1_Pin LL_GPIO_PIN_5
#define INA1_GPIO_Port GPIOA
#define MOTOR2_ENC_A_Pin LL_GPIO_PIN_6
#define MOTOR2_ENC_A_GPIO_Port GPIOA
#define ENABLE_A_Pin LL_GPIO_PIN_7
#define ENABLE_A_GPIO_Port GPIOA
#define ENABLE_B_Pin LL_GPIO_PIN_0
#define ENABLE_B_GPIO_Port GPIOB
#define MOTOR1_ENC_A_Pin LL_GPIO_PIN_8
#define MOTOR1_ENC_A_GPIO_Port GPIOA
#define MOTOR1_ENC_B_Pin LL_GPIO_PIN_9
#define MOTOR1_ENC_B_GPIO_Port GPIOA
#define INB2_Pin LL_GPIO_PIN_10
#define INB2_GPIO_Port GPIOA
#define BTN1_Pin LL_GPIO_PIN_11
#define BTN1_GPIO_Port GPIOA
#define BTN2_Pin LL_GPIO_PIN_12
#define BTN2_GPIO_Port GPIOA
#define LED2_Pin LL_GPIO_PIN_3
#define LED2_GPIO_Port GPIOB
#define SWT1_Pin LL_GPIO_PIN_6
#define SWT1_GPIO_Port GPIOB
#define SWT2_Pin LL_GPIO_PIN_7
#define SWT2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
