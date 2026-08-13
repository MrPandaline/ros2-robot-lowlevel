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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LEFT_AIN_2_Pin GPIO_PIN_13
#define LEFT_AIN_2_GPIO_Port GPIOC
#define LEFT_AIN_1_Pin GPIO_PIN_14
#define LEFT_AIN_1_GPIO_Port GPIOC
#define RIGHT_AIN1_Pin GPIO_PIN_15
#define RIGHT_AIN1_GPIO_Port GPIOC
#define TIM5_CH1_ENC_LEFT_Pin GPIO_PIN_0
#define TIM5_CH1_ENC_LEFT_GPIO_Port GPIOA
#define TIM5_CH2_ENC_LEFT_Pin GPIO_PIN_1
#define TIM5_CH2_ENC_LEFT_GPIO_Port GPIOA
#define TIM9_CH1_PWM_SERVO_4_Pin GPIO_PIN_2
#define TIM9_CH1_PWM_SERVO_4_GPIO_Port GPIOA
#define RIGHT_AIN2_Pin GPIO_PIN_3
#define RIGHT_AIN2_GPIO_Port GPIOA
#define STBY_Pin GPIO_PIN_4
#define STBY_GPIO_Port GPIOA
#define TIM2_CH1_ENC_RIGHT_Pin GPIO_PIN_5
#define TIM2_CH1_ENC_RIGHT_GPIO_Port GPIOA
#define TIM3_CH1_PWM_SERVO_3_Pin GPIO_PIN_6
#define TIM3_CH1_PWM_SERVO_3_GPIO_Port GPIOA
#define TIM3_CH2_PWM_SERVO_5_Pin GPIO_PIN_7
#define TIM3_CH2_PWM_SERVO_5_GPIO_Port GPIOA
#define EXTI1_BUMP1_Pin GPIO_PIN_1
#define EXTI1_BUMP1_GPIO_Port GPIOB
#define EXTI2_BUMP2_Pin GPIO_PIN_2
#define EXTI2_BUMP2_GPIO_Port GPIOB
#define TIM1_CH1_PWM_LEFT_Pin GPIO_PIN_8
#define TIM1_CH1_PWM_LEFT_GPIO_Port GPIOA
#define TIM1_CH2_PWM_RIGHT_Pin GPIO_PIN_9
#define TIM1_CH2_PWM_RIGHT_GPIO_Port GPIOA
#define UART1_RX_ORANGE_Pin GPIO_PIN_10
#define UART1_RX_ORANGE_GPIO_Port GPIOA
#define UART1_TX_ORANGE_Pin GPIO_PIN_15
#define UART1_TX_ORANGE_GPIO_Port GPIOA
#define TIM2_CH2_ENC_RIGHT_Pin GPIO_PIN_3
#define TIM2_CH2_ENC_RIGHT_GPIO_Port GPIOB
#define TIM4_CH1_PWM_SERVO_2_Pin GPIO_PIN_6
#define TIM4_CH1_PWM_SERVO_2_GPIO_Port GPIOB
#define TIM4_CH2_PWM_SERVO_1_Pin GPIO_PIN_7
#define TIM4_CH2_PWM_SERVO_1_GPIO_Port GPIOB
#define I2C1_SCL_MPU_NA226_Pin GPIO_PIN_8
#define I2C1_SCL_MPU_NA226_GPIO_Port GPIOB
#define I2C1_SDA_MPU_NA226_Pin GPIO_PIN_9
#define I2C1_SDA_MPU_NA226_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
