/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#define DC_PUMP_1_EN_2_Pin GPIO_PIN_3
#define DC_PUMP_1_EN_2_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define DC_PUMP_4_EN_2_Pin GPIO_PIN_6
#define DC_PUMP_4_EN_2_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define DC_PUMP_3_EN_2_Pin GPIO_PIN_2
#define DC_PUMP_3_EN_2_GPIO_Port GPIOC
#define DC_PUMP_3_EN_1_Pin GPIO_PIN_3
#define DC_PUMP_3_EN_1_GPIO_Port GPIOC
#define DC_PUMP_3_PWM_Pin GPIO_PIN_2
#define DC_PUMP_3_PWM_GPIO_Port GPIOA
#define DC_PUMP_1_PWM_Pin GPIO_PIN_3
#define DC_PUMP_1_PWM_GPIO_Port GPIOA
#define DC_PUMP_4_PWM_Pin GPIO_PIN_5
#define DC_PUMP_4_PWM_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define STEPPER_1_DIR_Pin GPIO_PIN_2
#define STEPPER_1_DIR_GPIO_Port GPIOB
#define STEPPER_1_EN_Pin GPIO_PIN_12
#define STEPPER_1_EN_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define SERVO_2_PWM_Pin GPIO_PIN_6
#define SERVO_2_PWM_GPIO_Port GPIOC
#define STEPPER_1_STEP_Pin GPIO_PIN_8
#define STEPPER_1_STEP_GPIO_Port GPIOC
#define SERVO_1_PWM_Pin GPIO_PIN_9
#define SERVO_1_PWM_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define DC_PUMP_1_EN_1_Pin GPIO_PIN_1
#define DC_PUMP_1_EN_1_GPIO_Port GPIOD
#define DC_PUMP_4_EN_1_Pin GPIO_PIN_2
#define DC_PUMP_4_EN_1_GPIO_Port GPIOD
#define DC_PUMP_2_EN_2_Pin GPIO_PIN_4
#define DC_PUMP_2_EN_2_GPIO_Port GPIOD
#define DC_PUMP_2_EN_1_Pin GPIO_PIN_5
#define DC_PUMP_2_EN_1_GPIO_Port GPIOD
#define RELAY_2_Pin GPIO_PIN_12
#define RELAY_2_GPIO_Port GPIOG
#define DC_PUMP_2_PWM_Pin GPIO_PIN_3
#define DC_PUMP_2_PWM_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define RELAY_1_Pin GPIO_PIN_1
#define RELAY_1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
