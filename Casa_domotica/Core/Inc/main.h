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
#define Detector_de_sonidos_Pin GPIO_PIN_0
#define Detector_de_sonidos_GPIO_Port GPIOC
#define Pulsador_luces_ON_Pin GPIO_PIN_1
#define Pulsador_luces_ON_GPIO_Port GPIOC
#define Pulsador_luces_ON_EXTI_IRQn EXTI1_IRQn
#define Pulsador_luces_OFF_Pin GPIO_PIN_2
#define Pulsador_luces_OFF_GPIO_Port GPIOC
#define Pulsador_luces_OFF_EXTI_IRQn EXTI2_IRQn
#define Pulsador_puerta_Pin GPIO_PIN_3
#define Pulsador_puerta_GPIO_Port GPIOC
#define Pulsador_puerta_EXTI_IRQn EXTI3_IRQn
#define Servo_Pin GPIO_PIN_1
#define Servo_GPIO_Port GPIOA
#define LDR_Pin GPIO_PIN_2
#define LDR_GPIO_Port GPIOA
#define Temperatura_Pin GPIO_PIN_3
#define Temperatura_GPIO_Port GPIOA
#define Pulsador_alarma_Pin GPIO_PIN_4
#define Pulsador_alarma_GPIO_Port GPIOA
#define Pulsador_alarma_EXTI_IRQn EXTI4_IRQn
#define Zumbador_pasivo_Pin GPIO_PIN_0
#define Zumbador_pasivo_GPIO_Port GPIOB
#define ECHO_Pin GPIO_PIN_9
#define ECHO_GPIO_Port GPIOE
#define LED_iluminaci_n_Pin GPIO_PIN_8
#define LED_iluminaci_n_GPIO_Port GPIOC
#define LED_temperatura_Pin GPIO_PIN_9
#define LED_temperatura_GPIO_Port GPIOC
#define LED_alarma_Pin GPIO_PIN_8
#define LED_alarma_GPIO_Port GPIOA
#define TRIG_Pin GPIO_PIN_10
#define TRIG_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
