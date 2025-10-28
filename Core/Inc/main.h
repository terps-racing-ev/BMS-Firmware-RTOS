/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* BMS Reset Semaphore - used to signal reset request to BMSResetHandler task */
extern osSemaphoreId_t BMSResetSemHandle;
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
#define ADC1_Pin GPIO_PIN_0
#define ADC1_GPIO_Port GPIOA
#define ADC2_Pin GPIO_PIN_1
#define ADC2_GPIO_Port GPIOA
#define ADC4_Pin GPIO_PIN_3
#define ADC4_GPIO_Port GPIOA
#define ADC5_Pin GPIO_PIN_4
#define ADC5_GPIO_Port GPIOA
#define ADC6_Pin GPIO_PIN_5
#define ADC6_GPIO_Port GPIOA
#define ADC3_Pin GPIO_PIN_6
#define ADC3_GPIO_Port GPIOA
#define ADC7_Pin GPIO_PIN_0
#define ADC7_GPIO_Port GPIOB
#define MUX_SIG1_Pin GPIO_PIN_1
#define MUX_SIG1_GPIO_Port GPIOB
#define MUX_SIG2_Pin GPIO_PIN_8
#define MUX_SIG2_GPIO_Port GPIOA
#define MUX_SIG3_Pin GPIO_PIN_5
#define MUX_SIG3_GPIO_Port GPIOB
#define BMS_RESET_Pin GPIO_PIN_6
#define BMS_RESET_GPIO_Port GPIOB
#define ALERT_IN_Pin GPIO_PIN_7
#define ALERT_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
