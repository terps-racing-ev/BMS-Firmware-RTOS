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

/* DS18B20 single-wire thermometer feature toggle (1 = enabled, 0 = disabled) */
#define DS18B20_FEATURE_ENABLED 0U

/**
  * @brief  MUX_SIG output mode selection
  * 
  * Define MUX_SIG_USE_OPEN_DRAIN to configure MUX_SIG1/2/3 as open-drain outputs
  * with external 5V pull-ups. In this mode:
  *   - GPIO_PIN_SET   = high-impedance (external pull-up pulls line HIGH)
  *   - GPIO_PIN_RESET = actively driven LOW
  * 
  * When undefined (default), pins use push-pull mode:
  *   - GPIO_PIN_SET   = actively driven HIGH (3.3V)
  *   - GPIO_PIN_RESET = actively driven LOW
  * 
  * Channel mapping (MUX_SIG3:MUX_SIG2:MUX_SIG1) is unchanged:
  *   Channel 0 = 000, Channel 1 = 001, ... Channel 7 = 111
  */
//#define MUX_SIG_USE_OPEN_DRAIN

/**
  * @brief  Dual-bank bootloader metadata layout
  *
  * These definitions mirror STM32-CAN-Bootloader/Core/Inc/bootloader.h.
  */
#define BOOT_METADATA_ADDRESS                 ((uint32_t)0x08007800)
#define BOOT_METADATA_MAGIC                   ((uint32_t)0xAB12AB12)
#define BOOT_METADATA_UPDATE_IN_PROGRESS      ((uint8_t)0xA5)
#define BOOT_BANK_A                           ((uint8_t)0U)
#define BOOT_BANK_B                           ((uint8_t)1U)
#define BOOT_BANK_A_ADDRESS                   ((uint32_t)0x08008000)
#define BOOT_BANK_B_ADDRESS                   ((uint32_t)0x08022000)
#define BOOT_BANK_SIZE                        ((uint32_t)0x1A000)

#define BOOT_STATUS_FLAG_ACTIVE_APP_VALID     ((uint8_t)(1U << 0))
#define BOOT_STATUS_FLAG_METADATA_VALID       ((uint8_t)(1U << 1))
#define BOOT_STATUS_FLAG_ACTIVE_BANK_B        ((uint8_t)(1U << 2))
#define BOOT_STATUS_FLAG_BANK_A_MARKED_VALID  ((uint8_t)(1U << 3))
#define BOOT_STATUS_FLAG_BANK_B_MARKED_VALID  ((uint8_t)(1U << 4))
#define BOOT_STATUS_FLAG_BANK_A_CRC_OK        ((uint8_t)(1U << 5))
#define BOOT_STATUS_FLAG_BANK_B_CRC_OK        ((uint8_t)(1U << 6))
#define BOOT_STATUS_FLAG_UPDATE_IN_PROGRESS   ((uint8_t)(1U << 7))

typedef struct {
  uint32_t magic;        /**< Metadata magic marker */
  uint8_t active_bank;   /**< Active application bank */
  uint8_t bank_a_valid;  /**< Bank A host-authoritative valid flag */
  uint8_t bank_b_valid;  /**< Bank B host-authoritative valid flag */
  uint8_t reserved0;     /**< Update transaction marker */
  uint32_t bank_a_crc;   /**< Expected reflected CRC32 for bank A */
  uint32_t bank_b_crc;   /**< Expected reflected CRC32 for bank B */
  uint32_t bank_a_size;  /**< Programmed image size for bank A */
  uint32_t bank_b_size;  /**< Programmed image size for bank B */
  uint32_t complement;   /**< Bitwise complement of BOOT_METADATA_MAGIC */
  uint32_t reserved1;    /**< Reserved for future metadata */
} BootMetadata_t;

/* Bootloader status flags - set at startup from boot metadata */
extern uint8_t g_bootloader_app_valid;
extern uint8_t g_bootloader_status_flags;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
