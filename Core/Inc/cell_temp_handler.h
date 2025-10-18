/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : cell_temp_handler.h
  * @brief          : Cell temperature monitoring handler header
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

#ifndef __CELL_TEMP_HANDLER_H
#define __CELL_TEMP_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can_ids.h"
#include <math.h>

/* Defines -------------------------------------------------------------------*/
#define NUM_ADC_CHANNELS 7          // Number of ADC channels (ADC1-ADC7)
#define MUX_CHANNELS 8              // Number of MUX channels per ADC (8:1 MUX)
#define TOTAL_THERMISTORS 56        // Total thermistors (7 ADCs × 8 MUX channels)

// ADC Channel Enable/Disable Configuration
// Set to 1 to enable, 0 to disable each ADC channel
#define ADC1_ENABLED 1              // ADC1 (Thermistors 0-7)
#define ADC2_ENABLED 1              // ADC2 (Thermistors 8-15)  - DISABLED
#define ADC3_ENABLED 1              // ADC3 (Thermistors 16-23) - DISABLED
#define ADC4_ENABLED 1              // ADC4 (Thermistors 24-31) - DISABLED
#define ADC5_ENABLED 1              // ADC5 (Thermistors 32-39) - DISABLED
#define ADC6_ENABLED 1              // ADC6 (Thermistors 40-47) - DISABLED
#define ADC7_ENABLED 1              // ADC7 (Thermistors 48-55) - DISABLED

// MUX Control Pin Definitions (from main.h)
#define MUX_SIG1_PIN    MUX_SIG1_Pin
#define MUX_SIG1_PORT   MUX_SIG1_GPIO_Port
#define MUX_SIG2_PIN    MUX_SIG2_Pin  
#define MUX_SIG2_PORT   MUX_SIG2_GPIO_Port
#define MUX_SIG3_PIN    MUX_SIG3_Pin
#define MUX_SIG3_PORT   MUX_SIG3_GPIO_Port

// ADC parameters
#define ADC_RESOLUTION 4095.0f      // 12-bit ADC resolution (0-4095)
#define ADC_VREF 3.3f               // ADC reference voltage (3.3V)

// Thermistor parameters (NTC 10k ohm @ 25°C, B-value 4300K)
#define THERMISTOR_R25 10000.0f     // Resistance at 25°C in ohms
#define THERMISTOR_B_VALUE 4300.0f  // B25/85 value of thermistor
#define PULLUP_RESISTOR 10000.0f    // Pull-up resistor value in ohms (to 3.3V)
#define REFERENCE_TEMP_K 298.15f    // Reference temperature in Kelvin (25°C)

// Temperature fault detection
#define TEMP_FAULT_DETECTION_ENABLED 1   // Set to 1 to enable, 0 to disable fault detection
#define TEMP_MIN_CELSIUS -20.0f          // Minimum safe cell temperature (°C)
#define TEMP_MAX_CELSIUS 60.0f           // Maximum safe cell temperature (°C)

// Task timing - Oversampling configuration
#define TEMP_OVERSAMPLE_PERIOD_MS 125    // Oversample each MUX channel for 125ms
#define TEMP_SAMPLE_INTERVAL_MS 10       // Take a sample every 10ms (12-13 samples per MUX channel)
#define TEMP_SAMPLES_PER_MUX (TEMP_OVERSAMPLE_PERIOD_MS / TEMP_SAMPLE_INTERVAL_MS)  // 12 samples

/* ADC Channel Mapping */
typedef enum {
    ADC_CH_1 = ADC_CHANNEL_5,    // PA0 -> ADC_CHANNEL_5 (ADC1_IN5)
    ADC_CH_2 = ADC_CHANNEL_6,    // PA1 -> ADC_CHANNEL_6 (ADC1_IN6)
    ADC_CH_3 = ADC_CHANNEL_11,   // PA6 -> ADC_CHANNEL_11 (ADC1_IN11)
    ADC_CH_4 = ADC_CHANNEL_8,    // PA3 -> ADC_CHANNEL_8 (ADC1_IN8)
    ADC_CH_5 = ADC_CHANNEL_9,    // PA4 -> ADC_CHANNEL_9 (ADC1_IN9)
    ADC_CH_6 = ADC_CHANNEL_10,   // PA5 -> ADC_CHANNEL_10 (ADC1_IN10)
    ADC_CH_7 = ADC_CHANNEL_15    // PB0 -> ADC_CHANNEL_15 (ADC1_IN15)
} adc_channel_t;

/* Structures ----------------------------------------------------------------*/
typedef struct {
    uint8_t adc_index;        // ADC channel index (0-6)
    uint8_t mux_channel;      // MUX channel (0-7)
    float temperature;        // Temperature in Celsius
    uint16_t raw_adc;         // Raw ADC value (averaged from oversampling)
    uint32_t last_read_time;  // Last reading timestamp
    uint32_t adc_accumulator; // Accumulator for oversampling
    uint16_t sample_count;    // Number of samples accumulated
} thermistor_data_t;

typedef struct {
    thermistor_data_t thermistors[TOTAL_THERMISTORS];
    uint8_t current_adc;      // Current ADC being read (0-6)
    uint8_t current_mux;      // Current MUX channel (0-7)
    uint8_t current_index;    // Current thermistor index (0-55)
    uint32_t cycle_count;     // Number of complete cycles
    uint8_t oversample_complete; // Flag indicating MUX channel oversampling is complete
} temp_monitor_state_t;

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern osMutexId_t CANHandle;

/* Function prototypes -------------------------------------------------------*/

/**
  * @brief  Initialize cell temperature monitoring system
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CellTemp_Init(void);

/**
  * @brief  Read single ADC value from specified channel
  * @param  adc_channel: ADC channel to read
  * @retval ADC value (0-4095)
  */
uint16_t CellTemp_ReadADC(uint32_t adc_channel);

/**
  * @brief  Set MUX channel using 3-bit digital control signals
  * @param  channel: MUX channel to select (0-7)
  * @retval None
  */
void CellTemp_SetMuxChannel(uint8_t channel);

/**
  * @brief  Calculate temperature from thermistor ADC reading
  * @param  adc_value: Raw ADC value (0-4095)
  * @retval Temperature in degrees Celsius
  */
float CellTemp_CalculateTemperature(uint16_t adc_value);

/**
  * @brief  Read temperature from specific thermistor
  * @param  adc_index: ADC channel index (0-6)
  * @param  mux_channel: MUX channel (0-7)
  * @retval Temperature in degrees Celsius
  */
float CellTemp_ReadThermistor(uint8_t adc_index, uint8_t mux_channel);

/**
  * @brief  Main temperature monitoring task (to be called from ReadCellTemps)
  * @param  argument: Not used
  * @retval None
  */
void CellTemp_MonitorTask(void *argument);

/**
  * @brief  Send temperature data over CAN bus
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CellTemp_SendCANData(void);

/**
  * @brief  Get current temperature readings
  * @param  temperatures: Array to store temperature readings
  * @param  size: Size of array (should be TOTAL_THERMISTORS)
  * @retval Number of valid readings
  */
uint8_t CellTemp_GetTemperatures(float *temperatures, uint8_t size);

/**
  * @brief  Get monitoring statistics
  * @param  cycle_count: Pointer to store cycle count
  * @param  current_index: Pointer to store current thermistor index
  * @retval None
  */
void CellTemp_GetStats(uint32_t *cycle_count, uint8_t *current_index);

#ifdef __cplusplus
}
#endif

#endif /* __CELL_TEMP_HANDLER_H */