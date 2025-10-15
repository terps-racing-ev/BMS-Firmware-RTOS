/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : cell_temp_handler.c
  * @brief          : Cell temperature monitoring handler implementation
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cell_temp_handler.h"
#include "can_manager.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static temp_monitor_state_t temp_state = {0};
static const uint32_t adc_channels[NUM_ADC_CHANNELS] = {
    ADC_CH_1, ADC_CH_2, ADC_CH_3, ADC_CH_4, ADC_CH_5, ADC_CH_6, ADC_CH_7
};

// ADC channel enabled flags - compile time configuration
static const uint8_t adc_channel_enabled[NUM_ADC_CHANNELS] = {
    ADC1_ENABLED, ADC2_ENABLED, ADC3_ENABLED, ADC4_ENABLED, 
    ADC5_ENABLED, ADC6_ENABLED, ADC7_ENABLED
};

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef CellTemp_ConfigureADCChannel(uint32_t channel);
static HAL_StatusTypeDef CellTemp_SendTemperatureMessage(uint8_t msg_index, uint8_t start_therm_idx);
static uint8_t CellTemp_IsADCEnabled(uint8_t adc_index);

/* Function Implementations --------------------------------------------------*/

/**
  * @brief  Initialize cell temperature monitoring system
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CellTemp_Init(void)
{
    // Initialize state structure
    memset(&temp_state, 0, sizeof(temp_monitor_state_t));

    // Initialize thermistor data
    for (uint8_t i = 0; i < TOTAL_THERMISTORS; i++) {
        temp_state.thermistors[i].adc_index = i / MUX_CHANNELS;
        temp_state.thermistors[i].mux_channel = i % MUX_CHANNELS;
        temp_state.thermistors[i].temperature = -127.0f; // Invalid temperature marker
        temp_state.thermistors[i].raw_adc = 0;
        temp_state.thermistors[i].last_read_time = 0;
    }

    // Note: ADC calibration is already done in main() before RTOS starts

    // Initialize MUX control signals to 0 (without calling osDelay)
    HAL_GPIO_WritePin(MUX_SIG1_PORT, MUX_SIG1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_SIG2_PORT, MUX_SIG2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_SIG3_PORT, MUX_SIG3_PIN, GPIO_PIN_RESET);

    return HAL_OK;
}

/**
  * @brief  Configure ADC for specific channel
  * @param  channel: ADC channel to configure
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef CellTemp_ConfigureADCChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5; // Long sampling for high impedance
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    
    return HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/**
  * @brief  Read single ADC value from specified channel
  * @param  adc_channel: ADC channel to read
  * @retval ADC value (0-4095)
  */
uint16_t CellTemp_ReadADC(uint32_t adc_channel)
{
    uint16_t adc_value = 0;
    
    // Configure ADC for the specified channel
    if (CellTemp_ConfigureADCChannel(adc_channel) != HAL_OK) {
        return 0;
    }
    
    // Start ADC conversion
    if (HAL_ADC_Start(&hadc1) == HAL_OK) {
        // Wait for conversion to complete
        if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
            // Get ADC value
            adc_value = HAL_ADC_GetValue(&hadc1);
        }
        // Stop ADC
        HAL_ADC_Stop(&hadc1);
    }
    
    return adc_value;
}

/**
  * @brief  Set MUX channel using 3-bit digital control signals
  * @param  channel: MUX channel to select (0-7)
  * @retval None
  */
void CellTemp_SetMuxChannel(uint8_t channel)
{
    // Validate channel range
    if (channel > 7) {
        return;
    }
    
    // Set MUX control signals based on 3-bit channel value
    // MUX_SIG1 = bit 0
    // MUX_SIG2 = bit 1  
    // MUX_SIG3 = bit 2
    
    if (channel & 0x01) {
        HAL_GPIO_WritePin(MUX_SIG1_PORT, MUX_SIG1_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MUX_SIG1_PORT, MUX_SIG1_PIN, GPIO_PIN_RESET);
    }
    
    if (channel & 0x02) {
        HAL_GPIO_WritePin(MUX_SIG2_PORT, MUX_SIG2_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MUX_SIG2_PORT, MUX_SIG2_PIN, GPIO_PIN_RESET);
    }
    
    if (channel & 0x04) {
        HAL_GPIO_WritePin(MUX_SIG3_PORT, MUX_SIG3_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MUX_SIG3_PORT, MUX_SIG3_PIN, GPIO_PIN_RESET);
    }
    
    // Allow time for MUX to settle (2 ticks = 2ms at default 1ms tick rate)
    osDelay(2);
}

/**
  * @brief  Calculate temperature from thermistor ADC reading using B-parameter equation
  * @param  adc_value: Raw ADC value (0-4095)
  * @retval Temperature in degrees Celsius
  */
float CellTemp_CalculateTemperature(uint16_t adc_value)
{
    // Convert ADC value to voltage
    float voltage = ((float)adc_value / ADC_RESOLUTION) * ADC_VREF;
    
    // Check for disconnected thermistor (very low ADC reading means low voltage = open circuit)
    if (adc_value < 10) {  // Less than ~8mV indicates disconnected sensor
        return -127.0f;  // Return obvious error value
    }
    
    // Calculate thermistor resistance using voltage divider
    // Circuit: 3.3V -> 10kΩ pullup -> ADC_input -> Thermistor -> GND
    // Voltage divider: V_adc = 3.3V * (R_thermistor / (R_pullup + R_thermistor))
    // Solving for R_thermistor: R_thermistor = (V_adc * R_pullup) / (3.3V - V_adc)
    
    float r_thermistor;
    
    if (voltage >= 3.29f) {  // Close to 3.3V limit, very high resistance (cold)
        r_thermistor = PULLUP_RESISTOR * 100.0f;  // Assume very high resistance
    } else {
        // Normal calculation with 3.3V supply
        r_thermistor = (voltage * PULLUP_RESISTOR) / (ADC_VREF - voltage);
    }
    
    // Handle edge cases
    if (r_thermistor <= 0) {
        return 125.0f; // Return maximum temperature for very low resistance
    }
    
    // Calculate temperature using B-parameter equation (derived from Steinhart-Hart)
    // 1/T = 1/T0 + (1/B) * ln(R/R0)
    // Where T0 = 298.15K (25°C), R0 = resistance at 25°C, B = B-value
    float ln_ratio = logf(r_thermistor / THERMISTOR_R25);
    float temp_kelvin = 1.0f / ((1.0f / REFERENCE_TEMP_K) + (ln_ratio / THERMISTOR_B_VALUE));
    
    // Convert to Celsius
    float temp_celsius = temp_kelvin - 273.15f;
    
    // Clamp to reasonable range
    if (temp_celsius < -40.0f) {
        temp_celsius = -40.0f;
    } else if (temp_celsius > 125.0f) {
        temp_celsius = 125.0f;
    }
    
    return temp_celsius;
}

/**
  * @brief  Read temperature from specific thermistor
  * @param  adc_index: ADC channel index (0-6)
  * @param  mux_channel: MUX channel (0-7)
  * @retval Temperature in degrees Celsius
  */
float CellTemp_ReadThermistor(uint8_t adc_index, uint8_t mux_channel)
{
    // Validate inputs
    if (adc_index >= NUM_ADC_CHANNELS || mux_channel >= MUX_CHANNELS) {
        return -127.0f;
    }
    
    // Set MUX to desired channel
    CellTemp_SetMuxChannel(mux_channel);
    
    // Read ADC value from the specified ADC channel
    uint16_t adc_value = CellTemp_ReadADC(adc_channels[adc_index]);
    
    // Calculate temperature
    return CellTemp_CalculateTemperature(adc_value);
}

/**
  * @brief  Main temperature monitoring task (to be called from ReadCellTemps)
  * @param  argument: Not used
  * @retval None
  */
void CellTemp_MonitorTask(void *argument)
{
    // Initialize the temperature monitoring system
    if (CellTemp_Init() != HAL_OK) {
        // Handle initialization error - blink would go here if we had an LED
        while(1) {
            osDelay(1000);  // Wait 1 second (1000 ticks at 1ms tick rate)
        }
    }
    
    // Note: CAN data is sent after each complete cycle automatically
    
    /* Infinite loop */
    for(;;)
    {
        uint32_t current_time = osKernelGetTickCount();
        
        // Check if current ADC channel is enabled
        if (CellTemp_IsADCEnabled(temp_state.current_adc)) {
            // Read current thermistor
            thermistor_data_t *current_therm = &temp_state.thermistors[temp_state.current_index];
            
            // Set MUX channel
            CellTemp_SetMuxChannel(temp_state.current_mux);
            
            // Read ADC value
            current_therm->raw_adc = CellTemp_ReadADC(adc_channels[temp_state.current_adc]);
            
            // Calculate temperature
            current_therm->temperature = CellTemp_CalculateTemperature(current_therm->raw_adc);
            current_therm->last_read_time = current_time;
        } else {
            // ADC disabled - mark thermistor as invalid
            temp_state.thermistors[temp_state.current_index].temperature = -127.0f;
            temp_state.thermistors[temp_state.current_index].raw_adc = 0;
        }
        
        // Move to next thermistor
        temp_state.current_index++;
        temp_state.current_mux++;
        
        // Check if we need to move to next ADC channel
        if (temp_state.current_mux >= MUX_CHANNELS) {
            // Completed all 8 MUX channels for current ADC (took 1 second)
            // Now send CAN messages for ALL enabled ADCs (not just the one we finished)
            // This ensures every enabled ADC sends its data every second
            
            for (uint8_t adc = 0; adc < NUM_ADC_CHANNELS; adc++) {
                if (CellTemp_IsADCEnabled(adc)) {
                    // Calculate which message(s) this ADC uses
                    // Each ADC has 8 thermistors, each message has 4 thermistors
                    // So each ADC uses 2 messages
                    uint8_t start_msg_idx = adc * 2;
                    uint8_t start_therm_idx = adc * MUX_CHANNELS;
                    
                    // Send 2 messages for this ADC (4 thermistors per message)
                    CellTemp_SendTemperatureMessage(start_msg_idx, start_therm_idx);
                    osDelay(5); // Small delay between messages
                    CellTemp_SendTemperatureMessage(start_msg_idx + 1, start_therm_idx + 4);
                    osDelay(5); // Small delay before next ADC's messages
                }
            }
            
            temp_state.current_mux = 0;
            temp_state.current_adc++;
            
            // Check if we completed full cycle of all ADCs
            if (temp_state.current_adc >= NUM_ADC_CHANNELS) {
                temp_state.current_adc = 0;
                temp_state.current_index = 0;
                temp_state.cycle_count++;
            }
        }
        
        // Wait 125ms (TEMP_READ_DELAY_MS) before next reading (only if ADC enabled)
        // With 8 MUX channels × 125ms = 1 second per ADC
        if (CellTemp_IsADCEnabled(temp_state.current_adc)) {
            osDelay(TEMP_READ_DELAY_MS);
        } else {
            osDelay(1); // 1ms delay for disabled channels to quickly skip
        }
    }
}

/**
  * @brief  Check if ADC channel is enabled
  * @param  adc_index: ADC channel index (0-6)
  * @retval 1 if enabled, 0 if disabled
  */
static uint8_t CellTemp_IsADCEnabled(uint8_t adc_index)
{
    if (adc_index >= NUM_ADC_CHANNELS) {
        return 0;
    }
    return adc_channel_enabled[adc_index];
}

/**
  * @brief  Send single temperature CAN message
  * @param  msg_index: Message index (0-13, for 14 messages total)
  * @param  start_therm_idx: Starting thermistor index
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef CellTemp_SendTemperatureMessage(uint8_t msg_index, uint8_t start_therm_idx)
{
    uint8_t TxData[8];
    
    // Check if this message corresponds to a disabled ADC channel
    // Each message covers 4 thermistors, determine which ADC channel(s) it uses
    uint8_t first_adc = start_therm_idx / MUX_CHANNELS;
    uint8_t last_adc = (start_therm_idx + 3) / MUX_CHANNELS;
    
    // Skip message if all ADC channels for this message are disabled
    uint8_t any_enabled = 0;
    for (uint8_t adc = first_adc; adc <= last_adc && adc < NUM_ADC_CHANNELS; adc++) {
        if (CellTemp_IsADCEnabled(adc)) {
            any_enabled = 1;
            break;
        }
    }
    
    if (!any_enabled) {
        return HAL_OK; // Skip sending this message
    }
    
    // Prepare CAN message with 4 thermistor readings per message
    // Temperature format: temp_celsius * 10 (0.1°C resolution), little endian
    
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t therm_idx = start_therm_idx + i;
        int16_t temp_data = 0x8000; // Default invalid temperature marker
        
        // Check if this thermistor's ADC is enabled
        if (therm_idx < TOTAL_THERMISTORS) {
            uint8_t therm_adc = therm_idx / MUX_CHANNELS;
            if (CellTemp_IsADCEnabled(therm_adc) && 
                temp_state.thermistors[therm_idx].temperature > -126.0f) {
                temp_data = (int16_t)(temp_state.thermistors[therm_idx].temperature * 10.0f);
            }
        }
        
        // Pack temperature data (little endian)
        TxData[i * 2] = temp_data & 0xFF;         // LSB
        TxData[i * 2 + 1] = (temp_data >> 8) & 0xFF; // MSB
    }
    
    // Send via CAN Manager (non-blocking, queued)
    uint32_t can_id = CAN_TEMP_BASE_ID + msg_index;
    return CAN_SendMessage(can_id, TxData, 8, CAN_PRIORITY_NORMAL);
}

/**
  * @brief  Send temperature data over CAN bus (manual/on-demand)
  * @note   In normal operation, CAN messages are sent automatically after each
  *         ADC completes its 8 MUX readings (once per second per enabled ADC).
  *         This function can be called manually to send all messages at once.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef CellTemp_SendCANData(void)
{
    HAL_StatusTypeDef result = HAL_OK;
    
    // Send 14 CAN messages to cover all 56 thermistors (4 per message)
    // Only enabled ADC channels will have their messages sent
    for (uint8_t msg = 0; msg < 14; msg++) {
        uint8_t start_idx = msg * 4;
        
        if (CellTemp_SendTemperatureMessage(msg, start_idx) != HAL_OK) {
            result = HAL_ERROR;
        }
        
        // Small delay between messages to avoid bus congestion (5ms)
        osDelay(5);
    }
    
    return result;
}

/**
  * @brief  Get current temperature readings
  * @param  temperatures: Array to store temperature readings
  * @param  size: Size of array (should be TOTAL_THERMISTORS)
  * @retval Number of valid readings
  */
uint8_t CellTemp_GetTemperatures(float *temperatures, uint8_t size)
{
    uint8_t valid_count = 0;
    uint8_t max_count = (size < TOTAL_THERMISTORS) ? size : TOTAL_THERMISTORS;
    
    for (uint8_t i = 0; i < max_count; i++) {
        temperatures[i] = temp_state.thermistors[i].temperature;
        if (temperatures[i] > -126.0f) {
            valid_count++;
        }
    }
    
    return valid_count;
}

/**
  * @brief  Get monitoring statistics
  * @param  cycle_count: Pointer to store cycle count
  * @param  current_index: Pointer to store current thermistor index
  * @retval None
  */
void CellTemp_GetStats(uint32_t *cycle_count, uint8_t *current_index)
{
    if (cycle_count != NULL) {
        *cycle_count = temp_state.cycle_count;
    }
    
    if (current_index != NULL) {
        *current_index = temp_state.current_index;
    }
}