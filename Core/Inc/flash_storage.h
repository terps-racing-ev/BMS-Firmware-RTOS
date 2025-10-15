/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : flash_storage.h
  * @brief          : Flash storage area definitions and utilities
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

#ifndef __FLASH_STORAGE_H
#define __FLASH_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Defines -------------------------------------------------------------------*/
// Storage area addresses - defined in linker script
extern uint32_t __storage_start__;
extern uint32_t __storage_end__;
extern uint32_t __storage_size__;

// Flash storage area definitions
#define STORAGE_START_ADDRESS   ((uint32_t)&__storage_start__)
#define STORAGE_END_ADDRESS     ((uint32_t)&__storage_end__)
#define STORAGE_SIZE            ((uint32_t)&__storage_size__)

// STM32L4 Flash page size is 2KB
#define FLASH_PAGE_SIZE         2048U
#define STORAGE_PAGE_COUNT      (STORAGE_SIZE / FLASH_PAGE_SIZE)  // Should be 8 pages

// Calculate which flash page number the storage starts at
// STM32L432KC has 256KB flash = 128 pages of 2KB each
// Storage starts at 0x8042000
// Page number = (address - 0x8000000) / 2048
#define STORAGE_START_PAGE      ((STORAGE_START_ADDRESS - 0x08000000) / FLASH_PAGE_SIZE)

/* Macros --------------------------------------------------------------------*/
/**
  * @brief  Check if address is within storage area
  * @param  addr: Address to check
  * @retval 1 if within storage, 0 otherwise
  */
#define IS_STORAGE_ADDRESS(addr) \
    (((addr) >= STORAGE_START_ADDRESS) && ((addr) < STORAGE_END_ADDRESS))

/**
  * @brief  Get offset within storage area
  * @param  addr: Address within storage
  * @retval Offset from start of storage area
  */
#define STORAGE_OFFSET(addr) \
    ((addr) - STORAGE_START_ADDRESS)

/* Public Function Prototypes ------------------------------------------------*/

/**
  * @brief  Get storage area start address
  * @retval Storage start address
  */
static inline uint32_t FlashStorage_GetStartAddress(void)
{
    return STORAGE_START_ADDRESS;
}

/**
  * @brief  Get storage area end address
  * @retval Storage end address
  */
static inline uint32_t FlashStorage_GetEndAddress(void)
{
    return STORAGE_END_ADDRESS;
}

/**
  * @brief  Get storage area size
  * @retval Storage size in bytes
  */
static inline uint32_t FlashStorage_GetSize(void)
{
    return STORAGE_SIZE;
}

/**
  * @brief  Get number of pages in storage area
  * @retval Number of 2KB pages
  */
static inline uint32_t FlashStorage_GetPageCount(void)
{
    return STORAGE_PAGE_COUNT;
}

/**
  * @brief  Get starting page number for storage area
  * @retval Flash page number
  */
static inline uint32_t FlashStorage_GetStartPage(void)
{
    return STORAGE_START_PAGE;
}

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_STORAGE_H */
