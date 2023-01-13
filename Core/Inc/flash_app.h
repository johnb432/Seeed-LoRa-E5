/*
 * flash_app.h
 *
 *  Created on: 5 Jan 2023
 *      Author: PGa Reverse engineering ground water sensor
 *      https://controllerstech.com/flash-programming-in-stm32/
 */

#ifndef INC_FLASH_APP_H_
#define INC_FLASH_APP_H_

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "stdbool.h"

#include "sensor_app.h"
#include "sys_app.h"

#include "stm32wlxx_hal.h"
#include "stm32wlxx_hal_flash.h"

/* Exported macros -----------------------------------------------------------*/
/* Definitions ---------------------------------------------------------------*/
// A multiple of MEASUREMENT_SIZE_BYTES might not fit into FLASH_PAGE_SIZE
// This means there are going to be sections of a page that aren't used
#define BYTES_USED_PER_PAGE                     ((FLASH_PAGE_SIZE / (MEASUREMENT_SIZE_BYTES + MEASUREMENT_STATUS_SIZE_BYTES)) * MEASUREMENT_SIZE_BYTES)

#define FLASH_PAGE_START_ADDRESS(PAGE)          (FLASH_BASE + PAGE * FLASH_PAGE_SIZE)
#define FLASH_PAGE_END_ADDRESS(PAGE)            (FLASH_PAGE_START_ADDRESS(PAGE) + FLASH_PAGE_SIZE - 1)

#define FLASH_START_ADDRESS_CONFIG              FLASH_PAGE_START_ADDRESS(FLASH_PAGE_CONFIG)
#define FLASH_END_ADDRESS_CONFIG                (FLASH_START_ADDRESS_CONFIG + FLASH_PAGE_SIZE - 1)

#define FLASH_START_ADDRESS_MEASUREMENTS        FLASH_PAGE_START_ADDRESS(FLASH_START_PAGE_MEASUREMENTS)
#define FLASH_END_ADDRESS_MEASUREMENTS          FLASH_PAGE_END_ADDRESS(FLASH_END_PAGE_MEASUREMENTS)

// Flash memory status macros
#define FLASH_MEMORY_STATUS_INVALID_ADDRESS     0x00000000U
#define FLASH_MEMORY_STATUS_CLEAR_MEMORY        0x00000001U

// Flash memory error macros
#define FLASH_MEMORY_ERROR_NOT_ENOUGH_BYTES     0x0000000FU
#define FLASH_MEMORY_ERROR_COULD_NOT_CLEAR      0x000000FFU
#define FLASH_MEMORY_ERROR_INVALID_ADDRESS      0x00000FFFU

// The 5th 64-bit block of a measurement will be set to 0xFFFFFFFFFFFFFFFF if it hasn't been sent yet and 0 when it has been sent
#define MEASUREMENT_STATUS_SIZE_BYTES           8
#define MEASUREMENT_STATUS_ADDRESS(ADDRESS)     (ADDRESS + MEASUREMENT_SIZE_BYTES)

#define FLASH_MEMORY_STATUS_FLAG_REMOVAL_MASK   (~(((uint64_t) 0xFF) << (FLASH_MEMORY_STATUS_FLAG_BYTE * 8)))

/* Parameters ----------------------------------------------------------------*/
// Page 68 reserved for config data
#define FLASH_PAGE_CONFIG                       68U

// Pages 126 & 127 are reserved for LoRa!
// Page 69 - 125 are for measurements
#define FLASH_START_PAGE_MEASUREMENTS           69U
#define FLASH_END_PAGE_MEASUREMENTS             125U

/* Exported functions --------------------------------------------------------*/
void FlashAppReadConfig(CONFIG_TYPE *config);
bool FlashAppWriteConfig(CONFIG_TYPE *config);

void FlashAppReadUnsentMeasurements();
uint32_t FlashAppWriteMeasurement(uint8_t *data);
void FlashAppMeasurementHasBeenSent(const uint32_t address);

#endif /* INC_FLASH_APP_H_ */
