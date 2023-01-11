/*
 * flash_app.c
 *
 *  Created on: 5 Jan 2023
 *      Author: PGa Reverse engineering ground water sensor
 *      https://controllerstech.com/flash-programming-in-stm32/
 */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>

#include "flash_app.h"

/* Private function prototypes -----------------------------------------------*/
static uint32_t FlashAppWriteData(uint32_t address, uint64_t *data, uint16_t size);
static void FlashAppReadData(uint32_t address, uint64_t *rxBuffer, uint16_t size);
static uint32_t FlashAppErasePage(const uint16_t page);

static uint32_t FlashAppFindConfig(uint32_t address);
static uint32_t FlashAppFindFreeSpotConfig(uint32_t address);

static uint32_t FlashAppFindFreeSpotMeasurement(uint32_t startAddress);
static bool FlashAppHasMeasurementBeenSent(const uint32_t address);
static void FlashAppReadMeasurement(const uint32_t address, uint64_t *rxBuffer);

/* Exported functions --------------------------------------------------------*/
static uint32_t flashAddressCurrentConfig = FLASH_START_ADDRESS_CONFIG;
static uint32_t flashAddressNextFreeConfig = FLASH_START_ADDRESS_CONFIG;

void FlashAppReadConfig(CONFIG_TYPE *config) {
    APP_LOG(TS_OFF, VLEVEL_M, "Reading config start\r\n");

    // Go through entire memory page to find where the config is stored
    flashAddressCurrentConfig = FlashAppFindConfig(flashAddressCurrentConfig);

    uint32_t address = flashAddressCurrentConfig;
    uint8_t size = CONFIG_SIZE;

    // Get config data
    while (true) {
        // Little endian
        *config++ = *(__IO CONFIG_TYPE*) address;

        if ((--size) == 0) {
            return;
        }

        address += CONFIG_TYPE_SIZE;
    }

    APP_LOG(TS_OFF, VLEVEL_M, "Reading config done | Config address: %d\r\n", flashAddressCurrentConfig);
}

/*
 * Writes a given config into flash.
 */
bool FlashAppWriteConfig(CONFIG_TYPE *config) {
    static CONFIG_TYPE currentConfig[CONFIG_SIZE];

    // Read existing config
    FlashAppReadConfig(currentConfig);

    uint64_t dataToWrite = 0;
    bool changesFound = false;

    // Compare old config with new one (Little endian)
    for (uint16_t i = 0; i < CONFIG_SIZE; i++) {
        if (!changesFound && config[i] != currentConfig[i]) {
            changesFound = true;
        }

        dataToWrite += ((uint64_t) config[i]) << (i * CONFIG_TYPE_SIZE);
    }

    // If existing config is the same as the new one, ignore
    if (!changesFound) {
        return true;
    }

    // Go to next address
    flashAddressNextFreeConfig = FlashAppFindFreeSpotConfig(flashAddressNextFreeConfig);

    uint8_t errors = 0;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    // Program the user Flash area double word by double word
    while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flashAddressNextFreeConfig, dataToWrite) != HAL_OK) {
        // If 3 errors have occurred whilst trying to write to memory, abort
        if (++errors == 3) {
            return false;
        }
    }

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();

    return true;
}

/*
 * When flash memory is reset, it's set to 1. Once a cell is set to 0, it can't be set to 1 unless its erased.
 */
void FlashAppMeasurementHasBeenSent(const uint32_t address) {
    static uint64_t rxBuffer[1];

    // Read first 8 bytes
    FlashAppReadData(address, rxBuffer, 1);

    // Remove status flag (see FLASH_MEMORY_STATUS_FLAG_BYTE)
    *rxBuffer = FLASH_MEMORY_STATUS_FLAG_REMOVAL_MASK & *rxBuffer;

    // Rewrite data
    FlashAppWriteData(address, rxBuffer, 1);
}

/*
 * Reads valid unsent measurements and gets them ready to be sent.
 */
void FlashAppReadUnsentMeasurements(void) {
    uint32_t address;
    static uint64_t rxBuffer[4];
    static uint8_t measurement[SIZE_MEASUREMENT];
    uint8_t measurementPointer = 0;

    // Go through the entire measurement flash storage
    for (uint16_t page = FLASH_START_PAGE_MEASUREMENTS; page <= FLASH_END_PAGE_MEASUREMENTS; page++) {
        address = FLASH_PAGE_START_ADDRESS(page);

        for (uint32_t endAddress = address + BYTES_USED_PER_PAGE; address < endAddress; address += SIZE_MEASUREMENT) {
            // Check if current double word length memory content is empty (64 bits set to 1)
            if ((~(*(__IO uint64_t*) address)) == 0) {
                continue;
            }

            // Check if the measurement has already been sent and if it's already been added into RAM
            if (!FlashAppHasMeasurementBeenSent(address) && !SensorAppIsMeasurementInRAM(address)) {
                // Get the measurement value
                FlashAppReadMeasurement(address, rxBuffer);

                APP_LOG(TS_OFF, VLEVEL_M, "Read unsent measurement at: %d\r\n", address);

                measurementPointer = 0;

                // Convert 64 bit values into an array of 8 bit values, little endian (LSB is first)
                for (uint8_t i = 0; i < 3; i++) {
                    for (uint8_t j = 0; j < 8; j++) {
                        measurement[measurementPointer++] = (uint8_t) (rxBuffer[i] >> (j * 8));
                    }
                }

                // Add measurement to RAM
                if (!SensorAppAddMeasurementToStorage(measurement, false)) {
                    return;
                }
            }
        }
    }
}

/* Private Functions Definition -----------------------------------------------*/

/*
 * Finds the current config.
 */
static uint32_t FlashAppFindConfig(uint32_t address) {
    while (address <= FLASH_END_ADDRESS_CONFIG) {
        // If current double word length memory content is empty (64 bits set to 1), it means previous cell had config
        if ((~(*(__IO uint64_t*) address)) == 0) {
            if (address != FLASH_START_ADDRESS_CONFIG) {
                address -= 8;
            }

            return address;
        }

        address += 8;
    }

    return FLASH_START_ADDRESS_CONFIG;
}

/*
 * Finds a free spot for writing config. Returns address of free spot.
 */
static uint32_t FlashAppFindFreeSpotConfig(uint32_t address) {
    while (address <= FLASH_END_ADDRESS_CONFIG) {
        // If current double word length memory content is empty (64 bits set to 1), this is the spot
        if ((~(*(__IO uint64_t*) address)) == 0) {
            return address;
        }

        address += 8;
    }

    // If nothing was found before end of page, erase page
    FlashAppErasePage(FLASH_PAGE_CONFIG);

    return FLASH_START_ADDRESS_CONFIG;
}

/*
 * Read a measurement from flash.
 */
static void FlashAppReadMeasurement(const uint32_t address, uint64_t *rxBuffer) {
    FlashAppReadData(address, rxBuffer, SIZE_MEASUREMENT / 8);

    // Remove status flag (see FLASH_MEMORY_STATUS_FLAG_BYTE)
    *rxBuffer = FLASH_MEMORY_STATUS_FLAG_REMOVAL_MASK & *rxBuffer;

    return;
}

/*
 * Returns if a measurement has been sent or not.
 */
static bool FlashAppHasMeasurementBeenSent(const uint32_t address) {
    // Measurement has been marked as sent if status flags have been set to 0
    return ((*(__IO uint8_t*) (address + FLASH_MEMORY_STATUS_FLAG_BYTE)) == 0);
}

/*
 * Check page if all data is ready to be erased.
 * Not all bytes are used (see BYTES_USED_PER_PAGE).
 */
static bool FlashAppIsPageReadyToBeErased(const uint16_t page) {
    uint32_t address = FLASH_PAGE_START_ADDRESS(page);

    for (uint32_t endAddress = address + BYTES_USED_PER_PAGE; address < endAddress; address += SIZE_MEASUREMENT) {
        if (!FlashAppHasMeasurementBeenSent(address)) {
            return false;
        }
    }

    return true;
}

/*
 * Finds a spot within the flash to write an entire measurement.
 */
static uint32_t FlashAppFindFreeSpotMeasurement(uint32_t startAddress) {
    // Throw error if invalid memory address
    if (startAddress % 8) {
        return FLASH_MEMORY_STATUS_INVALID_ADDRESS;
    }

    uint16_t page = (startAddress - FLASH_BASE) / FLASH_PAGE_SIZE;
    uint32_t address = startAddress;
    bool overflow = false;

    while (true) {
        // If address outside of BYTES_USED_PER_PAGE, go to next page
        if (address >= (FLASH_PAGE_START_ADDRESS(page) + BYTES_USED_PER_PAGE)) {
            // If out of bounds, start over again
            if (page == FLASH_END_PAGE_MEASUREMENTS) {
                page = FLASH_START_PAGE_MEASUREMENTS;
                address = FLASH_START_ADDRESS_MEASUREMENTS;
                overflow = true;
            } else {
                address = FLASH_PAGE_START_ADDRESS(++page);
            }
        }

        // Check if all bits are set to 1
        if ((~(*(__IO uint64_t*) address)) == 0) {
            return address;
        }

        if (overflow && address == startAddress) {
            return FLASH_MEMORY_STATUS_CLEAR_MEMORY;
        }

        address += SIZE_MEASUREMENT;
    }
}

/*
 * Write given data into flash.
 */
uint32_t FlashAppWriteMeasurement(uint8_t *data) {
    static uint32_t measurementFlashAddress = FLASH_START_ADDRESS_MEASUREMENTS;

    uint32_t status = FlashAppFindFreeSpotMeasurement(measurementFlashAddress);

    switch (status) {
        // If there is not enough space anymore, clear a page
        case FLASH_MEMORY_STATUS_CLEAR_MEMORY: {
            bool pageErased = false;

            // Erase pages if possible
            for (uint16_t page = FLASH_START_PAGE_MEASUREMENTS; page <= FLASH_END_PAGE_MEASUREMENTS; page++) {
                if (FlashAppIsPageReadyToBeErased(page)) {
                    FlashAppErasePage(page);

                    // Reset config pointer to a valid pointer
                    if (!pageErased) {
                        pageErased = true;
                        measurementFlashAddress = FLASH_PAGE_START_ADDRESS(page);
                    }
                }
            }

            // Flash memory is full and couldn't be reset
            if (!pageErased) {
                APP_LOG(TS_OFF, VLEVEL_M, "Flash memory is full and can't be cleared!!!\r\n");
                return FLASH_MEMORY_ERROR_COULD_NOT_CLEAR;
            }

            break;
        }
        case FLASH_MEMORY_STATUS_INVALID_ADDRESS: {
            return FLASH_MEMORY_ERROR_INVALID_ADDRESS;
        }
        default: {
            measurementFlashAddress = status;
        }
    }

    uint32_t startAddress = measurementFlashAddress;
    uint64_t dataToWrite;
    uint8_t errors = 0;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    // Write 8 Bytes at a time
    for (uint16_t i = 0; i < SIZE_MEASUREMENT; i += 8) {
        dataToWrite = 0;

        // Group 8 bytes into one 64 bit value
        for (uint8_t j = 0; j < 8; j++) {
            // 6th byte of time and date: Set to 0xFF as flag
            if (i == 0 && j == FLASH_MEMORY_STATUS_FLAG_BYTE) {
                dataToWrite += 0xFF << (j * 8);
            } else {
                dataToWrite += ((uint64_t) *(data + i + j)) << (j * 8);
            }
        }

        while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, measurementFlashAddress, dataToWrite) != HAL_OK) {
            // If 3 errors have occurred whilst trying to write to memory, abort
            if (++errors == 3) {
                return HAL_FLASH_GetError();
            }
        }

        errors = 0;
        measurementFlashAddress += 8;
    }

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();

    APP_LOG(TS_OFF, VLEVEL_M, "Measurement address: %d\r\n", measurementFlashAddress);

    return startAddress;
}

/*
 * Write data into flash, starting at a given address.
 */
static uint32_t FlashAppWriteData(uint32_t address, uint64_t *data, uint16_t size) {
    uint8_t errors = 0;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    // Write 8 Bytes at a time
    while (true) {
        while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, *data) != HAL_OK) {
            // If 3 errors have occurred whilst trying to write to memory, abort
            if (++errors == 3) {
                return HAL_FLASH_GetError();
            }
        }

        // Check if there is no more data to write
        if ((--size) == 0) {
            break;
        }

        errors = 0;
        address += 8;
        data++;
    }

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();

    return 0;
}

/*
 * Fills a given buffer by reading at a given address.
 */
static void FlashAppReadData(uint32_t address, uint64_t *rxBuffer, uint16_t size) {
    while (true) {
        *rxBuffer++ = *(__IO uint64_t*) address;

        if ((--size) == 0) {
            return;
        }

        address += 8;
    }
}

/*
 * Erases a given page.
 */
static uint32_t FlashAppErasePage(const uint16_t page) {
    static FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t errorStatus;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    // Fill EraseInit structure
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page = page;
    EraseInitStruct.NbPages = 1;

    // Check if an error has occurred while erasing the page
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &errorStatus) != HAL_OK) {
        return HAL_FLASH_GetError();
    }

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();

    APP_LOG(TS_OFF, VLEVEL_M, "Page %d erased: %d\r\n", page, errorStatus);

    return errorStatus;
}
