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
static uint32_t FlashAppFindConfig(uint32_t address);
static uint32_t FlashAppFindFreeSpotConfig(uint32_t address);

static bool FlashAppHasMeasurementBeenSent(const uint32_t address);
static uint32_t FlashAppFindFreeSpotMeasurement(uint32_t address);

static uint32_t FlashAppWriteData(uint32_t address, uint64_t *data, uint16_t size);
static void FlashAppReadData(uint32_t address, uint64_t *rxBuffer, uint16_t size);
static uint32_t FlashAppErasePage(const uint16_t page);
static bool FlashAppIsPageReadyToBeErased(const uint16_t page);

/* Exported functions --------------------------------------------------------*/
static uint32_t Flash_addressCurrentConfig = FLASH_START_ADDRESS_CONFIG;
static uint32_t Flash_addressNextFreeConfig = FLASH_START_ADDRESS_CONFIG;

/**
  * @brief  Reads config parameters and puts them into the passed array.
  * @param  Array to insert config parameters into
  */
void FlashAppReadConfig(CONFIG_TYPE *config) {
    APP_LOG(TS_OFF, VLEVEL_M, "Reading config start\r\n");

    // Go through entire memory page to find where the config is stored
    Flash_addressCurrentConfig = FlashAppFindConfig(Flash_addressCurrentConfig);

    uint32_t address = Flash_addressCurrentConfig;
    uint8_t size = CONFIG_SIZE;

    // Get config data
    while (true) {
        // Little endian
        *config++ = *(__IO CONFIG_TYPE*) address;

        if ((--size) == 0) {
            APP_LOG(TS_OFF, VLEVEL_M, "Reading config done | Config address: %d\r\n", Flash_addressCurrentConfig);
            return;
        }

        address += CONFIG_TYPE_SIZE_BYTES;
    }
}

/**
  * @brief  Writes a given config into flash.
  * @param  Array from which the config parameters should be written into flash
  * @retval bool - returns if operation was completed successfully
  */
bool FlashAppWriteConfig(CONFIG_TYPE *config) {
    static CONFIG_TYPE currentConfig[CONFIG_SIZE];

    // Read existing config
    FlashAppReadConfig(currentConfig);

    uint64_t dataToWrite = 0;
    bool changesFound = false;

    // Compare old config with new one (Little endian)
    for (uint16_t i = 0; i < CONFIG_SIZE; i++) {
        if (!changesFound && (config[i] != currentConfig[i])) {
            changesFound = true;
        }

        // Get data to write
        dataToWrite += ((uint64_t) config[i]) << (i * (CONFIG_TYPE_SIZE_BYTES * 8));
    }

    // If existing config is the same as the new one, ignore
    if (!changesFound) {
        return true;
    }

    // Go to next address
    Flash_addressNextFreeConfig = FlashAppFindFreeSpotConfig(Flash_addressNextFreeConfig);

    uint8_t errors = 0;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    // Try writing config 3 times at most
    while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Flash_addressNextFreeConfig, dataToWrite) != HAL_OK) {
        if (++errors == 3) {
            // If 3 errors have occurred whilst trying to write to flash, abort
            return false;
        }
    }

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();

    return true;
}

/**
  * @brief  Returns if the a measurement in flash has been marked as sent.
  * @param  address at which the measurement resides
  */
void FlashAppMeasurementHasBeenSent(const uint32_t address) {
    static uint64_t rxBuffer[1] = {0};

    // Write status flag 64-bits 0
    uint32_t status = FlashAppWriteData(MEASUREMENT_STATUS_ADDRESS(address), rxBuffer, 1);

    APP_LOG(TS_OFF, VLEVEL_M, "Measurement has been sent: %d | %d\r\n", FlashAppHasMeasurementBeenSent(address), status);
}

/**
  * @brief  Reads valid unsent measurements and gets them ready to be sent.
  */
void FlashAppReadUnsentMeasurements(void) {
    uint32_t address;
    static uint64_t rxBuffer[4];
    static uint8_t measurement[MEASUREMENT_SIZE_BYTES];
    uint8_t measurementPointer = 0;

    // Go through the entire measurement flash storage section
    for (uint16_t page = FLASH_START_PAGE_MEASUREMENTS; page <= FLASH_END_PAGE_MEASUREMENTS; page++) {
        address = FLASH_PAGE_START_ADDRESS(page);

        for (uint32_t endAddress = address + BYTES_USED_PER_PAGE; address < endAddress; address += (MEASUREMENT_SIZE_BYTES + MEASUREMENT_STATUS_SIZE_BYTES)) {
            // Check if current double word length memory content is empty (64 bits set to 1)
            if ((~(*(__IO uint64_t*) address)) == 0) {
                continue;
            }

            // Check if the measurement has already been sent and if it's already been added into RAM
            if (!FlashAppHasMeasurementBeenSent(address) && !SensorAppIsMeasurementInRAM(address)) {
                // Get the measurement value
                FlashAppReadData(address, rxBuffer, MEASUREMENT_SIZE_BYTES / 8);

                APP_LOG(TS_OFF, VLEVEL_M, "Read unsent measurement at: %d\r\n", address);

                measurementPointer = 0;

                // Convert 64 bit values into an array of 8 bit values, little endian (LSB is first)
                for (uint8_t i = 0; i < 3; i++) {
                    for (uint8_t j = 0; j < 8; j++) {
                        measurement[measurementPointer++] = (uint8_t) (rxBuffer[i] >> (j * 8));
                    }
                }

                // Add measurement to RAM
                if (!SensorAppAddMeasurementToStorage(measurement, address)) {
                    // If it fails, do not continue again, as buffer is most likely full
                    return;
                }
            }
        }
    }
}

/**
  * @brief  Writes given data into flash.
  * @param  pointer to data to be written (array of uint8_t)
  */
uint32_t FlashAppWriteMeasurement(uint8_t *data) {
    static uint32_t measurementFlashAddress = FLASH_START_ADDRESS_MEASUREMENTS;

    // Get a free spot in flash
    uint32_t status = FlashAppFindFreeSpotMeasurement(measurementFlashAddress);

    switch (status) {
        // If there is not enough space anymore, clear a page
        case FLASH_MEMORY_STATUS_CLEAR_MEMORY: {
            bool pageErased = false;

            // Erase pages if possible
            for (uint16_t page = FLASH_START_PAGE_MEASUREMENTS; page <= FLASH_END_PAGE_MEASUREMENTS; page++) {
                if (FlashAppIsPageReadyToBeErased(page)) {
                    FlashAppErasePage(page);

                    // Reset measurement pointer to a valid pointer
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
        // If successful, continue with returned address
        default: {
            measurementFlashAddress = status;

            break;
        }
    }

    uint32_t startAddress = measurementFlashAddress;
    uint64_t dataToWrite;
    uint8_t errors = 0;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    // Write 8 Bytes at a time
    for (uint16_t i = 0; i < MEASUREMENT_SIZE_BYTES; i += 8) {
        dataToWrite = 0;

        // Group 8 bytes into one 64 bit value
        for (uint8_t j = 0; j < 8; j++) {
            dataToWrite += ((uint64_t) *(data + i + j)) << (j * 8);
        }

        // Try writing data 3 times at most
        while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, measurementFlashAddress, dataToWrite) != HAL_OK) {
            // If 3 errors have occurred whilst trying to write to flash, abort
            if (++errors == 3) {
                return HAL_FLASH_GetError();
            }
        }

        errors = 0;
        measurementFlashAddress += 8;
    }

    // Leave space for status (64-bit number)
    measurementFlashAddress += MEASUREMENT_STATUS_SIZE_BYTES;

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();

    APP_LOG(TS_OFF, VLEVEL_M, "Measurement address: %d - %d\r\n", startAddress, measurementFlashAddress);

    return startAddress;
}

/* Private Functions Definition -----------------------------------------------*/

/**
  * @brief  Finds the current config.
  * @param  address from which to start looking for config
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

/**
  * @brief  Finds a free spot for writing config and returns address of free spot.
  * @param  address from which to start looking for free spot
  * @retval address
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

/**
  * @brief  Returns if a measurement in flash has been marked as sent.
  * @param  address at which the measurement resides
  */
static bool FlashAppHasMeasurementBeenSent(const uint32_t address) {
    // Measurement has been marked as sent if status flags have been set to 0
    return ((*(__IO uint64_t*) MEASUREMENT_STATUS_ADDRESS(address)) == 0);
}

/**
  * @brief  Finds a spot within the flash to write an entire measurement and returns its address.
  * @param  address from which to start looking for free spot
  * @retval address
  */
static uint32_t FlashAppFindFreeSpotMeasurement(uint32_t address) {
    uint16_t page = (address - FLASH_BASE) / FLASH_PAGE_SIZE;
    uint32_t startAddress = address;
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

        if (overflow && (address == startAddress)) {
            return FLASH_MEMORY_STATUS_CLEAR_MEMORY;
        }

        address += (MEASUREMENT_SIZE_BYTES + MEASUREMENT_STATUS_SIZE_BYTES);
    }
}

/**
  * @brief  Write data of size into flash, starting at address.
  * @param  address from which to start writing data
  * @param  pointer to data to write
  * @param  size of data to write
  * @retval status
  */
static uint32_t FlashAppWriteData(uint32_t address, uint64_t *data, uint16_t size) {
    uint8_t errors = 0;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    // Write 8 Bytes at a time
    while (true) {
        // Try writing data 3 times at most
        while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, *data) != HAL_OK) {
            if (++errors == 3) {
                // If 3 errors have occurred whilst trying to write to flash, abort
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

/**
  * @brief  Fills a given buffer by reading at a given address.
  * @param  address from which to start reading data
  * @param  pointer to buffer to write into
  * @param  size of data to read
  * @retval status
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

/**
  * @brief  Erases a given flash page.
  * @param  page number to erase
  * @retval status
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

/**
  * @brief  Checks page if all data is ready to be erased.
  *         Not all bytes are used in a page (see BYTES_USED_PER_PAGE).
  * @param  page number to check
  * @retval bool - if page contains no unsent measurements
  */
static bool FlashAppIsPageReadyToBeErased(const uint16_t page) {
    uint32_t address = FLASH_PAGE_START_ADDRESS(page);

    for (uint32_t endAddress = address + BYTES_USED_PER_PAGE; address < endAddress; address += (MEASUREMENT_SIZE_BYTES + MEASUREMENT_STATUS_SIZE_BYTES)) {
        if (!FlashAppHasMeasurementBeenSent(address)) {
            return false;
        }
    }

    return true;
}
