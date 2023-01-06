/*
 * flash_app.c
 *
 *  Created on: 5 Jan 2023
 *      Author: PGa Reverse engineering ground water sensor
 *      https://controllerstech.com/flash-programming-in-stm32/
 */

#include <stdlib.h>

#include "flash_app.h"
#include "sensor_app.h"

static inline uint32_t GetPage(const uint32_t Addr) {
	return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;;
}

/*
 * For STM32WLE5JC: 128 Pages
 */
uint32_t FlashAppErasePage(uint32_t startPage, uint32_t numberOfPages) {
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t errorStatus;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = startPage;
	EraseInitStruct.NbPages = numberOfPages;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &errorStatus) != HAL_OK) {
		/*Error occurred while page erase.*/
		return HAL_FLASH_GetError ();
	}

	/* Lock the Flash to disable the flash control register access (recommended
	to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	return errorStatus;
}

uint32_t FlashAppWriteData(uint32_t address, uint64_t *data, uint16_t size) {
	uint16_t i = 0;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Program the user Flash area double word by double word*/
	while (i < size) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data[i]) == HAL_OK) {
			address += 8;  // use startPageAddress += 2 for uint16_t, += 4 (half word) for uint32_t and += 8 (word) for uint64_t (double word)
			i++;
		} else {
			/* Error occurred while writing data in Flash memory*/
			return HAL_FLASH_GetError();
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	return 0;
}

void FlashAppReadData(uint32_t address, uint64_t *rxBuffer, uint16_t size) {
	while (1) {
		*rxBuffer = *(__IO uint32_t *)address;

		if ((--size) == 0) {
			return;
		}

		address += 8;
		rxBuffer++;
	}
}

uint32_t FlashAppWriteConfigData(uint32_t data) {
	if (configAddress > FLASH_END_ADDRESS_CONFIG) {
		// Reset config pointer
		configAddress = FLASH_START_ADDRESS_CONFIG;

		// Erase page
		FlashAppErasePage(FLASH_PAGE_CONFIG, 1);
	}

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Program the user Flash area word by word*/
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, configAddress, data) == HAL_OK) {
		configAddress += 8;
	} else {
		/* Error occurred while writing data in Flash memory*/
		return HAL_FLASH_GetError();
	}

	APP_LOG(TS_OFF, VLEVEL_M, "Config address: %d\r\n", configAddress);

	/* Lock the Flash to disable the flash control register access (recommended
	to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	return 0;
}

uint32_t FlashAppWriteMeasurement(uint8_t *data) {
	/*
	if (measurementFlashAddress > FLASH_END_ADDRESS_CONFIG) {
		// Reset config pointer
		measurementFlashAddress = FLASH_START_ADDRESS_CONFIG;

		// Erase page
		FlashAppErasePage(FLASH_PAGE_CONFIG, 1);
	}
	*/

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	uint64_t dataToWrite;
	uint8_t length = sizeof(data);

	/* Program the user Flash area word by word*/
	for (uint8_t i = 0; i < sizeof(data); i += 8) {
		dataToWrite = 0;

		// Group 8 bytes into one 64 bit value
		for (uint8_t j = 0; j < 8; j++) {
			// 7th byte of time and date: Set to 0xFF as flag
			if (i = 0 && j = 7) {
				dataToWrite += 0xFF << (j * 8);
			} else if (i + j < length) {
				dataToWrite += ((uint64_t) *(data + i + j)) << (j * 8);
			} else {
				dataToWrite += 0 << (j * 8);
			}
		}

		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, measurementFlashAddress, dataToWrite) == HAL_OK) {
			measurementFlashAddress += 8;
		} else {
			/* Error occurred while writing data in Flash memory*/
			return HAL_FLASH_GetError();
		}
	}

	APP_LOG(TS_OFF, VLEVEL_M, "Measurement address: %d\r\n", measurementFlashAddress);

	/* Lock the Flash to disable the flash control register access (recommended
	to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	return 0;
}
