/*
 * flash_app.h
 *
 *  Created on: 5 Jan 2023
 *      Author: PGa Reverse engineering ground water sensor
 *      https://controllerstech.com/flash-programming-in-stm32/
 */

#ifndef INC_FLASH_APP_H_
#define INC_FLASH_APP_H_

#include "string.h"
#include "stdbool.h"

#include "sys_app.h"

#include "stm32wlxx_hal.h"

extern uint32_t configAddress;

uint32_t FlashAppWriteData(uint32_t startPageAddress, uint64_t *data, uint16_t size);
void FlashAppReadData(uint32_t address, uint64_t *rxBuffer, uint16_t size);

uint32_t FlashAppWriteConfigData(uint32_t data);

#endif /* INC_FLASH_APP_H_ */
