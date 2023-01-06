/*
 * sensor_app.h
 *
 *  Created on: Jan 3, 2023
 *      Author: PGa Reverse engineering ground water sensor
 */

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "stdbool.h"

#include "sys_app.h"
#include "lora_app.h"
#include "usart.h"

#include "stm32_timer.h"
#include "stm32_lpm_if.h"

#include "app_lorawan.h"
#include "LmHandler.h"
#include "LoRaMacInterfaces.h"

#ifndef INC_SENSOR_APP_H_
#define INC_SENSOR_APP_H_

/* Exported constants --------------------------------------------------------*/
#define MESSAGE_WAKE_UP 1
#define MESSAGE_START 2
#define MESSAGE_END 3

#define FLASH_STORAGE_LENGTH 8

// Page 70 reserved for config data
#define FLASH_PAGE_CONFIG ((uint32_t) 70)
#define FLASH_START_ADDRESS_CONFIG 	(FLASH_BASE + FLASH_PAGE_CONFIG * 0x00000800u)
#define FLASH_END_ADDRESS_CONFIG 	(FLASH_START_ADDRESS_CONFIG + 0x00000800u - 1)

#define FLASH_PAGE_STORAGE_START 71
#define FLASH_PAGE_STORAGE_END 127

/*
#define FLASH_ADDRESS_TIMER_INTERVAL 0x0803F800u
#define FLASH_ADDRESS_SEND_MEASUREMENTS_AFTER (FLASH_ADDRESS_TIMER_INTERVAL + FLASH_STORAGE_LENGTH)

// Flash storage is set to 0xFFFFFFFFFFFFFFFFu when reset
#define FLASH_STORAGE_MASK 0xFFFFFFFFFFFF0000u // 64 bit value
*/

#define MINUTE 60000

#define SIZE_MEASUREMENT 32//67
#define SIZE_STORAGE 3 * 24

#define MINIMUM_TIMER_INTERVAL_MINUTES 1
#define MAXIMUM_TIMER_INTERVAL_MINUTES 10000

#define MINIMUM_SEND_MEASUREMENTS_AFTER 1
#define MAXIMUM_SEND_MEASUREMENTS_AFTER (SIZE_STORAGE - 4)

// LoRa received message encoding
#define LORA_TIMER_INTERVAL_MINS 1
#define LORA_SEND_MEASURMENTS_AFTER 2

// Function prototypes
void SensorAppInit(void);
void SensorAppReadMeasurementsInit(void);
void SensorAppReadMeasurements(void);
void SensorAppLoRaSend(void);
bool SensorAppLoRaJoin(void);
bool SensorAppLoRaDisconnect(void);
void SensorAppWriteConfig(void);

#endif /* INC_SENSOR_APP_H_ */
