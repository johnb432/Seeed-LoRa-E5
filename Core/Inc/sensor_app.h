/*
 * sensor_app.h
 *
 *  Created on: Jan 3, 2023
 *      Author: PGa Reverse engineering ground water sensor
 */

#ifndef INC_SENSOR_APP_H_
#define INC_SENSOR_APP_H_

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

/* Exported macros -----------------------------------------------------------*/
/* Definitions ---------------------------------------------------------------*/
// Messages
#define MESSAGE_WAKE_UP                     1
#define MESSAGE_START                       2
#define MESSAGE_END                         3

#define MINUTE                              60000U

// LoRa received message encoding
#define LORA_TIMER_INTERVAL_MINS            1
#define LORA_SEND_MEASURMENTS_AFTER         2

#define MINIMUM_TIMER_INTERVAL_MINUTES      1
#define MAXIMUM_TIMER_INTERVAL_MINUTES      70000U // 71582 * MINUTE = 0xFFFF4740 (close to max of uint32_t)

#define MINIMUM_SEND_MEASUREMENTS_AFTER     1
#define MAXIMUM_SEND_MEASUREMENTS_AFTER     (SIZE_STORAGE / 2)

#define CONFIG_SIZE                         2
#define CONFIG_TYPE_SIZE                    32

/* Parameters ----------------------------------------------------------------*/
// If you change SIZE_MEASUREMENT, reset flash memory
#define SIZE_MEASUREMENT                    24 // 3 * 64 bits = 192 bits = 24 bytes
#define SIZE_STORAGE                        (8 * 24U) // Min: 1, Max: 512

/* Exported functions prototypes ---------------------------------------------*/
void SensorAppInit(void);

void SensorAppReadMeasurementsInit(void);
void SensorAppReadMeasurements(void);
bool SensorAppIsMeasurementInRAM(const uint32_t address);
bool SensorAppAddMeasurementToStorage(const uint8_t *measurement, const bool addToFlash);

void SensorAppLoRaSend(void);
bool SensorAppLoRaJoin(void);
bool SensorAppLoRaDisconnect(void);

void SensorAppWriteConfig(void);

#endif /* INC_SENSOR_APP_H_ */
