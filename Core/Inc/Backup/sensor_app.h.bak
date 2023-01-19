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

#include "timer_if.h"
#include "rtc.h"

#include "stm32_timer.h"
#include "stm32_lpm_if.h"

#include "app_lorawan.h"
#include "LmHandler.h"
#include "LoRaMacInterfaces.h"

/* Exported macros -----------------------------------------------------------*/
/* Definitions ---------------------------------------------------------------*/
// Messages
#define IRDA_MESSAGE_WAKE_UP                    1
#define IRDA_MESSAGE_START                      2
#define IRDA_MESSAGE_END                        3

#define MINUTE                                  60000U

#define BATTERY_REGISTER_NUMBER                 1
#define BATTERY_COMMA_POSITION                  3

// LoRa received message encoding
#define LORA_TIMER_INTERVAL_MINS                1
#define LORA_SEND_MEASUREMENTS_AFTER            2

#define TIMER_INTERVAL_MINUTES_MINIMUM          1
#define TIMER_INTERVAL_MINUTES_MAXIMUM          0xFFFFU

#define SEND_MEASUREMENTS_AFTER_MINIMUM         1
#define SEND_MEASUREMENTS_AFTER_MAXIMUM         (RAM_STORAGE_SIZE / 2)

// (CONFIG_SIZE * CONFIG_TYPE_SIZE_BYTES) % 8 = 0
#define CONFIG_SIZE                             2
#define CONFIG_TYPE                             uint32_t
#define CONFIG_TYPE_SIZE_BYTES                  sizeof(CONFIG_TYPE)
#define CONFIG_TYPE_SIZE_BITS                   (CONFIG_TYPE_SIZE_BYTES * 8)

/* Parameters ----------------------------------------------------------------*/
// If you change MEASUREMENT_SIZE_BYTES, reset flash memory
#define MEASUREMENT_SIZE_BYTES                  32 // In bytes -> 4x 64 bits
#define RAM_STORAGE_SIZE                        (8 * 24U) // Min: 1, Max: 512

#define SEND_UNCONFIRMED_MESSAGES_INTERVAL      5000U
#define LORA_STOP_AFTER_FAILURES                5

#define LORA_MESSAGE_CONFIRMATION_UNKNOWN       0
#define LORA_MESSAGE_UNCONFIRMED                1
#define LORA_MESSAGE_CONFIRMED                  2

/* Exported functions prototypes ---------------------------------------------*/
void SensorAppInit(void);

void SensorAppReadMeasurementsInit(void);
void SensorAppReadMeasurements(void);
bool SensorAppIsMeasurementInRAM(const uint32_t address);
bool SensorAppAddMeasurementToStorage(const uint8_t *measurement, uint32_t flashAddress);

void SensorAppLoRaSend(void);
bool SensorAppLoRaJoin(void);
bool SensorAppLoRaDisconnect(void);

void SensorAppWriteConfig(void);

#endif /* INC_SENSOR_APP_H_ */
