/*
 * sensor_app.c
 *
 *  Created on: Jan 3, 2023
 *      Author: PGa Reverse engineering ground water sensor
 */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>

#include "sensor_app.h"
#include "flash_app.h"

/* Private function prototypes -----------------------------------------------*/
static void SensorAppLoRaJoinInit(void);

static void SensorAppRemoveMeasurementFromStorage(uint16_t index);
static void SensorAppExtractMeasurementTime(const char *measurementRaw);
static void SensorAppExtractMeasurementValue(const char *measurementRaw);

/* Exported functions --------------------------------------------------------*/
static IRDA_HandleTypeDef *hirdaInstance;

// Stored in flash memory
uint32_t measurementIntervalMins                        = 1;
uint16_t sendMeasurementsAfterNumber                    = 1;

bool measurementIntervalMinsUpdated                     = false;
bool sendMeasurementsAfterNumberUpdated                 = false;

static uint8_t measurementExtracted[MEASUREMENT_SIZE_BYTES] = {0};
static uint16_t measurementExtractedPointer                 = 0;

// Measurement storage
static uint8_t *measurementRAMStorage[RAM_STORAGE_SIZE]     = {NULL}; // Stores pointers to measurements in RAM
static uint32_t measurementFlashStorage[RAM_STORAGE_SIZE]   = {0}; // Stores the addresses of the measurements that are in RAM
static uint16_t numberOfMeasurementsInStorage               = 0; // Keeps track of how many measurements are in RAM storage

bool IRDA_startMeasurements                                 = false;
bool IRDA_readingMeasurements                               = false;
static bool IRDA_wakeUpCallbackFlag                         = false;
static uint8_t IRDA_wakeUpCounter                           = 0;

bool LoRa_sendData                                          = false;
static bool LoRa_sendDataCallbackFlag                       = false;
bool LoRa_sendDataTimerRunnning                             = false;
bool LoRa_dataSent                                          = false;
static bool LoRa_disconnectCallbackFlag                     = true;
static bool LoRa_disconnectTimerRunning                     = false;

static bool LoRa_pause                                      = false;
static bool LoRa_pauseTimerRunning                          = false;
static bool LoRa_resumeTimerRunning                         = false;

static bool LoRa_isInitialised                              = false;

uint8_t LoRa_messageAcknowledged                            = LORA_MESSAGE_CONFIRMATION_UNKNOWN;
static bool LoRa_messageSent                                = false;

static uint32_t sendUnconfirmedMessageAfter             = 0;
static uint8_t LoRa_messageConfirmationFailures              = 0;

static uint8_t message                                  = 0;

static UTIL_TIMER_Object_t MeasurementTimer;
static UTIL_TIMER_Object_t WakeSensorUpTimer;
UTIL_TIMER_Object_t SendDataTimer;
static UTIL_TIMER_Object_t DisconnectLoRaTimer;
static UTIL_TIMER_Object_t PauseLoRaTimer;
static UTIL_TIMER_Object_t ResumeLoRaTimer;

static void StartMeasurementsTimerCallback(void *context) {
    APP_LOG(TS_OFF, VLEVEL_M, "Starting measurements timer triggered\r\n");
    IRDA_startMeasurements = true;

    // When exiting this interrupt, don't go back to sleep
    HAL_PWR_DisableSleepOnExit();
}

static void WakeSensorUpTimerCallback(void *context) {
    IRDA_wakeUpCallbackFlag = true;
    IRDA_wakeUpCounter++;
}

static void SendDataTimerCallback(void *context) {
    LoRa_sendDataCallbackFlag = true;
    sendUnconfirmedMessageAfter += 1000U;
}

static void DisconnectLoRaTimerCallback(void *context) {
    LoRa_disconnectCallbackFlag = true;
}

static void PauseLoRaTimerCallback(void *context) {
    APP_LOG(TS_OFF, VLEVEL_M, "LoRa_pause = true\r\n");
    LoRa_pause = true;
}

static void ResumeLoRaTimerCallback(void *context) {
    APP_LOG(TS_OFF, VLEVEL_M, "LoRa_pause = false\r\n");
    LoRa_pause = false;
}

/*
 * Finds the next taken spot from a given index (given index is included in search).
 */
static uint16_t SensorAppFindNextMeasurementToSend(uint16_t index) {
    uint16_t startIndex = index;
    bool overflow = false;

    // Iterate over the whole storage array to see if there is a taken slot
    while (measurementRAMStorage[index] == NULL) {
        // If we have started over again and have arrived at the starting point, it means nothing was found, so quit
        if (overflow && startIndex == index) {
            return (uint16_t) -1;
        }

        index++;

        // If out of bounds, start over again
        if (index == RAM_STORAGE_SIZE) {
            index = 0;
            overflow = true;
        }
    }

    return index;
}

/*
 * Finds the next free spot from a given index (given index is included in search).
 */
static uint16_t SensorAppFindNextFreeSpot(uint16_t index) {
    uint16_t startIndex = index;
    bool overflow = false;

    // Iterate over the whole storage array to see if there is a free slot
    while (measurementRAMStorage[index] != NULL) {
        if (overflow && startIndex == index) {
            return (uint16_t) -1;
        }

        index++;

        // If out of bounds, start over again
        if (index == RAM_STORAGE_SIZE) {
            index = 0;
            overflow = true;
        }
    }

    return index;
}

/*
 * Returns if an address's measurement (in flash memory) has been put into RAM.
 */
bool SensorAppIsMeasurementInRAM(const uint32_t address) {
    for (uint16_t i = 0; i < RAM_STORAGE_SIZE; i++) {
        if (measurementFlashStorage[i] == address) {
            return true;
        }
    }

    return false;
}

/*
 * Adds a measurement into RAM (and flash, if specified). Returns if successful or not.
 */
bool SensorAppAddMeasurementToStorage(const uint8_t *measurement, const bool addToFlash, uint32_t flashAddress) {
    static uint16_t measurementStoragePointer = 0;

    // If the measurement should only be added to RAM and half or more of RAM storage is used, don't write to it
    if (!addToFlash && (numberOfMeasurementsInStorage > (RAM_STORAGE_SIZE / 2))) {
        return false;
    }

    bool storageIsFull = false;

    // Find a free spot to put upcoming measurements
    uint16_t index = SensorAppFindNextFreeSpot(measurementStoragePointer);

    // If there is no more free space available
    if (index == (uint16_t) -1) {
        // TODO: Implement countermeasure?
        APP_LOG(TS_OFF, VLEVEL_M, "Measurement storage is full!!!\r\n");

        storageIsFull = true;
    } else {
        // If index is out of bounds, stop
        if (index >= RAM_STORAGE_SIZE) {
            return false;
        }

        measurementStoragePointer = index;
    }

    // Allocate memory for new array
    uint8_t *storedMeasurement = (uint8_t*) calloc(MEASUREMENT_SIZE_BYTES, sizeof(uint8_t));

    // If memory can't be allocated
    if (storedMeasurement == NULL) {
        // TODO: What do we do if we can't allocate memory?
        return false;
    }

    // Copy measurement into storedMeasurement
    memcpy(storedMeasurement, measurement, MEASUREMENT_SIZE_BYTES);

    measurementRAMStorage[measurementStoragePointer] = storedMeasurement;

    // Check if we should write in flash memory
    if (addToFlash || (flashAddress > 0)) {
        if (flashAddress == 0) {
            // Write measurement into flash memory and keep address of measurement in storage
            flashAddress = FlashAppWriteMeasurement(storedMeasurement);
        }

        // Measurement was successfully written to flash
        if (flashAddress >= FLASH_START_ADDRESS_MEASUREMENTS) {
            // Add flash address, if RAM storage isn't full
            if (!storageIsFull) {
                measurementFlashStorage[measurementStoragePointer] = flashAddress;
            }
        } else {
            // TODO: What do we do if it has failed to write to flash?
            APP_LOG(TS_OFF, VLEVEL_M, "Failed to write to flash memory!!! %d\r\n", flashAddress);
        }
    }

    // Increase the counter for the amount of measurements that are in storage
    numberOfMeasurementsInStorage++;

    return true;
}

static char rxBuffer[256] = { '\0' };
static uint8_t rxPointer = 0;

/*
 * Polls for IRDA reception.
 */
static bool IRDA_Receive(void) {
    static uint8_t reception[1] = { '\0' };

    // Wait for 50 ms to see if anything else has been sent
    switch (HAL_IRDA_Receive(&(*hirdaInstance), reception, sizeof(reception), 50)) {
        // Reception is complete
        case HAL_TIMEOUT: {
            return true;
        }
        case HAL_OK: {
            rxBuffer[rxPointer++] = reception[0];

            break;
        }
        default:
            break;
    }

    return false;
}

/*
 * This function calculates the checksum of a given message.
 * Message includes "\r" at end of the transmission.
 */
static bool IRDA_checksum(const char *message) {
    uint8_t messageLength = strlen(message);

    if (message[messageLength - 1] != '\r') {
        return false;
    }

    // Get checksum from message (convert from ASCII to binary)
    // Checksum is sent in ASCII format (e.g "5288" -> 5288 in decimal)
    uint16_t checksumRead = 0;
    uint16_t power = 1000;

    for (int8_t i = -7; i <= -4; i++) {
        checksumRead += (message[messageLength + i] - 0x30U) * power;
        power /= 10;
    }

    uint16_t checksumCalculated = 0;

    // Skip over checksum characters
    for (uint8_t i = 0; i < messageLength - 10; i++) {
        checksumCalculated += message[i];
    }

    // Replace checksum within message with spaces
    checksumCalculated += 8 * 0x20U + message[messageLength - 2];

    APP_LOG(TS_OFF, VLEVEL_M, "Checksum Read: %d | Calculated: %d\r\n", checksumRead, checksumCalculated);

    return checksumRead == checksumCalculated;
}

void SensorAppWriteConfig(void) {
    // Set new timer value
    if (measurementIntervalMinsUpdated) {
        uint32_t elapsedTime;

        // Get remaining time left on current timer
        UTIL_TIMER_GetRemainingTime(&MeasurementTimer, &elapsedTime);

        // Restart timer, with new period
        UTIL_TIMER_StartWithPeriod(&MeasurementTimer, measurementIntervalMins * MINUTE);

        APP_LOG(TS_OFF, VLEVEL_M, "Updated timer: %d\r\n", elapsedTime);

        // Load old timer value
        MeasurementTimer.Timestamp = elapsedTime;

        APP_LOG(TS_OFF, VLEVEL_M, "Updated measurementIntervalMins: %d\r\n", measurementIntervalMins);
    }

    // Set new send after number of measurements
    if (sendMeasurementsAfterNumberUpdated) {
        APP_LOG(TS_OFF, VLEVEL_M, "Updated sendMeasurementsAfterNumber: %d\r\n", sendMeasurementsAfterNumber);
    }

    if (measurementIntervalMinsUpdated || sendMeasurementsAfterNumberUpdated) {
        measurementIntervalMinsUpdated = false;
        sendMeasurementsAfterNumberUpdated = false;

        CONFIG_TYPE configData[CONFIG_SIZE] = { measurementIntervalMins, sendMeasurementsAfterNumber };

        // Write to flash memory
        FlashAppWriteConfig(configData);
    }
}

/*
 * Reads config and sets affected values.
 */
void SensorAppReadConfig(void) {
    static CONFIG_TYPE configData[CONFIG_SIZE];

    // Get config data
    FlashAppReadConfig(configData);

    // Parse config data
    APP_LOG(TS_OFF, VLEVEL_M, "measurementIntervalMins from config: %d\r\n", configData[0]);
    APP_LOG(TS_OFF, VLEVEL_M, "sendMeasurementsAfterNumber from config: %d\r\n", configData[1]);

    // Make sure config is valid
    if ((configData[0] >= TIMER_INTERVAL_MINUTES_MINIMUM) && (configData[0] <= TIMER_INTERVAL_MINUTES_MAXIMUM)) {
        measurementIntervalMins = configData[0];
    }

    if ((configData[1] >= SEND_MEASUREMENTS_AFTER_MINIMUM) && (configData[1] <= SEND_MEASUREMENTS_AFTER_MAXIMUM)) {
        sendMeasurementsAfterNumber = configData[1];
    }

    APP_LOG(TS_OFF, VLEVEL_M, "measurementIntervalMins: %d\r\n", measurementIntervalMins);
    APP_LOG(TS_OFF, VLEVEL_M, "sendMeasurementsAfterNumber: %d\r\n", sendMeasurementsAfterNumber);
}

/*
 * Reads config from flash memory, reads unsent measurements from flash memory and gets them ready to send.
 * Initialises necessary timers.
 */
void SensorAppInit(void) {
    // Init timers and other
    SystemApp_Init();

    APP_LOG(TS_OFF, VLEVEL_M, "\r\nProgram start\r\n");

    /*
    if (true) {
        FlashAppMeasurementHasBeenSent(0x8022940);
        return;
    }
    */

    // Read config
    SensorAppReadConfig();

    // Assign chosen IRDA instance
    hirdaInstance = &hirda2;

    // Create timers
    UTIL_TIMER_Create(&MeasurementTimer, measurementIntervalMins * MINUTE, UTIL_TIMER_PERIODIC, StartMeasurementsTimerCallback, NULL);
    UTIL_TIMER_Create(&WakeSensorUpTimer, 200, UTIL_TIMER_PERIODIC, WakeSensorUpTimerCallback, NULL);

    UTIL_TIMER_Create(&SendDataTimer, 1000, UTIL_TIMER_PERIODIC, SendDataTimerCallback, NULL);

    UTIL_TIMER_Create(&PauseLoRaTimer, 1 * MINUTE, UTIL_TIMER_ONESHOT, PauseLoRaTimerCallback, NULL);
    UTIL_TIMER_Create(&ResumeLoRaTimer, 5 * MINUTE, UTIL_TIMER_ONESHOT, ResumeLoRaTimerCallback, NULL);

    UTIL_TIMER_Create(&DisconnectLoRaTimer, 1000, UTIL_TIMER_PERIODIC, DisconnectLoRaTimerCallback, NULL);

    // Start timer that is responsible to measurement taking
    UTIL_TIMER_Start(&MeasurementTimer);

    // When booting, take a measurement
    IRDA_startMeasurements = true;
}

/*
 * Initialises the necessary to take measurements via IRDA.
 */
void SensorAppReadMeasurementsInit(void) {
    static RTC_TimeTypeDef ttd;
    static RTC_DateTypeDef dtd;

    HAL_RTC_GetTime(&hrtc, &ttd, RTC_FORMAT_BCD);
    HAL_RTC_GetDate(&hrtc, &dtd, RTC_FORMAT_BCD);

    APP_LOG(TS_OFF, VLEVEL_M, "Date: %d | %d | %d\r\n", dtd.Date, dtd.Month, dtd.Year);
    APP_LOG(TS_OFF, VLEVEL_M, "Time: %d | %d | %d\r\n", ttd.Hours, ttd.Minutes, ttd.Seconds);

    // Stop timers if they were running
    if (LoRa_pauseTimerRunning) {
        LoRa_pauseTimerRunning = false;
        UTIL_TIMER_Stop(&PauseLoRaTimer);
    }

    if (LoRa_resumeTimerRunning) {
        LoRa_resumeTimerRunning = false;
        UTIL_TIMER_Stop(&ResumeLoRaTimer);
    }

    // For some reason, we need to reinit the IRDA module
    MX_USART2_IRDA_Init();

    // Just to be safe, flush the RX and TX buffers
    __HAL_IRDA_FLUSH_DRREGISTER(&(*hirdaInstance));

    IRDA_startMeasurements = false;
    APP_LOG(TS_OFF, VLEVEL_M, "Starting measurements\r\n");

    UTIL_TIMER_Start(&WakeSensorUpTimer);

    IRDA_readingMeasurements = true;
}

/*
 * Reads the measurements coming over IRDA.
 */
void SensorAppReadMeasurements(void) {
    if (IRDA_wakeUpCallbackFlag) {
        IRDA_wakeUpCallbackFlag = false;

        if (IRDA_wakeUpCounter < 5) {
            uint8_t messageSend[] = "A\r";

            while (HAL_OK != HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 10)) {}
        } else if (IRDA_wakeUpCounter > 30) {
            IRDA_wakeUpCounter = 0;
        }
    }

    // Check if there is data available
    if (IRDA_Receive() && (rxPointer != 0)) {
        // Get received content in string form, so that it can be compared
        char receivedMessage[rxPointer + 1];

        strncpy(receivedMessage, rxBuffer, rxPointer);

        // String must end with a NULL char
        receivedMessage[rxPointer] = '\0';

        // If message contains checksum
        if ((strstr(receivedMessage, "K23") != NULL) && !IRDA_checksum(receivedMessage)) {
            uint8_t messageSend[] = "?06\r";
            while (HAL_OK != HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 10)) {}

            rxPointer = 0;

            return;
        }

        // Once woken up, sensor will ask "what function" with "08?\r"
        if (!strcmp(receivedMessage, "?08\r")) {
            // Stop timer
            UTIL_TIMER_Stop(&WakeSensorUpTimer);

            // Send request
            uint8_t messageSend[] = "S\r";
            HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 50);

            message = IRDA_MESSAGE_START;

            IRDA_wakeUpCounter = 0;
        } else if (!strcmp(receivedMessage, "*\r")) {
            switch (message) {
                // Acknowledge that there is a send request ("S")
                case IRDA_MESSAGE_START: {
                    // Send first register value request
                    uint8_t messageSend[] = "F0017G0010\r";
                    HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 50);

                    break;
                }
                    // Acknowledge the end of transmission ("A", Abbruch)
                case IRDA_MESSAGE_END: {
                    IRDA_readingMeasurements = false;

                    // Put measurement into storage (both RAM and flash)
                    SensorAppAddMeasurementToStorage(measurementExtracted, true, 0);

                    APP_LOG(TS_OFF, VLEVEL_M, "Finishing measurements | Number of measurements taken: %d\r\n", numberOfMeasurementsInStorage);

                    if (numberOfMeasurementsInStorage >= sendMeasurementsAfterNumber) {
                        SensorAppLoRaJoinInit();
                    } else {
                        APP_LOG(TS_OFF, VLEVEL_M, "Entering sleep mode\r\n\r\n");

                        // Enter low power mode until next measurement is taken
                        HAL_PWR_EnableSleepOnExit();

                        PWR_EnterSleepMode();
                    }

                    break;
                }
            }
        } else if (!strncmp(receivedMessage, "K85 00170010", 12)) {
            // Register value requests
            SensorAppExtractMeasurementTime(receivedMessage);
            SensorAppExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "F0017G0020\r";
            HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else if (!strncmp(receivedMessage, "K85 00170020", 12)) {
            SensorAppExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "F0017G0030\r";
            HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else if (!strncmp(receivedMessage, "K85 00170030", 12)) {
            SensorAppExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "F0017G0035\r";
            HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else if (!strncmp(receivedMessage, "K85 00170035", 12)) {
            SensorAppExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "F0017G0036\r";
            HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else if (!strncmp(receivedMessage, "K85 00170036", 12)) {
            message = IRDA_MESSAGE_END;

            SensorAppExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "F0017G0090\r";
            HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else if (!strncmp(receivedMessage, "K85 00170090", 12)) {
            SensorAppExtractMeasurementValue(receivedMessage);

            // Last register value request
            uint8_t messageSend[] = "A\r";
            HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else {
            // Not awaited reception handling goes here
            if (rxPointer != 0) {
                // TODO: What is to be done?
            }
        }

        rxPointer = 0;
    }
}

static void SensorAppLoRaJoinInit(void) {
    // Read unsent measurements from storage and get them ready to be sent
    FlashAppReadUnsentMeasurements();

    APP_LOG(TS_OFF, VLEVEL_M, "Sending measurements over LoRa\r\n\r\n");

    LoRa_messageConfirmationFailures = 0;

    LoRa_pause = false;
    LoRa_sendData = true;
}

/*
 * Tries to join the LoRa network if it hasn't joined. Returns if connected to LoRa network.
 */
bool SensorAppLoRaJoin(void) {
    if (LoRa_pause) {
        // Pause
        if (LmHandlerJoinStatus() == LORAMAC_HANDLER_RESET) {
            // Disconnected

            // Turn off pause timer (if on)
            if (LoRa_pauseTimerRunning) {
                LoRa_pauseTimerRunning = false;
                UTIL_TIMER_Stop(&PauseLoRaTimer);
            }

            // Turn on resume timer
            if (!LoRa_resumeTimerRunning) {
                LoRa_resumeTimerRunning = true;
                UTIL_TIMER_Start(&ResumeLoRaTimer);
            }
        } else {
            // Connected

            // If still connected, disconnect
            if (SensorAppLoRaDisconnect()) {
                APP_LOG(TS_OFF, VLEVEL_M, "Pausing LoRa connection\r\n");
            }
        }
    } else {
        // Resume
        if (LmHandlerJoinStatus() == LORAMAC_HANDLER_RESET) {
            // Disconnected

            // Initialise LoRa
            if (!LoRa_isInitialised) {
                LoRa_isInitialised = true;

                MX_LoRaWAN_Init();
            }

            // Turn on pause timer
            if (!LoRa_pauseTimerRunning) {
                LoRa_pauseTimerRunning = true;
                UTIL_TIMER_Start(&PauseLoRaTimer);
            }

            // Turn off resume timer (if on)
            if (LoRa_resumeTimerRunning) {
                LoRa_resumeTimerRunning = false;
                UTIL_TIMER_Stop(&ResumeLoRaTimer);
            }

            MX_LoRaWAN_Process();
        } else {
            // Connected

            // Stop pause and resume timers
            if (LoRa_pauseTimerRunning) {
                LoRa_pauseTimerRunning = false;
                UTIL_TIMER_Stop(&PauseLoRaTimer);
            }

            if (LoRa_resumeTimerRunning) {
                LoRa_resumeTimerRunning = false;
                UTIL_TIMER_Stop(&ResumeLoRaTimer);
            }
        }
    }

    return LmHandlerJoinStatus() == LORAMAC_HANDLER_SET;
}

/*
 * Sends data once connected to LoRa.
 */
void SensorAppLoRaSend(void) {
    static uint16_t sensorAppLoRaSendPointer = 0;

    if (!LoRa_sendDataTimerRunnning) {
        LoRa_sendDataTimerRunnning = true;
        UTIL_TIMER_Start(&SendDataTimer);
    }

    // With this flag, we try to send data every 5 s
    if (LoRa_sendDataCallbackFlag) {
        LoRa_sendDataCallbackFlag = false;

        sensorAppLoRaSendPointer = SensorAppFindNextMeasurementToSend(sensorAppLoRaSendPointer);

        // If nothing was found (all measurements have been sent), get ready for disconnect
        if (sensorAppLoRaSendPointer == (uint16_t) -1) {
            UTIL_TIMER_Stop(&SendDataTimer);
            LoRa_sendDataTimerRunnning = false;

            sensorAppLoRaSendPointer = 0;
            LoRa_messageConfirmationFailures = 0;

            LoRa_sendData = false;
            LoRa_dataSent = true;

            return;
        }

        // If unconfirmed message
        if (LORAWAN_DEFAULT_CONFIRMED_MSG_STATE == LORAMAC_HANDLER_UNCONFIRMED_MSG) {
            // If data was sent successfully, remove measurement from storage
            if ((sendUnconfirmedMessageAfter >= SEND_UNCONFIRMED_MESSAGES_INTERVAL) && SendTxData(measurementRAMStorage[sensorAppLoRaSendPointer])) {
                SensorAppRemoveMeasurementFromStorage(sensorAppLoRaSendPointer);
            }
        } else {
            // If confirmed message

            // If data was sent successfully, remove measurement from storage
            if (!LoRa_messageSent && SendTxData(measurementRAMStorage[sensorAppLoRaSendPointer])) {
                LoRa_messageSent = true;
            }

            // Wait until we know if the message has been acknowledged or not
            if (LoRa_messageSent && (LoRa_messageAcknowledged > LORA_MESSAGE_CONFIRMATION_UNKNOWN)) {
                if (LoRa_messageAcknowledged == LORA_MESSAGE_CONFIRMED) {
                    SensorAppRemoveMeasurementFromStorage(sensorAppLoRaSendPointer);

                    LoRa_messageSent = false;
                    LoRa_messageConfirmationFailures = 0;

                    APP_LOG(TS_OFF, VLEVEL_M, "Message has been confirmed\r\n");
                } else {
                    if (++LoRa_messageConfirmationFailures == LORA_STOP_AFTER_FAILURES) {
                        UTIL_TIMER_Stop(&SendDataTimer);
                        LoRa_sendDataTimerRunnning = false;

                        LoRa_messageConfirmationFailures = 0;

                        LoRa_sendData = false;
                        LoRa_dataSent = true;

                        APP_LOG(TS_OFF, VLEVEL_M, "Message has not been confirmed, stopping\r\n");

                        return;
                    } else {
                        LoRa_messageSent = false;

                        APP_LOG(TS_OFF, VLEVEL_M, "Message has not been confirmed, retrying: %d\r\n", LoRa_messageConfirmationFailures);
                    }
                }

                LoRa_messageAcknowledged = LORA_MESSAGE_CONFIRMATION_UNKNOWN;
            }
        }
    } else {
        MX_LoRaWAN_Process();
    }
}

/*
 * Disconnects the client from the LoRa network. Tries every 1 s to disconnect from network
 */
bool SensorAppLoRaDisconnect(void) {
    LmHandlerFlagStatus_t status = LmHandlerJoinStatus();

    // Run timer if it hasn't been activated and if client is still connected
    if ((status == LORAMAC_HANDLER_SET) && !LoRa_disconnectTimerRunning) {
        LoRa_disconnectTimerRunning = true;
        UTIL_TIMER_Start(&DisconnectLoRaTimer);
    }

    // If timer has been triggered
    if (LoRa_disconnectCallbackFlag) {
        LoRa_disconnectCallbackFlag = false;

        // If client is connected to network, try to disconnect
        if ((status == LORAMAC_HANDLER_RESET) || (LmHandlerDeInit() == LORAMAC_HANDLER_SUCCESS)) {
            // Successful disconnection
            APP_LOG(TS_OFF, VLEVEL_M, "Stopped LoRa connection\r\n");

            if (LoRa_pauseTimerRunning) {
                LoRa_pauseTimerRunning = false;
                UTIL_TIMER_Stop(&PauseLoRaTimer);
            }

            if (LoRa_resumeTimerRunning) {
                LoRa_resumeTimerRunning = false;
                UTIL_TIMER_Stop(&ResumeLoRaTimer);
            }

            // Make sure that it's called immediately next time
            LoRa_disconnectCallbackFlag = true;

            LoRa_isInitialised = false;

            // Make sure it's off
            LoRa_sendData = false;
            LoRa_messageSent = false;
            LoRa_messageAcknowledged = false;

            return true;
        }
    }

    MX_LoRaWAN_Process();

    return false;
}

/* Private Functions Definition -----------------------------------------------*/

/*
 * Resets a taken measurement spot.
 */
static void SensorAppRemoveMeasurementFromStorage(uint16_t index) {
    // If out of bounds, do nothing
    if (index >= RAM_STORAGE_SIZE) {
        return;
    }

    // Free memory
    free(measurementRAMStorage[index]);

    // Set content to nullptr
    measurementRAMStorage[index] = NULL;

    // Notify flash storage that measurement has been sent
    FlashAppMeasurementHasBeenSent(measurementFlashStorage[index]);
    measurementFlashStorage[index] = 0;

    // Decrease the amount of measurements in storage
    numberOfMeasurementsInStorage--;
}

/*
 * Extracts date and time from first raw measurement message.
 */
static void SensorAppExtractMeasurementTime(const char *measurementRaw) {
    // Reset pointer
    measurementExtractedPointer = 0;

    uint64_t dateAndTime = 0;
    uint64_t power = 100000000000U;

    // // K28 is for date (date in format ddmmyy)
    char *ptrBuffer = strstr(measurementRaw, "K28");
    for (uint8_t i = 4; i <= 11; i++) {
        if (*(ptrBuffer + i) != ' ') {
            dateAndTime += (*(ptrBuffer + i) - 0x30U) * power;
            power /= 10;
        }
    }

    // K24 is for time (time in format hhmmss)
    ptrBuffer = strstr(measurementRaw, "K24");
    for (uint8_t i = 4; i <= 11; i++) {
        if (*(ptrBuffer + i) != ' ') {
            dateAndTime += (*(ptrBuffer + i) - 0x30U) * power;
            power /= 10;
        }
    }

    // Storage in little endian (LSB first in buffer)
    // Only 40 bits are required, but additional 8 bits are used as status flags when stored in flash memory
    // To get MEASUREMENT_SIZE_BYTES = 32, we fill up 2 additional bytes on top of that (so 64 bits total)
    for (uint8_t i = 0; i < 8; i++) {
        measurementExtracted[measurementExtractedPointer++] = (uint8_t) (dateAndTime >> (i * 8));
    }
}

/*
 * Extracts measurement values from raw measurement message and converts content binary (for better data compression).
 */
static void SensorAppExtractMeasurementValue(const char *measurementRaw) {
    uint16_t registerNumber = 0; // 12 bits
    uint16_t power = 1000;

    // K06 is for command
    char *ptrBuffer = strstr(measurementRaw, "K06");
    for (uint8_t i = 8; i <= 11; i++) {
        if (*(ptrBuffer + i) != ' ') {
            registerNumber += (*(ptrBuffer + i) - 0x30) * power;
            power /= 10;
        }
    }

    // K20 is for data
    ptrBuffer = strstr(measurementRaw, "K20");

    // + = 0, - = 1
    uint8_t sign = *(ptrBuffer + 12) == '-'; // 1 bit
    uint8_t commaPosition = *(ptrBuffer + 11) - 0x30; // 3 bits

    uint16_t value = 0; // 16 bits
    power = 10000;

    for (uint8_t i = 13; i <= 17; i++) {
        if (*(ptrBuffer + i) != ' ') {
            value += (*(ptrBuffer + i) - 0x30) * power;
            power /= 10;
        }
    }

    uint32_t measurementHeader = (registerNumber << 4) + (commaPosition << 1) + sign; // 16 bits used
    uint32_t measurement = value + (measurementHeader << 16); // 32 bits used

    // LSB first in buffer
    for (uint8_t i = 0; i < 4; i++) {
        measurementExtracted[measurementExtractedPointer++] = (uint8_t) (measurement >> (i * 8));
    }
}
