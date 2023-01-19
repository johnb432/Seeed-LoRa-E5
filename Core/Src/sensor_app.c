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

static void SensorAppExtractMeasurementTime(const char *measurement);
static void SensorAppExtractMeasurementValue(const char *measurement);

static bool SensorAppIRDAReceiveByte(void);
static bool SensorAppIRDAChecksum(const char *message);

static uint16_t SensorAppFindNextMeasurementToSend(uint16_t index);
static uint16_t SensorAppFindNextFreeSpot(uint16_t index);
static void SensorAppRemoveMeasurementFromStorage(uint16_t index);

static void StartMeasurementsTimerCallback(void *context);
static void WakeSensorUpTimerCallback(void *context);
static void SendDataTimerCallback(void *context);
static void DisconnectLoRaTimerCallback(void *context);
static void PauseLoRaTimerCallback(void *context);
static void ResumeLoRaTimerCallback(void *context);

/* Exported functions --------------------------------------------------------*/
static IRDA_HandleTypeDef *IRDA_uartInstance;

// Config parameters (stored in flash memory)
uint32_t IRDA_intervalBetweenMeasurements                   = 1;
uint16_t LoRa_sendMeasurementsAfter                         = 1;

bool IRDA_intervalBetweenMeasurementsUpdated                = false;
bool LoRa_sendMeasurementsAfterUpdated                      = false;

static uint8_t measurementExtracted[MEASUREMENT_SIZE_BYTES] = {0};
static uint16_t measurementExtractedPointer                 = 0;

// Measurement storage
static uint8_t *measurementRAMStorage[RAM_STORAGE_SIZE]     = {NULL};   // Stores pointers to measurements in RAM
static uint32_t measurementFlashStorage[RAM_STORAGE_SIZE]   = {0};      // Stores the flash addresses of the measurements that are in RAM
static uint16_t numberOfMeasurementsInStorage               = 0;        // Keeps track of how many measurements are in RAM storage

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

static uint32_t LoRa_sendUnconfirmedMessageAfterCounter     = 0;
static uint8_t LoRa_messageConfirmationFailures             = 0;

static uint8_t message                                      = 0;

static char rxBuffer[256] = { '\0' };
static uint8_t rxPointer = 0;

// Timer declarations
UTIL_TIMER_Object_t SendDataTimer;
static UTIL_TIMER_Object_t MeasurementTimer;
static UTIL_TIMER_Object_t WakeSensorUpTimer;
static UTIL_TIMER_Object_t DisconnectLoRaTimer;
static UTIL_TIMER_Object_t PauseLoRaTimer;
static UTIL_TIMER_Object_t ResumeLoRaTimer;

/**
  * @brief  Reads config parameters and sets affected values.
  */
void SensorAppReadConfig(void) {
    static CONFIG_TYPE configData[CONFIG_SIZE];

    // Get config data
    FlashAppReadConfig(configData);

    APP_LOG(TS_OFF, VLEVEL_M, "IRDA_intervalBetweenMeasurements from config: %d\r\n", configData[0]);
    APP_LOG(TS_OFF, VLEVEL_M, "LoRa_sendMeasurementsAfter from config: %d\r\n", configData[1]);

    // Make sure config is valid
    if ((configData[0] >= TIMER_INTERVAL_MINUTES_MINIMUM) && (configData[0] <= TIMER_INTERVAL_MINUTES_MAXIMUM)) {
        IRDA_intervalBetweenMeasurements = configData[0];
    }

    if ((configData[1] >= SEND_MEASUREMENTS_AFTER_MINIMUM) && (configData[1] <= SEND_MEASUREMENTS_AFTER_MAXIMUM)) {
        LoRa_sendMeasurementsAfter = configData[1];
    }

    APP_LOG(TS_OFF, VLEVEL_M, "IRDA_intervalBetweenMeasurements: %d\r\n", IRDA_intervalBetweenMeasurements);
    APP_LOG(TS_OFF, VLEVEL_M, "LoRa_sendMeasurementsAfter: %d\r\n", LoRa_sendMeasurementsAfter);
}

/**
  * @brief  Checks if the config values have been updated.
  *         If there are new values, it writes them to flash.
  */
void SensorAppWriteConfig(void) {
    if (IRDA_intervalBetweenMeasurementsUpdated || LoRa_sendMeasurementsAfterUpdated) {
        // Set new timer value
        if (IRDA_intervalBetweenMeasurementsUpdated) {
            uint32_t elapsedTime;

            // Get remaining time left on current timer
            UTIL_TIMER_GetRemainingTime(&MeasurementTimer, &elapsedTime);

            // Restart timer, with new period
            // This means that the next measurement will still be taken with the old interval
            UTIL_TIMER_StartWithPeriod(&MeasurementTimer, IRDA_intervalBetweenMeasurements * MINUTE);

            APP_LOG(TS_OFF, VLEVEL_M, "Updated timer: %d\r\n", elapsedTime);

            // Load old timer value
            MeasurementTimer.Timestamp = elapsedTime;

            APP_LOG(TS_OFF, VLEVEL_M, "Updated IRDA_intervalBetweenMeasurements: %d\r\n", IRDA_intervalBetweenMeasurements);
        }

        // Set new send after number of measurements
        if (LoRa_sendMeasurementsAfterUpdated) {
            APP_LOG(TS_OFF, VLEVEL_M, "Updated LoRa_sendMeasurementsAfter: %d\r\n", LoRa_sendMeasurementsAfter);
        }

        IRDA_intervalBetweenMeasurementsUpdated = false;
        LoRa_sendMeasurementsAfterUpdated = false;

        CONFIG_TYPE configData[CONFIG_SIZE] = { IRDA_intervalBetweenMeasurements, LoRa_sendMeasurementsAfter };

        // Write to flash memory
        FlashAppWriteConfig(configData);
    }
}

/**
  * @brief  Reads config parameters from flash memory.
  *         Finds unsent measurements in flash memory and gets them ready to send.
  *         Initialises necessary timers.
  */
void SensorAppInit(void) {
    // Init timers and other
    SystemApp_Init();

    APP_LOG(TS_OFF, VLEVEL_M, "\r\nProgram start\r\n");

    // Read config
    SensorAppReadConfig();

    // Assign chosen IRDA instance
    IRDA_uartInstance = &hirda1;

    // Create timers
    UTIL_TIMER_Create(&MeasurementTimer, IRDA_intervalBetweenMeasurements * MINUTE, UTIL_TIMER_PERIODIC, StartMeasurementsTimerCallback, NULL);
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

/**
  * @brief  Initialises the necessary to take measurements via IRDA.
  */
void SensorAppReadMeasurementsInit(void) {
    /*
    // For reading the current RTC values
    static RTC_TimeTypeDef ttd;
    static RTC_DateTypeDef dtd;

    HAL_RTC_GetTime(&hrtc, &ttd, RTC_FORMAT_BCD);
    HAL_RTC_GetDate(&hrtc, &dtd, RTC_FORMAT_BCD);

    APP_LOG(TS_OFF, VLEVEL_M, "Date: %d | %d | %d\r\n", dtd.Date, dtd.Month, dtd.Year);
    APP_LOG(TS_OFF, VLEVEL_M, "Time: %d | %d | %d\r\n", ttd.Hours, ttd.Minutes, ttd.Seconds);
    */

    // Stop pause and resume LoRa timers if they were running
    if (LoRa_pauseTimerRunning) {
        LoRa_pauseTimerRunning = false;
        UTIL_TIMER_Stop(&PauseLoRaTimer);
    }

    if (LoRa_resumeTimerRunning) {
        LoRa_resumeTimerRunning = false;
        UTIL_TIMER_Stop(&ResumeLoRaTimer);
    }

    // Reinit the IRDA module after sleep
    MX_USART1_IRDA_Init();

    // Flush the RX and TX buffers
    __HAL_IRDA_FLUSH_DRREGISTER(&(*IRDA_uartInstance));

    IRDA_startMeasurements = false;
    APP_LOG(TS_OFF, VLEVEL_M, "Starting measurements\r\n");

    // Start timer to wake up sensor
    UTIL_TIMER_Start(&WakeSensorUpTimer);

    IRDA_readingMeasurements = true;
}

/**
  * @brief  Handles the logic flow behind getting the measurements from the sensor.
  */
void SensorAppReadMeasurements(void) {
    // Flag is toggled every 200 ms
    if (IRDA_wakeUpCallbackFlag) {
        IRDA_wakeUpCallbackFlag = false;

        // Send the wake up message 5 consecutive times maximum
        if (IRDA_wakeUpCounter < 5) {
            uint8_t messageSend[] = "A\r";

            // Make sure to transmit the wake up message
            while (HAL_OK != HAL_IRDA_Transmit(&(*IRDA_uartInstance), messageSend, sizeof(messageSend) - 1, 10)) {}
        } else if (IRDA_wakeUpCounter >= (150 + 5)) {
            // If it has failed, wait for 30 s (150 * 200 ms), then try to wake up sensor again
            IRDA_wakeUpCounter = 0;
        }
    }

    // Check if there is data available
    if (SensorAppIRDAReceiveByte() && (rxPointer != 0)) {
        // Get received content in string form, so that it can be compared
        char receivedMessage[rxPointer + 1];

        strncpy(receivedMessage, rxBuffer, rxPointer);

        // String must end with a NULL char
        receivedMessage[rxPointer] = '\0';

        // If message contains checksum, check if checksum is good
        if ((strstr(receivedMessage, "K23") != NULL) && !SensorAppIRDAChecksum(receivedMessage)) {
            uint8_t messageSend[] = "?06\r";

            // If checksum failed, ask to resend last transmission
            while (HAL_OK != HAL_IRDA_Transmit(&(*IRDA_uartInstance), messageSend, sizeof(messageSend) - 1, 10)) {}

            rxPointer = 0;

            return;
        }

        // Once woken up, sensor will ask "what function" with "08?\r"
        if (!strcmp(receivedMessage, "?08\r")) {
            // Stop timer
            UTIL_TIMER_Stop(&WakeSensorUpTimer);

            uint8_t messageSend[] = "S\r";

            // Open send request
            HAL_IRDA_Transmit(&(*IRDA_uartInstance), messageSend, sizeof(messageSend) - 1, 50);

            message = IRDA_MESSAGE_START;

            IRDA_wakeUpCounter = 0;
        } else if (!strcmp(receivedMessage, "*\r")) {
            switch (message) {
                // Sensor has acknowledged that there is a send request ("S")
                case IRDA_MESSAGE_START: {
                    // Send first register value request
                    uint8_t messageSend[] = "F0017G0010\r";
                    HAL_IRDA_Transmit(&(*IRDA_uartInstance), messageSend, sizeof(messageSend) - 1, 50);

                    break;
                }
                // Sensors has acknowledged the end of transmission ("A", Abbruch)
                case IRDA_MESSAGE_END: {
                    IRDA_readingMeasurements = false;

                    // Put measurement into storage (both RAM and flash)
                    SensorAppAddMeasurementToStorage(measurementExtracted, 0);

                    APP_LOG(TS_OFF, VLEVEL_M, "Finishing measurements | Number of measurements taken: %d\r\n", numberOfMeasurementsInStorage);

                    // Check if we should send the measurements over LoRa or not
                    if (numberOfMeasurementsInStorage >= LoRa_sendMeasurementsAfter) {
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
        } else if (!strncmp(receivedMessage, "K85 00170010", 12)) { // Received ground water level
            // Store time from first measurement
            SensorAppExtractMeasurementTime(receivedMessage);
            SensorAppExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "F0017G0020\r";

            // Request next register
            HAL_IRDA_Transmit(&(*IRDA_uartInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else if (!strncmp(receivedMessage, "K85 00170020", 12)) { // Received ground water temperature
            SensorAppExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "F0017G0030\r";

            // Request next register
            HAL_IRDA_Transmit(&(*IRDA_uartInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else if (!strncmp(receivedMessage, "K85 00170030", 12)) { // Received ground water electrical conductivity
            SensorAppExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "F0017G0035\r";

            // Request next register
            HAL_IRDA_Transmit(&(*IRDA_uartInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else if (!strncmp(receivedMessage, "K85 00170035", 12)) { // Received ground water salinity
            SensorAppExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "F0017G0036\r";

            // Request next register
            HAL_IRDA_Transmit(&(*IRDA_uartInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else if (!strncmp(receivedMessage, "K85 00170036", 12)) { // Received ground water TDS
            message = IRDA_MESSAGE_END;

            SensorAppExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "F0017G0090\r";

            // Request last register
            HAL_IRDA_Transmit(&(*IRDA_uartInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else if (!strncmp(receivedMessage, "K85 00170090", 12)) { // Received ground water sensor battery voltage
            SensorAppExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "A\r";

            // Tell sensor that we are done
            HAL_IRDA_Transmit(&(*IRDA_uartInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else {
            // Not awaited reception handling goes here
            if (rxPointer != 0) {
                // TODO: What is to be done?
            }
        }

        rxPointer = 0;
    }
}

/**
  * @brief  Readies the uC to join the LoRa network.
  */
static void SensorAppLoRaJoinInit(void) {
    // Read unsent measurements from storage and get them ready to be sent
    FlashAppReadUnsentMeasurements();

    APP_LOG(TS_OFF, VLEVEL_M, "Sending measurements over LoRa\r\n\r\n");

    // Reset LoRa tracking values
    LoRa_messageConfirmationFailures = 0;
    LoRa_pause = false;
    LoRa_sendData = true;
}

/**
  * @brief  Tries to join the LoRa network if it hasn't joined.
  * @retval bool - returns if connected to LoRa network
  */
bool SensorAppLoRaJoin(void) {
    // Pause LoRa connection
    if (LoRa_pause) {
        // Disconnected
        if (LmHandlerJoinStatus() == LORAMAC_HANDLER_RESET) {
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
        } else { // Connected
            // If still connected, disconnect
            if (SensorAppLoRaDisconnect()) {
                APP_LOG(TS_OFF, VLEVEL_M, "Pausing LoRa connection\r\n");
            }
        }
    } else { // Resume
        // Disconnected
        if (LmHandlerJoinStatus() == LORAMAC_HANDLER_RESET) {
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
        } else { // Connected
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

    // Return if connected or not
    return LmHandlerJoinStatus() == LORAMAC_HANDLER_SET;
}

/**
  * @brief Handles the sending of data once connected to the LoRa network.
  */
void SensorAppLoRaSend(void) {
    static uint16_t sensorAppLoRaSendPointer = 0;

    // Start the timer for sending LoRa messages
    if (!LoRa_sendDataTimerRunnning) {
        LoRa_sendDataTimerRunnning = true;
        UTIL_TIMER_Start(&SendDataTimer);
    }

    // With this flag, we try to send data every 5 s
    if (LoRa_sendDataCallbackFlag) {
        LoRa_sendDataCallbackFlag = false;

        sensorAppLoRaSendPointer = SensorAppFindNextMeasurementToSend(sensorAppLoRaSendPointer);

        // If nothing was found (all measurements have been sent), we are going to disconnect afterwards
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
            // Try to send new frame after SEND_UNCONFIRMED_MESSAGES_INTERVAL ms
            if ((LoRa_sendUnconfirmedMessageAfterCounter >= SEND_UNCONFIRMED_MESSAGES_INTERVAL) && SendTxData(measurementRAMStorage[sensorAppLoRaSendPointer])) {
                // If data was sent successfully, remove measurement from storage
                SensorAppRemoveMeasurementFromStorage(sensorAppLoRaSendPointer);
            }
        } else { // If confirmed message
            // If data was sent successfully, await for acknowledgment/confirmation
            if (!LoRa_messageSent && SendTxData(measurementRAMStorage[sensorAppLoRaSendPointer])) {
                LoRa_messageSent = true;
            }

            // Wait until we know if the message has been acknowledged or not
            if (LoRa_messageSent && (LoRa_messageAcknowledged > LORA_MESSAGE_CONFIRMATION_UNKNOWN)) {
                // If the message has been confirmed
                if (LoRa_messageAcknowledged == LORA_MESSAGE_CONFIRMED) {
                    // If sent successfully and confirmed reception, delete from storage
                    SensorAppRemoveMeasurementFromStorage(sensorAppLoRaSendPointer);

                    LoRa_messageSent = false;
                    LoRa_messageConfirmationFailures = 0;

                    APP_LOG(TS_OFF, VLEVEL_M, "Message has been confirmed\r\n");
                } else { // If the message has failed to be confirmed
                    // If the message has been failed to be confirmed LORA_STOP_AFTER_FAILURES times
                    if (++LoRa_messageConfirmationFailures == LORA_STOP_AFTER_FAILURES) {
                        UTIL_TIMER_Stop(&SendDataTimer);
                        LoRa_sendDataTimerRunnning = false;

                        LoRa_messageConfirmationFailures = 0;

                        LoRa_sendData = false;
                        LoRa_dataSent = true;

                        APP_LOG(TS_OFF, VLEVEL_M, "Message has not been confirmed, stopping\r\n");

                        return;
                    } else { // Try to send the same message again, until we reach LORA_STOP_AFTER_FAILURES tries or we successfully send frame with confirmed reception
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

/**
  * @brief  Disconnects the client from the LoRa network.
  * @retval bool - returns if disconnected from LoRa network
  */
bool SensorAppLoRaDisconnect(void) {
    LmHandlerFlagStatus_t status = LmHandlerJoinStatus();

    // Run timer if it hasn't been activated and if client is still connected
    if ((status == LORAMAC_HANDLER_SET) && !LoRa_disconnectTimerRunning) {
        LoRa_disconnectTimerRunning = true;
        UTIL_TIMER_Start(&DisconnectLoRaTimer);
    }

    // If timer has been triggered (every 1 s)
    if (LoRa_disconnectCallbackFlag) {
        LoRa_disconnectCallbackFlag = false;

        // If client is connected to network, try to disconnect
        if ((status == LORAMAC_HANDLER_RESET) || (LmHandlerDeInit() == LORAMAC_HANDLER_SUCCESS)) {
            // Successful disconnection
            APP_LOG(TS_OFF, VLEVEL_M, "Stopped LoRa connection\r\n");

            // Turn off timers if they are still running
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

            // Make sure these variables are reset off
            LoRa_sendData = false;
            LoRa_messageSent = false;
            LoRa_messageAcknowledged = false;

            LoRa_isInitialised = false;

            return true;
        }
    }

    MX_LoRaWAN_Process();

    return false;
}

/**
  * @brief  Returns if a measuremen's address in flash memory has been put into RAM.
  * @param  address
  * @retval bool
  */
bool SensorAppIsMeasurementInRAM(const uint32_t address) {
    // Check entire storage to see if the address is stored within
    for (uint16_t i = 0; i < RAM_STORAGE_SIZE; i++) {
        if (measurementFlashStorage[i] == address) {
            return true;
        }
    }

    return false;
}

/**
  * @brief  Adds a measurement into RAM and flash, if specified. Returns if successful or not.
  * @param  pointer to data
  * @param  address to measurement in flash - if 0, it means it doesn't exist in flash yet, so get a new address
  * @retval bool - if operation was completed successfully
  */
bool SensorAppAddMeasurementToStorage(const uint8_t *measurement, uint32_t flashAddress) {
    static uint16_t measurementStoragePointer = 0;

    // If the measurement should only be added to RAM and half or more of RAM storage is used, don't write to it
    if ((flashAddress > 0) && (numberOfMeasurementsInStorage > (RAM_STORAGE_SIZE / 2))) {
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

    // Put pointer into storage
    measurementRAMStorage[measurementStoragePointer] = storedMeasurement;

    // Check if we should write in flash memory; If address is 0, it means it hasn't been written to flash
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

    // Increase the counter for the amount of measurements that are in storage
    numberOfMeasurementsInStorage++;

    return true;
}

/* Private Functions Definition -----------------------------------------------*/

/**
  * @brief  Removes measurements from both RAM and flash storage.
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

/**
  * @brief  Extracts date and time from first measurement message.
  *         It converts the content to binary for better data compression.
  * @param  pointer to data, which must be a string
  */
static void SensorAppExtractMeasurementTime(const char *measurement) {
    // Reset pointer
    measurementExtractedPointer = 0;

    uint64_t dateAndTime = 0;
    uint64_t power = 100000000000U;

    // // K28 is for date (date in format ddmmyy)
    char *ptrBuffer = strstr(measurement, "K28");
    for (uint8_t i = 4; i <= 11; i++) {
        if (*(ptrBuffer + i) != ' ') {
            dateAndTime += (*(ptrBuffer + i) - 0x30U) * power;
            power /= 10;
        }
    }

    // K24 is for time (time in format hhmmss)
    ptrBuffer = strstr(measurement, "K24");
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

/**
  * @brief  Extracts measurement values from measurement message.
  *         It converts the content to binary for better data compression.
  * @param  pointer to data, which must be a string
  */
static void SensorAppExtractMeasurementValue(const char *measurement) {
    uint16_t registerNumber = 0; // 12 bits
    uint16_t power = 1000;

    // K06 is for command
    char *ptrBuffer = strstr(measurement, "K06");
    for (uint8_t i = 8; i <= 11; i++) {
        if (*(ptrBuffer + i) != ' ') {
            registerNumber += (*(ptrBuffer + i) - 0x30) * power;
            power /= 10;
        }
    }

    // K20 is for data
    ptrBuffer = strstr(measurement, "K20");

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
    uint32_t measurementCompressed = value + (measurementHeader << 16); // 32 bits used

    // LSB first in buffer
    for (uint8_t i = 0; i < 4; i++) {
        measurementExtracted[measurementExtractedPointer++] = (uint8_t) (measurementCompressed >> (i * 8));
    }
}

/**
  * @brief  Polls for IRDA traffic.
  *         Puts received data into a buffer (rxBuffer) so that it can be analysed elsewhere.
  * @retval bool - if operation has timed out (which means nothing was received during 50 ms)
  */
static bool SensorAppIRDAReceiveByte(void) {
    static uint8_t reception[1] = { '\0' };

    // Receive one byte at a time; Wait for 50 ms to see if anything else has been sent
    switch (HAL_IRDA_Receive(&(*IRDA_uartInstance), reception, sizeof(reception), 50)) {
        // If it times out, reception is complete (even if nothing has been received)
        case HAL_TIMEOUT: {
            return true;
        }
        // If byte has been received, add to reception buffer
        case HAL_OK: {
            rxBuffer[rxPointer++] = reception[0];

            break;
        }
        default:
            break;
    }

    return false;
}

/**
  * @brief  Calculates the checksum of a given message and returns if checksums (calculated and from message) match.
  *         Message includes "\r" at end of the transmission.
  * @param  pointer to data, which must be a string
  * @retval bool
  */
static bool SensorAppIRDAChecksum(const char *message) {
    uint8_t messageLength = strlen(message);

    // If the last char isn't '\r', exit
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

    // Go over entire message, but skip over checksum characters
    for (uint8_t i = 0; i < messageLength - 10; i++) {
        checksumCalculated += message[i];
    }

    // Replace checksum within message with spaces
    checksumCalculated += 8 * 0x20U;

    // Add last character after checksum (should be either '0' or '1')
    checksumCalculated += message[messageLength - 2];

    APP_LOG(TS_OFF, VLEVEL_M, "Checksum Read: %d | Calculated: %d\r\n", checksumRead, checksumCalculated);

    return checksumRead == checksumCalculated;
}


/**
  * @brief  Finds the next measurement to send from a given index.
  *         Given index is included in search.
  * @param  index
  * @retval index, ((uint16) -1) if no measurement was found
  */
static uint16_t SensorAppFindNextMeasurementToSend(uint16_t index) {
    uint16_t startIndex = index;
    bool overflow = false;

    // Iterate over the whole storage array to see if there is a taken slot
    while (measurementRAMStorage[index] == NULL) {
        // If we have started over again and have arrived at the starting point, it means nothing was found, so quit
        if (overflow && (startIndex == index)) {
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

/**
  * @brief  Finds the next free spot for a measurement from a given index.
  *         Given index is included in search.
  * @param  index
  * @retval index, ((uint16) -1) if no space was found
  */
static uint16_t SensorAppFindNextFreeSpot(uint16_t index) {
    uint16_t startIndex = index;
    bool overflow = false;

    // Iterate over the whole storage array to see if there is a free slot
    while (measurementRAMStorage[index] != NULL) {
        if (overflow && (startIndex == index)) {
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

/**
  * @brief  Called when the timer to start a measurement has expired.
  *         Makes the whole measurement-taking process start.
  * @param  context pointer
  */
static void StartMeasurementsTimerCallback(void *context) {
    APP_LOG(TS_OFF, VLEVEL_M, "Starting measurements timer triggered\r\n");
    IRDA_startMeasurements = true;

    // When exiting this interrupt, don't go back to sleep
    HAL_PWR_DisableSleepOnExit();
}

/**
  * @brief  Called when timer to send a wake up message to the sensor has expired.
  *         Makes the uC send a wake up message over IR.
  * @param  context pointer
  */
static void WakeSensorUpTimerCallback(void *context) {
    IRDA_wakeUpCallbackFlag = true;
    IRDA_wakeUpCounter++;
}

/**
  * @brief  Called when the timer to send data has expired.
  *         Dictates in what interval the uC should try sending messages over LoRa.
  * @param  context pointer
  */
static void SendDataTimerCallback(void *context) {
    LoRa_sendDataCallbackFlag = true;
    LoRa_sendUnconfirmedMessageAfterCounter += 1000U;
}

/**
  * @brief  Called when the timer to disconnect from the LoRa network has expired.
  *         Dictates in what interval the uC should try disconnecting from the LoRa network.
  * @param  context pointer
  */
static void DisconnectLoRaTimerCallback(void *context) {
    LoRa_disconnectCallbackFlag = true;
}

/**
  * @brief  Called when the timer to pause joining the LoRa network has expired.
  *          Dictates during how the uC can try joining the LoRa network.
  * @param  context pointer
  */
static void PauseLoRaTimerCallback(void *context) {
    APP_LOG(TS_OFF, VLEVEL_M, "LoRa_pause = true\r\n");
    LoRa_pause = true;
}

/**
  * @brief  Called when the timer to resume joining (after pausing) the LoRa network has expired.
  *         Dictates after how long (after pausing) the uC can try rejoining the LoRa network.
  * @param  context pointer
  */
static void ResumeLoRaTimerCallback(void *context) {
    APP_LOG(TS_OFF, VLEVEL_M, "LoRa_pause = false\r\n");
    LoRa_pause = false;
}
