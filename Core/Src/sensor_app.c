/*
 * sensor_app.c
 *
 *  Created on: Jan 3, 2023
 *      Author: PGa Reverse engineering ground water sensor
 */

#include <stdlib.h>

#include "sensor_app.h"
#include "flash_app.h"

static IRDA_HandleTypeDef *hirdaInstance;

// Stored in flash memory
uint32_t measurementIntervalMins                        = 1;
uint16_t sendMeasurementsAfterNumber                    = 1;

bool measurementIntervalMinsUpdated                     = false;
bool sendMeasurementsAfterNumberUpdated                 = false;

static uint8_t measurementExtracted[SIZE_MEASUREMENT]   = {0};
static uint16_t measurementExtractedPointer             = 0;

// Measurement storage
static uint8_t *measurementRAMStorage[SIZE_STORAGE]     = {NULL};
static uint32_t measurementFlashStorage[SIZE_STORAGE]   = {0};
static uint16_t numberOfMeasurementsInStorage           = 0;

bool startMeasurements                                  = false;
bool readingMeasurements                                = false;
static bool wakeUpCallbackFlag                          = false;
static uint8_t wakeUpCounter                            = 0;

bool sendData                                           = false;
static bool sendDataCallbackFlag                        = false;
bool sendDataTimerRunnning                              = false;
bool dataSent                                           = false;
static bool disconnectLoRaCallbackFlag                  = true;
static bool disconnectLoRaTimerRunning                  = false;

static bool pauseLoRa                                   = false;
static bool pauseLoRaTimerRunning                       = false;
static bool resumeLoRaTimerRunning                      = false;

static bool isLoRaInitialised                           = false;

static uint8_t message                                  = 0;

static UTIL_TIMER_Object_t MeasurementTimer;
static UTIL_TIMER_Object_t WakeSensorUpTimer;
UTIL_TIMER_Object_t SendDataTimer;
static UTIL_TIMER_Object_t DisconnectLoRaTimer;
static UTIL_TIMER_Object_t PauseLoRaTimer;
static UTIL_TIMER_Object_t ResumeLoRaTimer;

static void StartMeasurementsTimerCallback(void *context) {
    APP_LOG(TS_OFF, VLEVEL_M, "Starting measurements timer triggered\r\n");
    startMeasurements = true;

    // When exiting this interrupt, don't go back to sleep
    HAL_PWR_DisableSleepOnExit();
}

static void WakeSensorUpTimerCallback(void *context) {
    wakeUpCallbackFlag = true;
    wakeUpCounter++;
}

static void SendDataTimerCallback(void *context) {
    sendDataCallbackFlag = true;
}

static void DisconnectLoRaTimerCallback(void *context) {
    disconnectLoRaCallbackFlag = true;
}

static void PauseLoRaTimerCallback(void *context) {
    APP_LOG(TS_OFF, VLEVEL_M, "pauseLoRa = true\r\n");
    pauseLoRa = true;
}

static void ResumeLoRaTimerCallback(void *context) {
    APP_LOG(TS_OFF, VLEVEL_M, "pauseLoRa = false\r\n");
    pauseLoRa = false;
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
        if (index == SIZE_STORAGE) {
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
        if (index == SIZE_STORAGE) {
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
    for (uint16_t i = 0; i < SIZE_STORAGE; i++) {
        if (measurementFlashStorage[i] == address) {
            return true;
        }
    }

    return false;
}

/*
 * Adds a measurement into RAM (and flash, if specified). Returns if successful or not.
 */
bool SensorAppAddMeasurementToStorage(const uint8_t *measurement, const bool addToFlash) {
    static uint16_t measurementStoragePointer = 0;

    // If the measurement should only be added to RAM and half or more of RAM storage is used, don't write to it
    if (!addToFlash && (numberOfMeasurementsInStorage > (SIZE_STORAGE / 2))) {
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
        if (index >= SIZE_STORAGE) {
            return false;
        }

        measurementStoragePointer = index;
    }

    // Allocate memory for new array
    uint8_t *storedMeasurement = (uint8_t*) calloc(SIZE_MEASUREMENT, sizeof(uint8_t));

    // If memory can't be allocated
    if (storedMeasurement == NULL) {
        // TODO: What do we do if we can't allocate memory?
        return false;
    }

    // Copy measurement into storedMeasurement
    memcpy(storedMeasurement, measurement, SIZE_MEASUREMENT);

    measurementRAMStorage[measurementStoragePointer] = storedMeasurement;

    // Check if we should write in flash memory
    if (addToFlash) {
        // Write measurement into flash memory and keep address of measurement in storage
        uint32_t address = FlashAppWriteMeasurement(storedMeasurement);

        // Measurement was successfully written to flash
        if (address >= FLASH_START_ADDRESS_MEASUREMENTS) {
            // Add flash address, if RAM storage isn't full
            if (!storageIsFull) {
                measurementFlashStorage[measurementStoragePointer] = address;
            }
        } else {
            // TODO: What do we do if it has failed to write to flash?
            APP_LOG(TS_OFF, VLEVEL_M, "Failed to write to flash memory!!! %d\r\n", address);
        }
    }

    // Increase the counter for the amount of measurements that are in storage
    numberOfMeasurementsInStorage++;

    return true;
}

/*
 * Resets a taken measurement spot.
 */
static void SensorAppRemoveMeasurementFromStorage(uint16_t index) {
    // If out of bounds, do nothing
    if (index >= SIZE_STORAGE) {
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
static void ExtractMeasurementTime(const char *measurementRaw) {
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
    // To get SIZE_MEASUREMENT = 32, we fill up 2 additional bytes on top of that (so 64 bits total)
    for (uint8_t i = 0; i < 8; i++) {
        measurementExtracted[measurementExtractedPointer++] = (uint8_t) (dateAndTime >> (i * 8));
    }

    // Returns correct value, APP_LOG doesn't put it correctly in string
    APP_LOG(TS_OFF, VLEVEL_M, "Date and time: %d\r\n", dateAndTime);
}

/*
 * Extracts measurement values from raw measurement message and converts content binary (for better data compression).
 */
static void ExtractMeasurementValue(const char *measurementRaw) {
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
        default: {
        }
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

        uint32_t configData[2] = { measurementIntervalMins, sendMeasurementsAfterNumber };

        // Write to flash memory
        FlashAppWriteConfig(configData);
    }
}

/*
 * Reads config and sets affected values.
 */
void SensorAppReadConfig(void) {
    uint32_t configData[CONFIG_SIZE];

    // Get config data
    FlashAppReadConfig(configData);

    // Parse config data
    APP_LOG(TS_OFF, VLEVEL_M, "measurementIntervalMins from config: %d\r\n", configData[0]);
    APP_LOG(TS_OFF, VLEVEL_M, "sendMeasurementsAfterNumber from config: %d\r\n", configData[1]);

    // Make sure config is valid
    if (configData[0] >= MINIMUM_TIMER_INTERVAL_MINUTES && configData[0] <= MAXIMUM_TIMER_INTERVAL_MINUTES) {
        measurementIntervalMins = configData[0];
    }

    if (configData[1] >= MINIMUM_SEND_MEASUREMENTS_AFTER && configData[1] <= MAXIMUM_SEND_MEASUREMENTS_AFTER) {
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

    // Read config
    SensorAppReadConfig();

    // Assign chosen IRDA instance
    hirdaInstance = &hirda2;

    // Create timers
    UTIL_TIMER_Create(&MeasurementTimer, measurementIntervalMins * MINUTE, UTIL_TIMER_PERIODIC, StartMeasurementsTimerCallback, NULL);
    UTIL_TIMER_Create(&WakeSensorUpTimer, 200, UTIL_TIMER_PERIODIC, WakeSensorUpTimerCallback, NULL);

    UTIL_TIMER_Create(&SendDataTimer, 5000, UTIL_TIMER_PERIODIC, SendDataTimerCallback, NULL);

    UTIL_TIMER_Create(&PauseLoRaTimer, 1 * MINUTE, UTIL_TIMER_ONESHOT, PauseLoRaTimerCallback, NULL);
    UTIL_TIMER_Create(&ResumeLoRaTimer, 5 * MINUTE, UTIL_TIMER_ONESHOT, ResumeLoRaTimerCallback, NULL);

    UTIL_TIMER_Create(&DisconnectLoRaTimer, 1000, UTIL_TIMER_PERIODIC, DisconnectLoRaTimerCallback, NULL);

    // Start timer that is responsible to measurement taking
    UTIL_TIMER_Start(&MeasurementTimer);

    // When booting, take a measurement
    startMeasurements = true;
}

/*
 * Initialises the necessary to take measurements via IRDA.
 */
void SensorAppReadMeasurementsInit(void) {
    // Stop timers if they were running
    if (pauseLoRaTimerRunning) {
        pauseLoRaTimerRunning = false;
        UTIL_TIMER_Stop(&PauseLoRaTimer);
    }

    if (resumeLoRaTimerRunning) {
        resumeLoRaTimerRunning = false;
        UTIL_TIMER_Stop(&ResumeLoRaTimer);
    }

    // For some reason, we need to reinit the IRDA module
    MX_USART2_IRDA_Init();

    // Just to be safe, flush the RX and TX buffers
    __HAL_IRDA_FLUSH_DRREGISTER(&(*hirdaInstance));

    startMeasurements = false;
    APP_LOG(TS_OFF, VLEVEL_M, "Starting measurements\r\n");

    UTIL_TIMER_Start(&WakeSensorUpTimer);

    readingMeasurements = true;
}

/*
 * Reads the measurements coming over IRDA.
 */
void SensorAppReadMeasurements(void) {
    if (wakeUpCallbackFlag) {
        wakeUpCallbackFlag = false;

        if (wakeUpCounter < 5) {
            uint8_t messageSend[] = "A\r";

            while (HAL_OK != HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 10)) {
            }
        } else if (wakeUpCounter > 30) {
            wakeUpCounter = 0;
        }
    }

    // Check if there is data available
    if (IRDA_Receive() && rxPointer != 0) {
        // Get received content in string form, so that it can be compared
        char receivedMessage[rxPointer + 1];

        strncpy(receivedMessage, rxBuffer, rxPointer);

        // String must end with a NULL char
        receivedMessage[rxPointer] = '\0';

        // If message contains checksum
        if (strstr(receivedMessage, "K23") != NULL && !IRDA_checksum(receivedMessage)) {
            uint8_t messageSend[] = "?06\r";
            while (HAL_OK != HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 10)) {
            }

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

            message = MESSAGE_START;

            wakeUpCounter = 0;
        } else if (!strcmp(receivedMessage, "*\r")) {
            switch (message) {
                // Acknowledge that there is a send request ("S")
                case MESSAGE_START: {
                    // Send first register value request
                    uint8_t messageSend[] = "F0017G0010\r";
                    HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 50);

                    break;
                }
                    // Acknowledge the end of transmission ("A", Abbruch)
                case MESSAGE_END: {
                    readingMeasurements = false;

                    // Put measurement into storage (both RAM and flash)
                    SensorAppAddMeasurementToStorage(measurementExtracted, true);

                    APP_LOG(TS_OFF, VLEVEL_M, "Finishing measurements | Number of measurements taken: %d\r\n\r\n", numberOfMeasurementsInStorage);

                    if (numberOfMeasurementsInStorage >= sendMeasurementsAfterNumber) {
                        // Read unsent measurements from storage and get them ready to be sent
                        FlashAppReadUnsentMeasurements();

                        APP_LOG(TS_OFF, VLEVEL_M, "Sending measurements over LoRa\r\n");

                        pauseLoRa = false;

                        sendData = true;
                    } else {
                        APP_LOG(TS_OFF, VLEVEL_M, "Entering sleep mode\r\n");

                        // Enter low power mode until next measurement is taken
                        HAL_PWR_EnableSleepOnExit();

                        PWR_EnterSleepMode();
                    }

                    break;
                }
            }
        } else if (!strncmp(receivedMessage, "K85 00170010", 12)) {
            // Register value requests
            ExtractMeasurementTime(receivedMessage);
            ExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "F0017G0020\r";
            HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else if (!strncmp(receivedMessage, "K85 00170020", 12)) {
            ExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "F0017G0030\r";
            HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else if (!strncmp(receivedMessage, "K85 00170030", 12)) {
            ExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "F0017G0035\r";
            HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else if (!strncmp(receivedMessage, "K85 00170035", 12)) {
            ExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "F0017G0036\r";
            HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else if (!strncmp(receivedMessage, "K85 00170036", 12)) {
            message = MESSAGE_END;

            ExtractMeasurementValue(receivedMessage);

            uint8_t messageSend[] = "F0017G0090\r";
            HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 50);
        } else if (!strncmp(receivedMessage, "K85 00170090", 12)) {
            ExtractMeasurementValue(receivedMessage);

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

/*
 * Tries to join the LoRa network if it hasn't joined. Returns if connected to LoRa network.
 */
bool SensorAppLoRaJoin(void) {
    if (pauseLoRa) {
        // Pause
        if (LmHandlerJoinStatus() == LORAMAC_HANDLER_RESET) {
            // Disconnected

            // Turn off pause timer (if on)
            if (pauseLoRaTimerRunning) {
                pauseLoRaTimerRunning = false;
                UTIL_TIMER_Stop(&PauseLoRaTimer);
            }

            // Turn on resume timer
            if (!resumeLoRaTimerRunning) {
                resumeLoRaTimerRunning = true;
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
            if (!isLoRaInitialised) {
                isLoRaInitialised = true;

                MX_LoRaWAN_Init();
            }

            // Turn on pause timer
            if (!pauseLoRaTimerRunning) {
                pauseLoRaTimerRunning = true;
                UTIL_TIMER_Start(&PauseLoRaTimer);
            }

            // Turn off resume timer (if on)
            if (resumeLoRaTimerRunning) {
                resumeLoRaTimerRunning = false;
                UTIL_TIMER_Stop(&ResumeLoRaTimer);
            }

            MX_LoRaWAN_Process();
        } else {
            // Connected

            // Stop pause and resume timers
            if (pauseLoRaTimerRunning) {
                pauseLoRaTimerRunning = false;
                UTIL_TIMER_Stop(&PauseLoRaTimer);
            }

            if (resumeLoRaTimerRunning) {
                resumeLoRaTimerRunning = false;
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

    if (!sendDataTimerRunnning) {
        sendDataTimerRunnning = true;
        UTIL_TIMER_Start(&SendDataTimer);
    }

    // With this flag, we try to send data every 5 s
    if (sendDataCallbackFlag) {
        sendDataCallbackFlag = false;

        sensorAppLoRaSendPointer = SensorAppFindNextMeasurementToSend(sensorAppLoRaSendPointer);

        // If nothing was found (all measurements have been sent), get ready for disconnect
        if (sensorAppLoRaSendPointer == (uint16_t) -1) {
            UTIL_TIMER_Stop(&SendDataTimer);
            sendDataTimerRunnning = false;

            sensorAppLoRaSendPointer = 0;

            sendData = false;
            dataSent = true;

            return;
        }

        // If data was sent successfully, remove measurement from storage
        // TODO: Implement confirmed messages?
        if (SendTxData(measurementRAMStorage[sensorAppLoRaSendPointer])) {
            SensorAppRemoveMeasurementFromStorage(sensorAppLoRaSendPointer);
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
    if (status == LORAMAC_HANDLER_SET && !disconnectLoRaTimerRunning) {
        disconnectLoRaTimerRunning = true;
        UTIL_TIMER_Start(&DisconnectLoRaTimer);
    }

    // If timer has been triggered
    if (disconnectLoRaCallbackFlag) {
        disconnectLoRaCallbackFlag = false;

        // If client is connected to network, try to disconnect
        if (status == LORAMAC_HANDLER_RESET || LmHandlerDeInit() == LORAMAC_HANDLER_SUCCESS) {
            // Successful disconnection
            APP_LOG(TS_OFF, VLEVEL_M, "Stopped LoRa connection\r\n");

            if (pauseLoRaTimerRunning) {
                pauseLoRaTimerRunning = false;
                UTIL_TIMER_Stop(&PauseLoRaTimer);
            }

            if (resumeLoRaTimerRunning) {
                resumeLoRaTimerRunning = false;
                UTIL_TIMER_Stop(&ResumeLoRaTimer);
            }

            // Make sure that it's called immediately next time
            disconnectLoRaCallbackFlag = true;

            isLoRaInitialised = false;

            // Make sure it's off
            sendData = false;

            return true;
        }
    }

    MX_LoRaWAN_Process();

    return false;
}
