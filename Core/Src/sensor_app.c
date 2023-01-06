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
uint32_t measurementIntervalMins = 1;
uint8_t sendMeasurementsAfterNumber = 1;

uint32_t configAddress = FLASH_START_ADDRESS_CONFIG;

bool measurementIntervalMinsUpdated = false;
bool sendMeasurementsAfterNumberUpdated = false;

// 6 + 6(Time) + 6 * (2 + 7) (Measurements: Registers & Values) + 1 (NULL char at end)
//static char measurementExtracted[SIZE_MEASUREMENT] = {'\0'};
// 4 * 64 bits = 32 bytes
static uint8_t measurementExtracted[SIZE_MEASUREMENT] = {0};
static uint8_t measurementExtractedPointer = 0;

// Measurement storage
static uint8_t/*char*/ *measurementStorage[SIZE_STORAGE] = {NULL};
static uint8_t numberOfMeasurementsInStorage = 0;

bool startMeasurements = false;
bool readingMeasurements = false;
static bool wakeUpCallbackFlag = false;
static uint8_t wakeUpCounter = 0;

bool sendData = false;
static bool sendDataCallbackFlag = false;
bool sendDataTimerRunnning = false;
bool dataSent = false;
static bool disconnectLoRaCallbackFlag = true;
static bool disconnectLoRaTimerRunning = false;

static bool pauseLoRa = false;
static bool pauseLoRaTimerRunning = false;
static bool resumeLoRaTimerRunning = false;

static bool isLoRaInitialised = false;

static uint8_t message = 0;

static UTIL_TIMER_Object_t MeasurementTimer;
static UTIL_TIMER_Object_t WakeSensorUpTimer;
UTIL_TIMER_Object_t SendDataTimer;
static UTIL_TIMER_Object_t DisconnectLoRaTimer;
static UTIL_TIMER_Object_t PauseLoRaTimer;
static UTIL_TIMER_Object_t ResumeLoRaTimer;

static void StartMeasurementsTimerCallback(void *context) {
	APP_LOG(TS_OFF, VLEVEL_M, "Starting measurements timer triggered\r\n");
	startMeasurements = true;

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
static uint8_t SensorAppFindNextMeasurementToSend(uint8_t index) {
	uint8_t startIndex = index;
	bool overflow = false;

	// Iterate over the whole storage array to see if there is a taken slot
	while (measurementStorage[index] == NULL) {
		if (overflow && startIndex == index) {
			return (uint8_t) -1;
		}

		index++;

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
static uint8_t SensorAppFindNextFreeSpot(uint8_t index) {
	uint8_t startIndex = index;
	bool overflow = false;

	// Iterate over the whole storage array to see if there is a free slot
	while (measurementStorage[index] != NULL) {
		if (overflow && startIndex == index) {
			return (uint8_t) -1;
		}

		index++;

		if (index == SIZE_STORAGE) {
			index = 0;
			overflow = true;
		}
	}

	return index;
}

/*
 * Adds a measurement to a spot. Returns if successful or not.
 */
static bool SensorAppAddMeasurementToStorage(const /*char*/uint8_t *measurement) {
	static uint8_t measurementStoragePointer = 0;

	// Find a free spot to put upcoming measurements
	uint8_t index = SensorAppFindNextFreeSpot(measurementStoragePointer);

	// If there is no more free space available
	if (index == (uint8_t) -1) {
		// TODO: Implement countermeasure?
		APP_LOG(TS_OFF, VLEVEL_M, "Measurement storage is full!!!\r\n");

		return false;
	} else {
		// If index is out of bounds, stop
		if (index >= SIZE_STORAGE) {
			return false;
		}

		measurementStoragePointer = index;
	}

	// Allocate memory for new array
	//char *storedMeasurement = (char*) calloc(SIZE_MEASUREMENT, sizeof(char));
	uint8_t *storedMeasurement = (uint8_t*) calloc(SIZE_MEASUREMENT, sizeof(uint8_t));

	// If memory can't be allocated
	if (storedMeasurement == NULL) {
		return false;
	}

	// Copy measurement into storedMeasurement
	//strcpy(storedMeasurement, measurement);
	memcpy(storedMeasurement, measurement, sizeof(measurement));

	measurementStorage[measurementStoragePointer] = storedMeasurement;

	// Write measurement into flash storage
	FlashAppWriteMeasurement(storedMeasurement);

	numberOfMeasurementsInStorage++;

	return true;
}

/*
 * Resets a take measurement spot.
 */
static void SensorAppRemoveMeasurementFromStorage(uint8_t index) {
	// If out of bounds, do nothing
	if (index >= SIZE_STORAGE) {
		return;
	}

	// Unallocate the memory for array
	free(measurementStorage[index]);

	// Set content to nullptr
	measurementStorage[index] = NULL;

	// Reduce the amount of measurements in storage
	numberOfMeasurementsInStorage--;
}

/*
 * Extracts date and time from first raw measurement message.
 */
static void ExtractMeasurementTime(const char *measurementRaw) {
	// Reset pointer
	measurementExtractedPointer = 0;

	uint64_t dateAndTime = 0;
	uint64_t power = 100000000000;

	/* DATE in format ddmmyy */
	char *ptrBuffer = strstr(measurementRaw, "K28");	/* K28 is for date */
	for (uint8_t i = 4; i <= 11; i++) { 				/* Get all date characters and put them in transmit buffer */
		if (*(ptrBuffer + i) != ' ') {
			dateAndTime += (*(ptrBuffer + i) - 0x30) * power;
			power /= 10;
		}
	}

	/* TIME in format hhmmss */
	ptrBuffer = strstr(measurementRaw, "K24"); 			/* K24 is for time */
	for (uint8_t i = 4; i <= 11; i++) { 				/* Get all time characters and put them in transmit buffer */
		if (*(ptrBuffer + i) != ' ') {
			dateAndTime += (*(ptrBuffer + i) - 0x30) * power;
			power /= 10;
		}
	}

	// LSB first in buffer
	for (uint8_t i = 0; i < 8; i++) {
		measurementExtracted[measurementExtractedPointer++] = (uint8_t) (dateAndTime >> (i * 8));
	}

	// Returns correct value, APP_LOG doesn't put it correctly in string
	APP_LOG(TS_OFF, VLEVEL_M, "Date and time: %d\r\n", dateAndTime);
}

static void ExtractMeasurementTimeString(const char *measurementRaw) {
	// Reset pointer
	measurementExtractedPointer = 0;

	/* DATE in format ddmmyy */
	char *ptrBuffer = strstr(measurementRaw, "K28");	/* K28 is for date */
	for (uint8_t i = 4; i <= 11; i++) { 				/* Get all date characters and put them in transmit buffer */
		if (*(ptrBuffer + i) != ' ') {
			measurementExtracted[measurementExtractedPointer++] = *(ptrBuffer + i);
		}
	}

	/* TIME in format hhmmss */
	ptrBuffer = strstr(measurementRaw, "K24"); 			/* K24 is for time */
	for (uint8_t i = 4; i <= 11; i++) { 				/* Get all time characters and put them in transmit buffer */
		if (*(ptrBuffer + i) != ' ') {
			measurementExtracted[measurementExtractedPointer++] = *(ptrBuffer + i);
		}
	}
}

/*
 * Extracts measurement values from raw measurement message.
 */
static void ExtractMeasurementValue(const char *measurementRaw) {
	uint8_t registerNumber = 0;
	uint16_t power = 10;

	char *ptrBuffer = strstr(measurementRaw, "K06");	/* K06 is for command */
	for (uint8_t i = 10; i <= 11; i++) {
		if (*(ptrBuffer + i) != ' ') {
			registerNumber += (*(ptrBuffer + i) - 0x30) * power;
			power /= 10;
		}
	}

	ptrBuffer = strstr(measurementRaw, "K20");			/* K20 is for data */

	uint8_t commaPosition = *(ptrBuffer + 11) - 0x30;
	uint8_t sign = *(ptrBuffer + 12) != '0';			/* + = 0, - = 1*/

	uint16_t value = 0;
	power = 10000;

	for (uint8_t i = 13; i <= 17; i++) {
		if (*(ptrBuffer + i) != ' ') {
			value += (*(ptrBuffer + i) - 0x30) * power;
			power /= 10;
		}
	}

	// 24 bits long
	uint32_t measurementHeader = (registerNumber << 4) + (commaPosition << 1) + sign;
	uint32_t measurement = value + (measurementHeader << 16);

	// LSB first in buffer
	for (uint8_t i = 0; i < 4; i++) {
		measurementExtracted[measurementExtractedPointer++] = (uint8_t) (measurement >> (i * 8));
	}
};

static void ExtractMeasurementValueString(const char *measurementRaw) {
	char *ptrBuffer = strstr(measurementRaw, "K06");	/* K06 is for command */
	for (uint8_t i = 10; i <= 11; i++) {
		if (*(ptrBuffer + i) != ' ') {
			measurementExtracted[measurementExtractedPointer++] = *(ptrBuffer + i);
		}
	}

	ptrBuffer = strstr(measurementRaw, "K20");			/* K20 is for data */
	for (uint8_t i = 11; i <= 17; i++) {
		if (*(ptrBuffer + i) != ' ') {
			measurementExtracted[measurementExtractedPointer++] = *(ptrBuffer + i);
		}
	}
};

static char rxBuffer[256] = {'\0'};
static uint8_t rxPointer = 0;

/*
 * Polls for IRDA reception.
 */
static bool IRDA_Receive(void) {
	static uint8_t reception[1] = {'\0'};

	// Wait for 10 ms to see if anything else has been sent
	switch (HAL_IRDA_Receive(&(*hirdaInstance), reception, sizeof(reception), 50)) {
		// Reception is complete
		case HAL_TIMEOUT: {
			return true;
		}
		case HAL_OK: {
			rxBuffer[rxPointer++] = reception[0];
			break;
		}
		default: {}
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
	uint16_t checksumRead = (((uint16_t) message[messageLength - 7]) - 0x0030) * 1000;
	checksumRead += (((uint16_t) message[messageLength - 6]) - 0x0030) * 100;
	checksumRead += (((uint16_t) message[messageLength - 5]) - 0x0030) * 10;
	checksumRead += ((uint16_t) message[messageLength - 4]) - 0x0030;

	uint16_t checksumCalculated = 0;

	// Skip over checksum characters
	for (uint8_t i = 0; i < messageLength - 10; i++) {
		checksumCalculated += message[i];
	}

	// Replace checksum within message with spaces
	checksumCalculated += 8 * 0x0020 + message[messageLength - 2];

	APP_LOG(TS_OFF, VLEVEL_M, "Checksum Read: %d | Calculated: %d\r\n", checksumRead, checksumCalculated);

	return checksumRead == checksumCalculated;
}

void SensorAppFindConfig(void) {

}

void SensorAppReadConfig(void) {
	APP_LOG(TS_OFF, VLEVEL_M, "Reading config start\r\n");

	APP_LOG(TS_OFF, VLEVEL_M, "Finding config start\r\n");

	uint64_t readBuffer[1];
	uint32_t i = FLASH_START_ADDRESS_CONFIG;

	// Go through entire memory page to find where last config is stored
	// If current word-length memory content is empty (64 bits set to 1), it means previous cell had config
	while (i < FLASH_END_ADDRESS_CONFIG) {
		FlashAppReadData(i, readBuffer, 1);

		if ((~readBuffer[0]) != 0) {
			if (i == FLASH_START_ADDRESS_CONFIG) {
				configAddress = FLASH_START_ADDRESS_CONFIG;
			} else {
				configAddress = i - 8;
			}

			break;
		}

		i += 8;
	}

	APP_LOG(TS_OFF, VLEVEL_M, "Config address: %d\r\n", configAddress);
	APP_LOG(TS_OFF, VLEVEL_M, "Finding config done\r\n");

	// Parse config data
	uint32_t measurementIntervalMinsTemp = (uint32_t) readBuffer[0];
	uint32_t sendMeasurementsAfterNumberTemp = (uint32_t) (readBuffer[0] >> 32);

	APP_LOG(TS_OFF, VLEVEL_M, "measurementIntervalMins: %d\r\n", measurementIntervalMinsTemp);
	APP_LOG(TS_OFF, VLEVEL_M, "sendMeasurementsAfterNumber: %d\r\n", sendMeasurementsAfterNumberTemp);

	// Make sure config is valid
	if (measurementIntervalMinsTemp >= MINIMUM_TIMER_INTERVAL_MINUTES && measurementIntervalMinsTemp <= MAXIMUM_TIMER_INTERVAL_MINUTES) {
		measurementIntervalMins = measurementIntervalMinsTemp;
	}

	if (sendMeasurementsAfterNumberTemp >= MINIMUM_SEND_MEASUREMENTS_AFTER&& sendMeasurementsAfterNumberTemp <= MAXIMUM_SEND_MEASUREMENTS_AFTER) {
		sendMeasurementsAfterNumber = (uint8_t) sendMeasurementsAfterNumberTemp;
	}

	APP_LOG(TS_OFF, VLEVEL_M, "measurementIntervalMins: %d\r\n", measurementIntervalMins);
	APP_LOG(TS_OFF, VLEVEL_M, "sendMeasurementsAfterNumber: %d\r\n", sendMeasurementsAfterNumber);
	APP_LOG(TS_OFF, VLEVEL_M, "Reading config done\r\n");
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

		// Write to flash memory
		FlashAppWriteConfigData((((uint64_t) sendMeasurementsAfterNumber) << 32) | measurementIntervalMins);
	}
}

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

void SensorAppReadMeasurementsInit(void) {
	// For some reason, we need to reinit the IRDA module
	MX_USART2_IRDA_Init();

	// Just to be safe, flush the RX and TX buffers
	__HAL_IRDA_FLUSH_DRREGISTER(&(*hirdaInstance));

	startMeasurements = false;
	APP_LOG(TS_OFF, VLEVEL_M, "Starting measurements\r\n");

	UTIL_TIMER_Start(&WakeSensorUpTimer);

	readingMeasurements = true;
}

void SensorAppReadMeasurements(void) {
	if (wakeUpCallbackFlag) {
		wakeUpCallbackFlag = false;

		if (wakeUpCounter < 5) {
			uint8_t messageSend[] = "A\r";

			while (HAL_OK != HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 10)) {}
		} else if (wakeUpCounter > 30) {
			wakeUpCounter = 0;
		}
	}

	if (IRDA_Receive() && rxPointer != 0) {
		// Get received content in string form, so that it can be compared
		char receivedMessage[rxPointer + 1];

		strncpy(receivedMessage, rxBuffer, rxPointer);

		// String must end with a NULL char
		receivedMessage[rxPointer] = '\0';

		// If message contains checksum
		if (strstr(receivedMessage, "K23") != NULL && !IRDA_checksum(receivedMessage)) {
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
					SensorAppAddMeasurementToStorage(measurementExtracted);

					APP_LOG(TS_OFF, VLEVEL_M, "Finishing measurements | Number of measurements taken: %d\r\n\r\n", numberOfMeasurementsInStorage);

					if (numberOfMeasurementsInStorage >= sendMeasurementsAfterNumber) {
						APP_LOG(TS_OFF, VLEVEL_M, "Sending measurements over LoRa\r\n");

						pauseLoRa = false;

						sendData = true;
					} else {
						// Sleep?
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
			//////////////Not awaited reception handling goes here
			if (rxPointer != 0) {

			}
		}

		rxPointer = 0;
	}
}

/*
 * Tries to join the LoRa network if it hasn't joined. Returns if connected to LoRa network.
 */
bool SensorAppLoRaJoin(void) {
	LmHandlerFlagStatus_t status = LmHandlerJoinStatus();

	if (pauseLoRa) {
		// Pause
		if (status == LORAMAC_HANDLER_RESET) {
			// Disconnected
			if (pauseLoRaTimerRunning) {
				pauseLoRaTimerRunning = false;
				UTIL_TIMER_Stop(&PauseLoRaTimer);
			}

			if (!resumeLoRaTimerRunning) {
				resumeLoRaTimerRunning = true;
				UTIL_TIMER_Start(&ResumeLoRaTimer);
			}
		} else {
			// Connected

			// If still connected, try disconnecting
			if (SensorAppLoRaDisconnect()) {
				APP_LOG(TS_OFF, VLEVEL_M, "Pausing LoRa connection\r\n");
			}
		}
	} else {
		// Resume
		if (status == LORAMAC_HANDLER_RESET) {
			// Disconnected

			// Initialise LoRa
			if (!isLoRaInitialised) {
				isLoRaInitialised = true;

				MX_LoRaWAN_Init();
			}

			if (!pauseLoRaTimerRunning) {
				pauseLoRaTimerRunning = true;
				UTIL_TIMER_Start(&PauseLoRaTimer);
			}

			if (resumeLoRaTimerRunning) {
				resumeLoRaTimerRunning = false;
				UTIL_TIMER_Stop(&ResumeLoRaTimer);
			}

			MX_LoRaWAN_Process();
		} else {
			// Connected

			// Stop timers
			if (pauseLoRaTimerRunning) {
				pauseLoRaTimerRunning = false;
				UTIL_TIMER_Stop(&PauseLoRaTimer);

				APP_LOG(TS_OFF, VLEVEL_M, "Timer pause loRa stopped\r\n");
			}

			if (resumeLoRaTimerRunning) {
				resumeLoRaTimerRunning = false;
				UTIL_TIMER_Stop(&ResumeLoRaTimer);

				APP_LOG(TS_OFF, VLEVEL_M, "Timer resume loRa stopped\r\n");
			}
		}
	}

	return LmHandlerJoinStatus() == LORAMAC_HANDLER_SET;
}

/*
 * Sends data once connected to LoRa.
 */
void SensorAppLoRaSend(void) {
	static uint8_t sensorAppLoRaSendPointer = 0;

	if (!sendDataTimerRunnning) {
		sendDataTimerRunnning = true;
		UTIL_TIMER_Start(&SendDataTimer);
	}

	// With this flag, we try to send data every 5 s
	if (sendDataCallbackFlag) {
		sendDataCallbackFlag = false;

		sensorAppLoRaSendPointer = SensorAppFindNextMeasurementToSend(sensorAppLoRaSendPointer);

		// If nothing was found (all measurements have been sent), get ready for disconnect
		if (sensorAppLoRaSendPointer == (uint8_t) -1) {
			UTIL_TIMER_Stop(&SendDataTimer);
			sendDataTimerRunnning = false;

			sensorAppLoRaSendPointer = 0;

			sendData = false;
			dataSent = true;

			return;
		}

		// If data was sent successfully, remove measurement from storage
		// TODO: Implement confirmed messages?
		if (SendTxData(measurementStorage[sensorAppLoRaSendPointer])) {
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
		if (status == LORAMAC_HANDLER_RESET || (status == LORAMAC_HANDLER_SET && LmHandlerDeInit() == LORAMAC_HANDLER_SUCCESS)) {
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

			return true;
		}
	}

	MX_LoRaWAN_Process();

	return false;
}
