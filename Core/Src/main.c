/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_lorawan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "sys_app.h"

#include "stm32_timer.h"
#include "stm32wlxx_hal_rtc.h"
#include "stm32_lpm_if.h"

#include "LmHandler.h"
#include "LoRaMacInterfaces.h"
#include "lora_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MESSAGE_SEND_REQUEST 0
#define MESSAGE_CONFIRMED_1 1
#define MESSAGE_WATER_LEVEL 2
#define MESSAGE_WATER_TEMP 3
#define MESSAGE_WATER_EC 4
#define MESSAGE_WATER_SALINITY 5
#define MESSAGE_WATER_TDS 6
#define MESSAGE_BATTERY_LEVEL 7
#define MESSAGE_CONFIRMED_2 8

#define TIME_INDEX 0
#define WATER_LEVEL_INDEX 1
#define WATER_TEMP_INDEX 2
#define WATER_EC_INDEX 3
#define WATER_SALINITY_INDEX 4
#define WATER_TDS_INDEX 5
#define BATTERY_LEVEL_INDEX 6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static IRDA_HandleTypeDef *hirdaInstance;

// Storage for 256 measurements
static uint8_t *measurementStorage[256];
static uint8_t measurementStoragePointer = 0;

// Storage for a single measurement, which
static char *measurement[7];

static bool startMeasurements = false;
static bool readingMeasurements = false;
static bool wakeUp = false;
static uint8_t wakeUpCounter = 0;

static uint8_t sendData = 0;
static bool sendDataTimerRunnning = false;
static bool sendDataDone = false;
static bool postSendDataDone = false;
static bool stopLora = false;
static bool stopLoraTimerRunning = false;

static UTIL_TIMER_Object_t startMeasurementsTimer;
static UTIL_TIMER_Object_t wakeSensorUpTimer;
static UTIL_TIMER_Object_t sendDataTimer;
static UTIL_TIMER_Object_t stopLoraTimer;

static uint8_t commandsSendPointer = 0;
static char *commandsSend[] = {
	"S\r",
	"F0017G0010\r",
	"F0017G0020\r",
	"F0017G0030\r",
	"F0017G0035\r",
	"F0017G0036\r",
	"F0017G0090\r",
	"A\r"
};

static uint8_t commandsReceivePointer = 0;
static char *commandsReceive[] = {
	"?08\r",
	"*\r",
	"K85 00170010",
	"K85 00170020",
	"K85 00170030",
	"K85 00170035",
	"K85 00170036",
	"K85 00170090",
	"*\r"
};

// Buffer for whole measurement frames
static char measurementTime[6 + 6];
static char waterLevel[2 + 7];
static char waterTemp[2 + 7];
static char waterEC[2 + 7];
static char waterSalinity[2 + 7];
static char waterTDS[2 + 7];
static char batteryLevel[2 + 7];

static char rxBuffer[256];
static uint8_t rxPointer = 0;

static uint8_t reception[1];

static bool IRDA_Receive() {
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
		default: {}
	}

	return false;
}

static void StartMeasurements(void) {
	APP_LOG(TS_OFF, VLEVEL_M, "Starting measurement timer triggered\r\n");
	startMeasurements = true;
	PWR_ExitSleepMode();
}

static void WakeSensorUp(void) {
	wakeUp = true;
	wakeUpCounter++;
}

static void SendData(void) {
	sendData = SendTxData(measurementTime, waterLevel, waterTemp, waterEC, waterSalinity, waterTDS, batteryLevel);

	// If data was successfully sent
	if (!sendData) {
		sendDataDone = true;
	}
}

static void StopLora(void) {
	stopLora = true;
}

/*
 * This function calculates the checksum of a given message.
 * Message includes "\r" at end of the transmission.
 */
static bool ChecksumMessage(const char *message) {
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

// Pointer to measurement, pointer to dataCounter
static void ExtractMeasurement(const char *receiveString, char *measurement) {
	uint8_t dataCounter = 0;

	char *ptrBuffer = strstr(receiveString, "K06");		/* K06 is for command */
	for (uint8_t i = 10; i <= 11; i++) {				/* Checking all date character and put in sender buffer */
		if (*(ptrBuffer + i) != ' ') {
			measurement[dataCounter++] = *(ptrBuffer + i);
		}
	}

	ptrBuffer = strstr(receiveString, "K20");			/* K20 is for data */
	for (uint8_t i = 11; i <= 17; i++) {				/* Checking all date character and put in sender buffer */
		if (*(ptrBuffer + i) != ' ') {
			measurement[dataCounter++] = *(ptrBuffer + i);
		}
	}
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //uint8_t sizes[] = {4};
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_LoRaWAN_Init();
  SystemApp_Init();
  MX_USART2_IRDA_Init();
  /* USER CODE BEGIN 2 */
  APP_LOG(TS_OFF, VLEVEL_M, "Program start\r\n");

  hirdaInstance = &hirda2;

  // Create a timer that handles measurement taking
  UTIL_TIMER_Create(&startMeasurementsTimer, 60000, UTIL_TIMER_PERIODIC, StartMeasurements, NULL);
  UTIL_TIMER_Create(&wakeSensorUpTimer, 200, UTIL_TIMER_PERIODIC, WakeSensorUp, NULL);
  UTIL_TIMER_Create(&sendDataTimer, 20000, UTIL_TIMER_PERIODIC, SendData, NULL);
  UTIL_TIMER_Create(&stopLoraTimer, 6000, UTIL_TIMER_PERIODIC, StopLora, NULL);

  UTIL_TIMER_Start(&startMeasurementsTimer);

  startMeasurements = true;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if (startMeasurements) {
		startMeasurements = false;

		APP_LOG(TS_OFF, VLEVEL_M, "Starting measurement\r\n");

		// For some reason, we need to reinit the IRDA module
		MX_USART2_IRDA_Init();

		// Just to be safe, flush the RX and TX buffers
		__HAL_IRDA_FLUSH_DRREGISTER(&(*hirdaInstance));

		UTIL_TIMER_Start(&wakeSensorUpTimer);

		readingMeasurements = true;

		while (readingMeasurements) {
			if (wakeUp) {
				wakeUp = false;

				if (wakeUpCounter < 5) {
					uint8_t messageSend[] = "A\r";

					while (HAL_OK != HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 10)) {}
				} else if (wakeUpCounter > 30) {
					wakeUpCounter = 0;
				}
			}

			if (IRDA_Receive() && rxPointer != 0) {
				// Get received content in string form, so that it can be compared
				char receiveString[rxPointer + 1];

				strncpy(receiveString, rxBuffer, rxPointer);

				// String must end with a NULL char
				receiveString[rxPointer] = '\0';

				APP_LOG(TS_OFF, VLEVEL_M, receiveString);
				APP_LOG(TS_OFF, VLEVEL_M, "\r\n");

				// Echo for testing purposes
				//HAL_IRDA_Transmit(&(*hirdaInstance), receiveString, sizeof(receiveString) - 1, 200);

				// If message contains checksum and it failed, advise there was an error
				if (strstr(receiveString, "K23") != NULL && !ChecksumMessage(receiveString)) {
					uint8_t messageSend[] = "?06\r";

					while (HAL_OK != HAL_IRDA_Transmit(&(*hirdaInstance), messageSend, sizeof(messageSend) - 1, 50)) {}
				} else if (!strcmp(receiveString, commandsReceive[commandsReceivePointer])) {
					switch (commandsReceivePointer) {
						case MESSAGE_SEND_REQUEST: {
							// Stop timer
							UTIL_TIMER_Stop(&wakeSensorUpTimer);

							wakeUpCounter = 0;

							break;
						}
						case MESSAGE_CONFIRMED_1: {
							break;
						}
						case MESSAGE_WATER_LEVEL: {
							uint8_t dataCounter = 0;

							char measurementTime[6 + 6];

							/* DATE in format ddmmyy */
							char *ptrBuffer = strstr(receiveString, "K28");	/* K28 is for date */
							for (uint8_t i = 4; i <= 11; i++) { 			/* Get all date characters and put them in transmit buffer */
								if (*(ptrBuffer + i) != ' ') {
									measurementTime[dataCounter++] = *(ptrBuffer + i);
								}
							}

							/* TIME in format hhmmss */
							ptrBuffer = strstr(receiveString, "K24"); 		/* K24 is for time */
							for (uint8_t i = 4; i <= 11; i++) { 			/* Get all time characters and put them in transmit buffer */
								if (*(ptrBuffer + i) != ' ') {
									measurementTime[dataCounter++] = *(ptrBuffer + i);
								}
							}

							measurement[TIME_INDEX] = measurementTime;

							ExtractMeasurement(receiveString, measurement[WATER_LEVEL_INDEX]);
							break;
						}
						case MESSAGE_WATER_TEMP: {
							ExtractMeasurement(receiveString, measurement[WATER_TEMP_INDEX]);
							break;
						}
						case MESSAGE_WATER_EC: {
							ExtractMeasurement(receiveString, measurement[WATER_EC_INDEX]);
							break;
						}
						case MESSAGE_WATER_SALINITY: {
							ExtractMeasurement(receiveString, measurement[WATER_SALINITY_INDEX]);
							break;
						}
						case MESSAGE_WATER_TDS: {
							ExtractMeasurement(receiveString, measurement[WATER_TDS_INDEX]);
							break;
						}
						case MESSAGE_BATTERY_LEVEL: {
							ExtractMeasurement(receiveString, measurement[BATTERY_LEVEL_INDEX]);
							break;
						}
						case MESSAGE_CONFIRMED_2: {
							readingMeasurements = false;

							APP_LOG(TS_OFF, VLEVEL_M, "Establishing LoRa connection\r\n");

							MX_LoRaWAN_Init();

							sendData = 1;
							break;
						}
					}

					if (commandsReceivePointer < MESSAGE_CONFIRMED_2) {
						commandsReceivePointer++;

						// Don't transmit the last char ('\0')
						while (HAL_OK != HAL_IRDA_Transmit(&(*hirdaInstance), commandsSend[commandsSendPointer], sizeof(commandsSend[commandsSendPointer]) - 1, 50)) {}

						commandsSendPointer++;
					}
				} else {
					// Not awaited reception handling goes here
					if (rxPointer != 0) {

					}
				}

				rxPointer = 0;
			}
		}
	}

	// Keep trying to send the data until it's successful
	if (sendData) {
		if (!sendDataTimerRunnning) {
			sendDataTimerRunnning = true;
			UTIL_TIMER_Start(&sendDataTimer);
		}

		/* USER CODE END WHILE */
		MX_LoRaWAN_Process();

    /* USER CODE BEGIN 3 */
	}

	if (sendDataDone) {
		sendDataDone = false;

		UTIL_TIMER_Stop(&sendDataTimer);
		sendDataTimerRunnning = false;

		postSendDataDone = true;
	}

	if (postSendDataDone) {
		if (!stopLoraTimerRunning) {
			stopLoraTimerRunning = true;
			UTIL_TIMER_Start(&stopLoraTimer);
		}

		if (stopLora) {
			stopLora = false;

			if (LmHandlerJoinStatus()) {
				int8_t loraRunning = LmHandlerDeInit();

				APP_LOG(TS_OFF, VLEVEL_M, "Stopping LoRa connection: %d\r\n", loraRunning);

				if (loraRunning == 0) {
					UTIL_TIMER_Stop(&stopLoraTimer);
					postSendDataDone = false;

					//HAL_RTC_
					PWR_EnterSleepMode();
				}
			}
		}

		MX_LoRaWAN_Process();
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
