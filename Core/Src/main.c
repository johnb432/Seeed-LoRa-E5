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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
bool messageReceived = false;
bool messageReceivedFull = false;
bool messageTransmitted = false;
bool error = false;

UTIL_TIMER_Object_t ReceiveTimeout;

uint8_t reception[1];

uint8_t rxBuffer[256] = {'\0'};
uint8_t rxPointer = 0;

void HAL_IRDA_RxCpltCallback(IRDA_HandleTypeDef *hirda) {
	messageReceived = true;
	messageReceivedFull = true; // Remove when ready to test with more chars
}

extern void HAL_IRDA_TxCpltCallback(IRDA_HandleTypeDef *hirda) {
	messageTransmitted = true;
}

extern void HAL_IRDA_ErrorCallback(IRDA_HandleTypeDef *hirda) {
	error = true;
}

void IRDA_messageReceveivedFull() {
	messageReceivedFull = true;
}

/*
 * This function calculates the checksum of a given message.
 * Message includes "\r" at end of the transmission.
 */
bool IRDA_checksum(const char *message) {
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

	APP_LOG(TS_ON, VLEVEL_M, "Checksum Read: %d | Calculated: %d\r\n", checksumRead, checksumCalculated);

	return checksumRead == checksumCalculated;
}

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LoRaWAN_Init();
  MX_USART2_IRDA_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //UTIL_TIMER_Create(&ReceiveTimeout, 1, UTIL_TIMER_ONESHOT, IRDA_messageReceveivedFull, NULL);

  HAL_IRDA_Receive_IT(&hirda2, reception, sizeof(reception));

  APP_LOG(TS_ON, VLEVEL_M, "IRDA IDLE INTERRUPT ENABLED: %d", __HAL_IRDA_GET_IT_SOURCE(&hirda2, IRDA_IT_IDLE));

  while (1)
  {
	if (messageReceived) {
		messageReceived = false;

		//__HAL_IRDA_FLUSH_DRREGISTER(&hirda2);
		//SET_BIT((&hirda2)->Instance->RQR, IRDA_RXDATA_FLUSH_REQUEST);
		//// Nearly the same as the above, but below has a casting to uint16_t
		//__HAL_IRDA_SEND_REQ(&hirda2, IRDA_RXDATA_FLUSH_REQUEST);

		//rxBuffer[rxPointer++] = reception[0];

		//// Reset timeout timer
		//UTIL_TIMER_Stop(&ReceiveTimeout);
		//UTIL_TIMER_Start(&ReceiveTimeout);
	}

	if (messageReceivedFull) {
		messageReceivedFull = false;

		/*
		char measurement[] = "Test";

		if (strstr(measurement, "K23") != NULL) {
			IRDA_checksum(measurement);
		}
		*/

		//UTIL_TIMER_Stop(&ReceiveTimeout);

		APP_LOG(TS_ON, VLEVEL_M, "MESSAGE RECEIVED: %", reception[0]);

		HAL_IRDA_Transmit_IT(&hirda2, reception, sizeof(reception));
		//HAL_IRDA_Transmit_IT(&hirda2, rxBuffer, rxPointer);
		HAL_IRDA_Receive_IT(&hirda2, reception, sizeof(reception));
	}

	if (messageTransmitted) {
		messageTransmitted = false;
		APP_LOG(TS_ON, VLEVEL_M, "MESSAGE TRANSMITTED");
	}

	if (error) {
		APP_LOG(TS_ON, VLEVEL_M, "IRDA CALLBACK ERROR: %d", hirda2.ErrorCode);

		//__HAL_IRDA_CLEAR_OREFLAG(&hirda2);

		//uint8_t isLineClearFlagTriggered = __HAL_IRDA_GET_IT(&hirda2, IRDA_IT_IDLE);
		//uint8_t isLineClearFlagEnabled = __HAL_IRDA_GET_IT_SOURCE(&hirda2, IRDA_IT_IDLE);
	}

    /* USER CODE END WHILE */
    MX_LoRaWAN_Process();

    /* USER CODE BEGIN 3 */
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
