/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

	#include <stdio.h>
	#include <string.h>
	#include "SX1278.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

	#define SOFT_VERSION 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

	SX1278_hw_t SX1278_hw;
	SX1278_t 	SX1278;

	int master;
	int ret;

	char buffer[64];

	int message;
	int message_length;

	char DataChar[0xFF];
	volatile uint32_t ch1_u32 = 0;
	volatile uint32_t ch2_u32 = 0;
	volatile uint32_t ch3_u32 = 0;
	volatile uint32_t ch4_u32 = 0;
	volatile uint32_t ch5_u32 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	int soft_version_arr_int[3];
	soft_version_arr_int[0] = ((SOFT_VERSION) / 100)     ;
	soft_version_arr_int[1] = ((SOFT_VERSION) /  10) %10 ;
	soft_version_arr_int[2] = ((SOFT_VERSION)      ) %10 ;

	sprintf(DataChar,"\r\n\r\n\tLoRa over sx1278 v%d.%d.%d \r\nUART1 for debug on speed 115200 \r\n",
			soft_version_arr_int[0] ,
			soft_version_arr_int[1] ,
			soft_version_arr_int[2] ) ;
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	#define 	DATE_as_int_str 	(__DATE__)
	#define 	TIME_as_int_str 	(__TIME__)
	sprintf(DataChar,"Build: %s. Time: %s.\r\n" ,
			DATE_as_int_str ,
			TIME_as_int_str ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

		master = 0 ;
	//	master = 1 ;

	if (master == 1) {
		sprintf(DataChar, "Mode: Master\r\n" );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	//	HAL_GPIO_WritePin(MODE_GPIO_Port, MODE_Pin, GPIO_PIN_RESET);
	} else {
		sprintf(DataChar, "Mode: Slave\r\n" );
		HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	//	HAL_GPIO_WritePin(MODE_GPIO_Port, MODE_Pin, GPIO_PIN_SET);
	}

	//initialize LoRa module
	SX1278_hw.dio0.port		= DIO0_GPIO_Port;
	SX1278_hw.dio0.pin 		= DIO0_Pin;
	SX1278_hw.nss.port 		= NSS_GPIO_Port;
	SX1278_hw.nss.pin 		= NSS_Pin;
	SX1278_hw.reset.port	= RESET_GPIO_Port;
	SX1278_hw.reset.pin 	= RESET_Pin;
	SX1278_hw.spi 			= &hspi1;

	SX1278.hw = &SX1278_hw;

	sprintf(DataChar, "Configuring LoRa module ... " );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	SX1278_begin(	&SX1278					,
					SX1278_433MHZ			,
					SX1278_POWER_17DBM		,
					SX1278_LORA_SF_8		,
					SX1278_LORA_BW_20_8KHZ	,
					10						);

	sprintf(DataChar, " done.\r\n" );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	if (master == 1) {
		ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);
	} else {
		ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (master == 1) {
	  	  if (ch1_u32 == 1 ) {
	  			message_length = sprintf(buffer, "Box-1");
	  			ret = SX1278_LoRaEntryTx(	&SX1278			,
	  										message_length	,
	  										2000			);
	  			sprintf(DataChar, "entry: %d, ", ret );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	  			sprintf(DataChar, "send: %s,", buffer );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  			ret = SX1278_LoRaTxPacket(	&SX1278				,
	  										(uint8_t *) buffer	,
	  										message_length		,
	  										2000				);
	  			sprintf(DataChar, "trans: %d.\r\n", ret );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  			HAL_Delay(2500);
	  			ch1_u32 = 0 ;
	  	  }
	  	  //----------------------------------------------------------------------------

	  	  if (ch2_u32 == 1 ) {
	  			message_length = sprintf(buffer, "Box-2");
	  			ret = SX1278_LoRaEntryTx(	&SX1278			,
	  										message_length	,
	  										2000			);
	  			sprintf(DataChar, "entry: %d, ", ret );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	  			sprintf(DataChar, "send: %s,", buffer );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  			ret = SX1278_LoRaTxPacket(	&SX1278				,
	  										(uint8_t *) buffer	,
	  										message_length		,
	  										2000				);
	  			sprintf(DataChar, "trans: %d.\r\n", ret );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  			HAL_Delay(2500);
	  			ch2_u32 = 0 ;
	  	  }
	  	  //----------------------------------------------------------------------------

	  	  if (ch3_u32 == 1 ) {
	  			message_length = sprintf(buffer, "BOX-3");
	  			ret = SX1278_LoRaEntryTx(	&SX1278			,
	  										message_length	,
	  										2000			);
	  			sprintf(DataChar, "entry: %d, ", ret );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	  			sprintf(DataChar, "send: %s, ", buffer );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  			ret = SX1278_LoRaTxPacket(	&SX1278				,
	  										(uint8_t *) buffer	,
	  										message_length		,
	  										2000				);
	  			sprintf(DataChar, "trans: %d.\r\n", ret );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  			HAL_Delay(2500);
	  			ch3_u32 = 0 ;
	  	  }
	  	  //----------------------------------------------------------------------------

	  	  if (ch4_u32 == 1 ) {
	  			message_length = sprintf(buffer, "BOX-4");
	  			ret = SX1278_LoRaEntryTx(	&SX1278			,
	  										message_length	,
	  										2000			);
	  			sprintf(DataChar, "entry: %d, ", ret );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	  			sprintf(DataChar, "send: %s, ", buffer );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  			ret = SX1278_LoRaTxPacket(	&SX1278				,
	  										(uint8_t *) buffer	,
	  										message_length		,
	  										2000				);
	  			sprintf(DataChar, "trans: %d.\r\n", ret );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  			HAL_Delay(2500);
	  			ch4_u32 = 0 ;
	  	  }
	  	  //----------------------------------------------------------------------------

	  	  if (ch5_u32 == 1 ) {
	  			message_length = sprintf(buffer, "Box-5");
	  			ret = SX1278_LoRaEntryTx(	&SX1278			,
	  										message_length	,
	  										2000			);
	  			sprintf(DataChar, "entry: %d, ", ret );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	  			sprintf(DataChar, "send: %s, ", buffer );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  			ret = SX1278_LoRaTxPacket(	&SX1278				,
	  										(uint8_t *) buffer	,
	  										message_length		,
	  										2000				);
	  			sprintf(DataChar, "trans: %d.\r\n", ret );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  			HAL_Delay(2500);
	  			ch5_u32 = 0 ;
	  	  }
	  	  //----------------------------------------------------------------------------



	  	  //----------------------------------------------------------------------------
	  	  //----------------------------------------------------------------------------
	  			message_length = sprintf(buffer, "connect-%d", message);
	  			ret = SX1278_LoRaEntryTx(	&SX1278			,
	  										message_length	,
	  										2000			);
	  			sprintf(DataChar, "entry: %d, ", ret );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	  			sprintf(DataChar, "send: %s, ", buffer );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  			ret = SX1278_LoRaTxPacket(	&SX1278				,
	  										(uint8_t *) buffer	,
	  										message_length		,
	  										2000				);
	  			message += 1;

	  			sprintf(DataChar, "trans: %d.\r\n", ret );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	  			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	  			HAL_Delay(200);
	  			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	  			HAL_Delay(800);
	  		} else {
	  			ret = SX1278_LoRaRxPacket(&SX1278);
	  			sprintf(DataChar, "Rx4: %d ", ret );
	  			HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  			if (ret > 0) {
	  				SX1278_read(&SX1278, (uint8_t *) buffer, ret);
	  				sprintf(DataChar, "\"%s\"", buffer );
	  				HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  				if (buffer[4] == '4') {
	  					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	  					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	  					sprintf(DataChar, "\t\t- - - BINGO - - - - \r\n" );
	  					HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  				} else {
	  					sprintf(DataChar, "\r\n" );
	  					HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	  					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	  					HAL_Delay(50);
	  					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	  				}
	   			} else {
	  				sprintf(DataChar, " \r\n" );
	  				HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	  			}
	  			HAL_Delay(1000);
	  			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	  		}
    /* USER CODE END WHILE */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
