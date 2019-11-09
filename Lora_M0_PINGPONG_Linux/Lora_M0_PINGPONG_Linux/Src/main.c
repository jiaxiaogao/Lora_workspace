/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

//lora
#include "platform.h"
#include "radio.h"
#include "sx1276-Hal.h"
#include "sx1276-LoRa.h"
#include "sx1276-LoRaMisc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MASTER
//#define SLAVE
#define BUFFERSIZE 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t RegVersion = 0;

#ifdef MASTER 
uint8_t enable_master = true;
#else
uint8_t enable_master = false;
#endif

uint32_t Master_Tx_num = 0;
uint32_t Master_Rx_num = 0;
uint32_t Slave_Tx_num = 0;
uint32_t Slave_Rx_num = 0;

uint16_t buffer_size = BUFFERSIZE;
uint8_t buffer_rev[BUFFERSIZE] = {0};
uint8_t ping_msg [] = "PING";
uint8_t pong_msg [] = "PONG";

//lora设备初始化的返回值
tRadioDriver *Radio = NULL;

//USART2
uint8_t pData [128] = "";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void led_init()//LED初始化为三色
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);//LED_B
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);//LED_R
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);//LED_G
}
void led_send()//发送时：蓝灯
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

void led_rev()//接收时：绿灯
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}
void led_debug()//debug：红灯
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

void master_task()
{
	switch(Radio->Process())
	{
		case RF_RX_DONE:
			led_debug();
			Radio->GetRxPacket(buffer_rev, &buffer_size);
			printf("master_task: -%ld-rev  = -%s-\r\n", Master_Rx_num+1,  buffer_rev);
			if(strncmp((const char *)buffer_rev, "PONG", strlen((const char *)buffer_rev)) == 0)
			{
				Master_Rx_num++;
				Radio->SetTxPacket(ping_msg, strlen((const char *)ping_msg));	
				led_send();
				HAL_Delay(500);
			}
			break;
		case RF_TX_DONE:
				Radio->StartRx();
				led_rev();
				Master_Tx_num++;
				HAL_Delay(500);
			break;
		default:
			break;
	}
}

void slave_task()
{
	switch(Radio->Process())
	{
		case RF_RX_DONE:
			led_debug();
			Radio->GetRxPacket(buffer_rev, &buffer_size);
			printf("slave_task:-%ld- rev  = -%s-\r\n", Slave_Rx_num+1 ,  buffer_rev);
			if(strncmp((const char *)buffer_rev, "PING", strlen((const char *)buffer_rev)) == 0)
			{
				Slave_Rx_num++;
				Radio->SetTxPacket(pong_msg, strlen((const char *)pong_msg));	
				led_send();
				HAL_Delay(500);
			}
			break;
		case RF_TX_DONE:
				Radio->StartRx();
				led_rev();
				Slave_Tx_num++;
				HAL_Delay(500);
			break;
		default:
			break;
	}
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	led_init();
	printf("Hello World!\r\n");
	SX1276Read( REG_LR_VERSION, &RegVersion );
	if(RegVersion != 0x12)
	{
	printf("get RegVersion error, RegVersion = %#x\r\n", RegVersion);
	}
	else
	{
	printf("get RegVersion success, RegVersion = %#x\r\n", RegVersion);
	}
	//sx1278初始化
	Radio = RadioDriverInit();
	Radio->Init();

#ifdef MASTER
	Radio->SetTxPacket(ping_msg, strlen((const char *)ping_msg));	
	printf("MASTER sent ping\r\n");
#else
	Radio->StartRx();
	printf("SLAVE start rev\r\n");
#endif
	while (1)
	{
		if(enable_master)
		{
			master_task();
		}
		else
		{
			slave_task();
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int _write (int fd, char *pBuffer, int size) 
{ 
for (int i = 0; i < size; i++) 
{ 
while((USART1->SR&0X40)==0);//等待上一次串口数据发送完成 
USART1->DR = (uint8_t) pBuffer[i]; //写DR,串口1将发送数据
} 
return size; 
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
