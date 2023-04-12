/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void spi_write(uint16_t x)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi1, &x, 1, 100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 1);
	}

void max7219_write(uint8_t addr, uint8_t dt)
{
spi_write(dt | addr<<8);//sterowanie wyswietlaczem
}

void wyswietlanie1(void)
{		//max7219_write(kolumna, ktore ledy ON) 1 jako zapalona
max7219_write(1,0x00);//pierwsza kolumna, 0000 0000  1 oznacza zapalona LED
max7219_write(2,0x7F);//druga kolumna, 0111 1111     0 ozanacza zgaszona LED
max7219_write(3,0x30);//trzecia kolumna, 0011 0000
max7219_write(4,0x08);//czwarta kolumna, 0000 1000
max7219_write(5,0x08);//to samo tylko w odwrotnej kolejnosci
max7219_write(6,0x30);
max7219_write(7,0x7F);
max7219_write(8,0x00);

}
void wyswietlanie2(void)
{
max7219_write(1,0x00);
max7219_write(2,0x3C);
max7219_write(3,0x06);
max7219_write(5,0x18);
max7219_write(6,0x30);
max7219_write(7,0x60);
max7219_write(8,0xC0);
}

void wyswietlanie3(void)
{
max7219_write(1,0x03);
max7219_write(2,0x02);
max7219_write(3,0x42);
max7219_write(4,0x81);
max7219_write(5,0x81);
max7219_write(6,0x42);
max7219_write(7,0x3C);
max7219_write(8,0x00);
}

void wyswietlanie4(void)
{
max7219_write(1,0x0C);
max7219_write(2,0x10);
max7219_write(3,0x20);
max7219_write(4,0x10);
max7219_write(5,0x0C);
max7219_write(6,0x02);
max7219_write(7,0x0C);
max7219_write(8,0x10);
}

void wyswietlanie5(void)
{
max7219_write(1,0x81);
max7219_write(2,0x42);
max7219_write(3,0x48);
max7219_write(4,0x18);
max7219_write(5,0x18);
max7219_write(6,0x48);
max7219_write(7,0x42);
max7219_write(8,0x81);
}
void plywajace_swiatlo(void)
{
	int i=0;
	while (i<2)//powtorzenie 5 razy
	{
		//Działanie: LEDy beda zapalac sie i gasnac co 0,08s po takim mrugnieciu zapali sie
		  	  	  //kolejna LED, ktora zgasnie po 0,08s
		  	  	  //Kolejnosc LEDow zostala dobrana z pomoca schematu w skrypcie do laboratorium
	HAL_GPIO_TogglePin(LD7_GPIO_Port, LD7_Pin);
	HAL_Delay(80);
	HAL_GPIO_TogglePin(LD7_GPIO_Port, LD7_Pin);
	HAL_GPIO_TogglePin(LD9_GPIO_Port, LD9_Pin);
	HAL_Delay(80);
	HAL_GPIO_TogglePin(LD9_GPIO_Port, LD9_Pin);
	HAL_GPIO_TogglePin(LD10_GPIO_Port, LD10_Pin);
	HAL_Delay(80);
	HAL_GPIO_TogglePin(LD10_GPIO_Port, LD10_Pin);
	HAL_GPIO_TogglePin(LD8_GPIO_Port, LD8_Pin);
	HAL_Delay(80);
	HAL_GPIO_TogglePin(LD8_GPIO_Port, LD8_Pin);
	HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
	HAL_Delay(80);
	HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	HAL_Delay(80);
	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	HAL_Delay(80);
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
	HAL_Delay(80);
	HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
	i++;
	}
	i=0;
}
void max7219_init(void)
  {
  spi_write(0xB07);//proces inicjalizacji wyswietlacza
  spi_write(0x900);//-||-
  spi_write(0xC01);//-||-
  spi_write(0xF00);//-||-
  spi_write(0xA01);//-||-
  wyswietlanie1();//pierwsza klatka
  HAL_Delay(250);
  plywajace_swiatlo();//animacja swiatla pomiedzy klatkami
  HAL_Delay(200);
  wyswietlanie2();//druga klatka
  HAL_Delay(200);
  plywajace_swiatlo();//animacja swiatla pomiedzy klatkami
  HAL_Delay(200);
  wyswietlanie3();//druga klatka
  HAL_Delay(200);
  plywajace_swiatlo();//animacja swiatla pomiedzy klatkami
  HAL_Delay(100);
  wyswietlanie4();//druga klatka
  HAL_Delay(200);
  plywajace_swiatlo();//animacja swiatla pomiedzy klatkami
  HAL_Delay(100);
  wyswietlanie5();//druga klatka
  HAL_Delay(100);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  max7219_init();//wywolanie funkcji (inicjalizacja i wyswietlanie)
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin
                          |LD10_Pin|LD8_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD7_Pin
                           LD10_Pin LD8_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin
                          |LD10_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LD9_Pin */
  GPIO_InitStruct.Pin = LD9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LD9_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
