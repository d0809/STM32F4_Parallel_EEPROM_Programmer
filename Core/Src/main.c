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
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "values.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int getc(FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM14_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void delay(unsigned int delay);
void write(uint16_t address, uint8_t data);
uint8_t read(uint16_t address);

volatile uint32_t buf = 0;
volatile uint8_t flag = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int delayconst = 200;
uint8_t poll = 0;

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM14_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  int limit =sizeof(values)/sizeof(uint8_t);//number of bytes to be written
  setvbuf(stdin, NULL, _IONBF, 0);//disable buffers on stdin and stdout
  setvbuf(stdout, NULL, _IONBF, 0);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);//enable receive interrupt
  //
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Init(&htim14);

  HAL_TIM_Base_Start(&htim14);

  /*
   * 28C64B programmer, 8192 addresses 0-8191
   * PB[0-7] -> IO[1-8]
   * PB[8-15] -> A[0-7]
   * PE[2-6] -> A[8-12]
   * PC8 -> WE
   * PC9 -> OE
   * PC10 -> CE high by default
   * TIM14 configured with a prescaler of 167 for 1uS delays
   */
  while (1)
  {
		uint8_t data;
		uint8_t but = 0;
		int errors = 0;

		printf("Press user button to program EEPROM\n");
		while (but == 0) {//wait for button press
			but = HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin);
		}
		but = 0;

		printf("Programming EEPROM\n");

		uint32_t time = HAL_GetTick();
		for (uint16_t address = 0; address < limit; ++address) {
			write(address, values[address]);//write data read from the values array
		}

		time = (HAL_GetTick() - time) / 1000;
		printf("EEPROM programmed, operation took %u seconds. Verify?\n\r",(unsigned int) time);

		while (but == 0) {
			but = HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin);//wait for button plress
		}
		but = 0;

		for (int address = 0; address < limit; ++address) {
			data = read(address);
			if (data != values[address]) {//read from EEPROM and compare
				++errors;
				printf("Found error at %X. Expected %X, got %X\n\r", address, values[address], data);
			}
		}
		printf("Found %i errors, written %i bytes. Waiting for button.", errors,
				limit);
		while (but == 0) {
			but = HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin);
		}
		but = 0;
	}



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 83;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, A8_Pin|A9_Pin|A10_Pin|A11_Pin
                          |A12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, A2_Pin|A3_Pin|A4_Pin|A5_Pin
                          |A6_Pin|A7_Pin|A0_Pin|A1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, WE_Pin|OE_Pin|CE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : A8_Pin A9_Pin A10_Pin A11_Pin
                           A12_Pin */
  GPIO_InitStruct.Pin = A8_Pin|A9_Pin|A10_Pin|A11_Pin
                          |A12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IO0_Pin IO1_Pin IO2_Pin IO3_Pin
                           IO4_Pin IO5_Pin IO6_Pin PB7 */
  GPIO_InitStruct.Pin = IO0_Pin|IO1_Pin|IO2_Pin|IO3_Pin
                          |IO4_Pin|IO5_Pin|IO6_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : A2_Pin A3_Pin A4_Pin A5_Pin
                           A6_Pin A7_Pin A0_Pin A1_Pin */
  GPIO_InitStruct.Pin = A2_Pin|A3_Pin|A4_Pin|A5_Pin
                          |A6_Pin|A7_Pin|A0_Pin|A1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WE_Pin OE_Pin CE_Pin */
  GPIO_InitStruct.Pin = WE_Pin|OE_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void write(uint16_t address, uint8_t data){
	GPIOB->MODER = (GPIOB->MODER & ~0xFFFF) | 0b0101010101010101; //make PB0-7 outputs

	GPIOB->ODR = (GPIOB->ODR & ~(0xFFFF)) | ((address & 0xFF) << 8); //first 8 address bits to PB8-15, data to pb-7
	GPIOE->ODR = (GPIOE->ODR & ~(0b11111 << 2)) | ((address & 0x1F00) >> 6); //upper 5 address bits to pe2-6
	HAL_GPIO_WritePin(GPIOC, CE_Pin, GPIO_PIN_RESET); //enable CE after address and data are set
	HAL_GPIO_WritePin(GPIOC, OE_Pin, GPIO_PIN_SET); //disable WE before writing addresses+data
	HAL_GPIO_WritePin(GPIOC, WE_Pin, GPIO_PIN_SET); //disable OE before writing data

	GPIOB->ODR |= data; //push data out

	for (volatile int i = 0; i != delayconst; i++); //"1us" delay
	HAL_GPIO_WritePin(GPIOC, WE_Pin, GPIO_PIN_RESET);

	for (volatile int i = 0; i != delayconst; i++); //"1us" delay

	HAL_GPIO_WritePin(GPIOC, WE_Pin, GPIO_PIN_SET);

	for (volatile int i = 0; i != delayconst; i++); //"1us" delay

	GPIOB->MODER &= ~0xFFFF;//make PB[0-7] inputs
	HAL_GPIO_WritePin(GPIOC, CE_Pin, GPIO_PIN_SET);
	while (!poll) {//check IO7 to see if the write cycle has ended
		HAL_GPIO_WritePin(GPIOC, CE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, OE_Pin, GPIO_PIN_RESET);
		if ((GPIOB->IDR & 0x80) == (data & 0x80)) {
			poll = 1;
		}
		HAL_GPIO_WritePin(GPIOC, CE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, OE_Pin, GPIO_PIN_SET);
	}
	poll = 0;

}
uint8_t read(uint16_t address){
	uint8_t retval;
	GPIOB->MODER &= ~0xFFFF; //make PB0-7 inputs by setting first 16 bits of MODER to 0
	HAL_GPIO_WritePin(GPIOC, OE_Pin, GPIO_PIN_RESET); //chip in standby but prepared for read mode
	HAL_GPIO_WritePin(GPIOC, WE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CE_Pin, GPIO_PIN_SET);

	GPIOB->ODR = (GPIOB->ODR & ~(0xFF << 8)) | ((address & 0xFF) << 8); //first 8 address bits to PB8-15
	GPIOE->ODR = (GPIOE->ODR & ~(0b11111 << 2))| ((address & 0x1F00) >> 6); //upper 5 address bits to pe2-6

	for (volatile int i = 0; i != delayconst; i++) //"1us" delay
	HAL_GPIO_WritePin(GPIOC, CE_Pin, GPIO_PIN_RESET); //activate chip and read data
	for (volatile int i = 0; i != delayconst; i++); //"1us" delay
	retval = (GPIOB->IDR & 0xFF);
	HAL_GPIO_WritePin(GPIOC, CE_Pin, GPIO_PIN_SET);
	for (volatile int i = 0; i != delayconst; i++); //"1us" delay
	return retval;
}

void delay(unsigned int delay){
		__HAL_TIM_SET_COUNTER(&htim14,0);  // set the counter value a 0
		while (__HAL_TIM_GET_COUNTER(&htim14) < delay);  // wait for the counter to reach the us input in the parameter
}

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 1000);
  return ch;
}

GETCHAR_PROTOTYPE
{
  int ch = 0;
  __HAL_UART_CLEAR_OREFLAG(&huart3);
  HAL_UART_Receive(&huart3, (uint8_t *)&ch, 1, 1000);
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 1000);
  return ch;
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
