/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define CE_PORT GPIOB		//PB6 chip enable (aka slave select)
#define CE_PIN GPIO_PIN_6

#define DC_PORT GPIOA		//PA0 data/control
#define DC_PIN GPIO_PIN_0

#define RESET_PORT GPIOA	//PA1 reset
#define RESET_PIN GPIO_PIN_1

#define GLCD_WIDTH 84
#define GLCD_HEIGHT 48
#define NUM_BANKS 6

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
const char font_table[][6] = {
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // space
		{0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00}, // 'A'
		{0x7F, 0x49, 0x49, 0x49, 0x36, 0x00}, // 'B'
		{0x3E, 0x41, 0x41, 0x41, 0x22, 0x00}, // 'C'
		{0x00, 0x7F, 0x08, 0x08, 0x7F, 0x00}, // 'H' 4
		{0x00, 0x7F, 0x49, 0x49, 0x00, 0x00}, // 'E' 5
		{0x00, 0x7F, 0x40, 0x40, 0x00, 0x00}, // 'L' 6
		{0x00, 0x7F, 0x41, 0x41, 0x7F, 0x00}, // 'O' 7
		{0xFF, 0x80, 0x78, 0x78, 0x80, 0xFF}, // 'W' 8
		{0x00, 0xFF, 0x19, 0x29, 0xC6, 0x00}, // 'R' 9
		{0x00, 0xFF, 0x81, 0x81, 0x7C, 0x00}, // 'D' 10
		{0x00, 0x00, 0xDF, 0xDF, 0x00, 0x00}, // 'O' 11
		{0x00, 0x7E, 0x81, 0xB5, 0xA1, 0xA1}, // smile half 1 (12)
		{0xA1, 0xA1, 0xB5, 0x81, 0x7E, 0x00}, // smile half 2 (13)
		{0x00, 0x7E, 0x7E, 0x7E, 0x7E, 0x00}, // snake head start (14)
		{0x00, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E}, // snake moving right from start (15)
		{0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E}, // snake moving right from left or left from right (16)
		{0x7E, 0xFE, 0xFE, 0xFE, 0xFE, 0x00}, // snake moving down from left (17)
		{0x00, 0x7F, 0x7F, 0x7F, 0x7F, 0x00}, // snake head moved down (18)
		{0x00, 0xFE, 0xFE, 0xFE, 0xFE, 0x7E}, // snake moving down from right (19)
		{0x00, 0x7F, 0x7F, 0x7F, 0x7F, 0x7E}, // snake moving right from down (20)
		{0x7E, 0x7F, 0x7F, 0x7F, 0x7F, 0x00}, // snake moving up from left (21)
		{0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, // snake moving up from down or down from up (22)

};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void SPI_write(unsigned char data);
void GLCD_data_write(unsigned char data);
void GLCD_command_write(unsigned char data);
void GLCD_init(void);
void GLCS_setCursor(unsigned char x, unsigned char y);
void GLCD_clear(void);
void GLCD_putchar(int font_table_row);

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
  /* USER CODE BEGIN 2 */
  GLCD_init();  // initialize the screen
  GLCD_clear(); // clear the screen

  //GLCD_putchar(4); //h
  //GLCD_putchar(5); //e
  //GLCD_putchar(6); //l
  //GLCD_putchar(6); //l
  //GLCD_putchar(7); //0
  //GLCD_putchar(0); //
  //GLCD_putchar(8); //w
  //GLCD_putchar(7); //o
  //GLCD_putchar(9); //r
  //GLCD_putchar(6); //l
  //GLCD_putchar(10);//d
  //GLCD_putchar(11);//!
  //GLCD_putchar(12);
  //GLCD_putchar(13);


  //start with snake head
  GLCD_putchar(14);
  HAL_Delay(500);
  GLCD_clear();

  //snake moves to x=5
  GLCD_putchar(15);
  GLCD_putchar(16);
  HAL_Delay(500);
  GLCD_putchar(16);
  HAL_Delay(500);
  GLCD_putchar(16);
  HAL_Delay(500);
  GLCD_putchar(16);
  HAL_Delay(500);
  GLCD_putchar(16);
  HAL_Delay(500);

  //snake moves down
  GLCD_setCursor(30,0);
  GLCD_putchar(17);
  GLCD_setCursor(30,1);
  GLCD_putchar(18);
  HAL_Delay(500);

  //snake moves to (1,0)
  GLCD_setCursor(25,1);
  GLCD_putchar(16);
  HAL_Delay(500);
  GLCD_setCursor(20,1);
  GLCD_putchar(16);
  HAL_Delay(500);
  GLCD_setCursor(15,1);
  GLCD_putchar(16);
  HAL_Delay(500);
  GLCD_setCursor(10,1);
  GLCD_putchar(16);
  HAL_Delay(500);
  GLCD_setCursor(5,1);
  GLCD_putchar(16);
  HAL_Delay(500);
  GLCD_setCursor(0,1);
  GLCD_putchar(16);
  HAL_Delay(500);

  //snake moves down
  GLCD_setCursor(0,1);
  GLCD_putchar(19);
  GLCD_setCursor(0,2);
  GLCD_putchar(18);
  HAL_Delay(500);

  //snake moves to x=6
  GLCD_setCursor(0,2);
  GLCD_putchar(20);
  GLCD_putchar(16);
  HAL_Delay(500);
  GLCD_putchar(16);
  HAL_Delay(500);
  GLCD_putchar(16);
  HAL_Delay(500);
  GLCD_putchar(16);
  HAL_Delay(500);
  GLCD_putchar(16);
  HAL_Delay(500);
  GLCD_putchar(15);

  //snake moves up to y=0
  GLCD_setCursor(35,2);
  GLCD_putchar(21);
  GLCD_setCursor(35,1);
  GLCD_putchar(22);
  HAL_Delay(500);
  GLCD_setCursor(35,0);
  GLCD_putchar(22);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void GLCD_putchar(int font_table_row){
	int i;
	for (i=0; i<6; i++){
		GLCD_data_write(font_table[font_table_row][i]);
	}
}

void SPI_write(unsigned char data){
	// Chip Enable (low is asserted)
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_RESET);


	// Send data over SPI1
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &data, 1, HAL_MAX_DELAY);

	// Chip Disable
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_SET);
}

void GLCD_data_write(unsigned char data){
	//Switch to "data" mode (D/C pin high)
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);

	// Send data over SPI
	SPI_write(data);
}

void GLCD_command_write(unsigned char data){
	//Switch to "command" mode (D/C pin low)
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_RESET);

	// Send data over SPI
	SPI_write(data);
}

void GLCD_init(void){

	// Keep CE high when not transmitting
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_SET);

	//Reset the screen (low pulse - down and up)
	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_SET);

	//Configure the screen according to the datasheet
	GLCD_command_write(0x21); //enter extended command mode
	GLCD_command_write(0xB4); //Set LCD Vop for contrast (this may be adjusted)
	GLCD_command_write(0x04); //set temp coefficient
	GLCD_command_write(0x12); //set LCD bias mode (this may be adjusted)
	GLCD_command_write(0x20); //return to normal command mode
	GLCD_command_write(0x0C); //set display mode normal
}

void GLCD_setCursor(unsigned char x, unsigned char y){
	GLCD_command_write(0x80 | x);	//column
	GLCD_command_write(0x40 | y);	//bank
}

void GLCD_clear(void){
	int i;
	for(i = 0; i < (GLCD_WIDTH * NUM_BANKS); i++){
		GLCD_data_write(0x00); //write zeros
	}
	GLCD_setCursor(0,0);	//return cursor to top left
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
