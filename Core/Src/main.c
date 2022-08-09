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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include <stdio.h>
#include <string.h>
#include "ssd1306.h"
#include <stdbool.h>
#include "Password.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PASSWORD_COUNT 5
#define PASSWORD_LENGHT 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t print_password;
volatile uint8_t moveRight;
volatile uint8_t moveLeft;

volatile uint8_t enc_status, enc_last_status;
volatile uint32_t timer_click, timer_last_click;

extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct{
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
	uint8_t KEYCODE6;
}KeyboardHID;

KeyboardHID keybHID = {0, 0, 0, 0, 0, 0, 0, 0};

extern FontDef Font_6x8;
extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
void print_char(char *buff, uint16_t size);
void print_HID(uint8_t nomer);
void draw_disp(uint8_t * mode);
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
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Start", Font_16x26, White);
  ssd1306_UpdateScreen();
  HAL_Delay(300);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  static uint8_t mode = 0;
	  static uint32_t timer_refresh = 0;

	  if(HAL_GetTick() - timer_refresh > 30){
		  timer_refresh = HAL_GetTick();
		  draw_disp(&mode);
	  }
	  if (moveLeft > 0){
		  if (mode == 0){
			  mode = PASSWORD_COUNT - 1;
			  moveLeft--;
		  }else{
			  mode--;
			  moveLeft--;
		  }
	  }
	  if (moveRight > 0){
		  mode++;
		  if (mode == PASSWORD_COUNT) mode = 0;
		  moveRight--;
	  }
	  if (print_password == 1){
		  print_HID(mode);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 4800-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_CH1_Pin ENC_BUTTON_Pin */
  GPIO_InitStruct.Pin = ENC_CH1_Pin|ENC_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_CH2_Pin */
  GPIO_InitStruct.Pin = ENC_CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_CH2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == ENC_BUTTON_Pin){
		print_password = 1;
	}
	if(GPIO_Pin == ENC_CH2_Pin){
		/*
		if(move_enc > 0){
			moveLeft += move_enc;
			move_enc = 0;
		}else{
			move_enc++;
		}
		*/
		/*
		if (!HAL_GPIO_ReadPin(ENC_CH1_GPIO_Port, ENC_CH1_Pin) && !HAL_GPIO_ReadPin(ENC_CH2_GPIO_Port, ENC_CH2_Pin)){
			enc_status  = 0x00;
		}else if (HAL_GPIO_ReadPin(ENC_CH1_GPIO_Port, ENC_CH1_Pin) && !HAL_GPIO_ReadPin(ENC_CH2_GPIO_Port, ENC_CH2_Pin)){
			enc_status  = 0x10;
		}else if (!HAL_GPIO_ReadPin(ENC_CH1_GPIO_Port, ENC_CH1_Pin) && HAL_GPIO_ReadPin(ENC_CH2_GPIO_Port, ENC_CH2_Pin)){
			enc_status  = 0x01;
		}else if (HAL_GPIO_ReadPin(ENC_CH1_GPIO_Port, ENC_CH1_Pin) && HAL_GPIO_ReadPin(ENC_CH2_GPIO_Port, ENC_CH2_Pin)){
			enc_status  = 0x11;
		}
		*/
		enc_status = 0x00|((HAL_GPIO_ReadPin(ENC_CH1_GPIO_Port, ENC_CH1_Pin) << 4) | (HAL_GPIO_ReadPin(ENC_CH2_GPIO_Port, ENC_CH2_Pin)));
		if ((enc_last_status == 0x10 && enc_status == 0x11) || (enc_last_status == 0x01 && enc_status == 0x00)){
			timer_click = HAL_GetTick();
			moveRight++;
			timer_last_click = timer_click;
		}



		if (enc_last_status == 0x11 && enc_status == 0x10) {
			enc_last_status = 0x10;
		}else if (enc_last_status == 0x00 && enc_status == 0x01) {
			enc_last_status = 0x01;
		}else if ((enc_last_status == 0x10 && enc_status == 0x00) || (enc_last_status == 0x01 && enc_status == 0x11)){
			timer_click = HAL_GetTick();
			moveLeft++;
			timer_last_click = timer_click;
		}

		enc_last_status = enc_status;
	}
	if(GPIO_Pin == ENC_CH1_Pin){
		/*
		if(move_enc > 0){
			moveRight += move_enc;
			move_enc = 0;
		}else{
			move_enc++;
		}
		*/
		/*
		if (!HAL_GPIO_ReadPin(ENC_CH1_GPIO_Port, ENC_CH1_Pin) && !HAL_GPIO_ReadPin(ENC_CH2_GPIO_Port, ENC_CH2_Pin)){
			enc_status  = 0x00;
		}else if (HAL_GPIO_ReadPin(ENC_CH1_GPIO_Port, ENC_CH1_Pin) && !HAL_GPIO_ReadPin(ENC_CH2_GPIO_Port, ENC_CH2_Pin)){
			enc_status  = 0x10;
		}else if (!HAL_GPIO_ReadPin(ENC_CH1_GPIO_Port, ENC_CH1_Pin) && HAL_GPIO_ReadPin(ENC_CH2_GPIO_Port, ENC_CH2_Pin)){
			enc_status  = 0x01;
		}else if (HAL_GPIO_ReadPin(ENC_CH1_GPIO_Port, ENC_CH1_Pin) && HAL_GPIO_ReadPin(ENC_CH2_GPIO_Port, ENC_CH2_Pin)){
			enc_status  = 0x11;
		}
		*/
		enc_status = 0x00|((HAL_GPIO_ReadPin(ENC_CH1_GPIO_Port, ENC_CH1_Pin) << 4) | (HAL_GPIO_ReadPin(ENC_CH2_GPIO_Port, ENC_CH2_Pin)));

		if ((enc_last_status == 0x10 && enc_status == 0x11) || (enc_last_status == 0x01 && enc_status == 0x00)){
			timer_click = HAL_GetTick();
			moveRight++;
			timer_last_click = timer_click;
		}



		if (enc_last_status == 0x11 && enc_status == 0x10) {
			enc_last_status = 0x10;
		}else if (enc_last_status == 0x00 && enc_status == 0x01) {
			enc_last_status = 0x01;
		}else if ((enc_last_status == 0x10 && enc_status == 0x00) || (enc_last_status == 0x01 && enc_status == 0x11)){
			timer_click = HAL_GetTick();
			moveLeft++;
			timer_last_click = timer_click;
		}

		enc_last_status = enc_status;
	}
	if (GPIO_Pin == GPIO_PIN_0){
		moveRight++;
	}

}
void print_HID(uint8_t nomer){

	 ssd1306_Fill(Black);
	 ssd1306_SetCursor(0, 0);
	 ssd1306_WriteString("Button", Font_7x10, White);
	 ssd1306_SetCursor(0, 11);
	 ssd1306_WriteString("pressed", Font_7x10, White);
	 ssd1306_UpdateScreen();

	 print_password = 0;
	 char login_print[30];
	 for(uint8_t i = 0; i < 30; i++){
		 login_print[i] = login[nomer][i];
	 }
	 print_char(login_print, sizeof(login_print));

	 keybHID.KEYCODE1 = 0x2B;
	 USBD_HID_SendReport(&hUsbDeviceFS, &keybHID, sizeof(keybHID));

	 HAL_Delay(50);

	 keybHID.KEYCODE1 = 0x00;
	 USBD_HID_SendReport(&hUsbDeviceFS, &keybHID, sizeof(keybHID));
	 HAL_Delay(50);

	 char password_print[PASSWORD_LENGHT];
	 for(uint8_t i = 0; i < PASSWORD_LENGHT; i++){
	 		 password_print[i] = password[nomer][i];
	 }
	 print_char(password_print, sizeof(password_print));
}

void print_char(char *buff, uint16_t size){

	for (uint16_t i = 0; i < size; i++){
		uint8_t nom_of_symbol = 0;
		uint8_t shift = 0;
		/*=====Big letter=========*/
		if ((buff[i] ) >= 65 && buff[i] <= 90){
			shift = 1;
			uint8_t nom_of_letter = buff[i] - 65;
			nom_of_symbol = nom_of_letter + 4;
		}
		/*======Small letter======*/
		if (buff[i] >= 97 && buff[i] <= 122){
			shift = 0;
			uint8_t nom_of_letter =	buff[i] - 97;
			nom_of_symbol = nom_of_letter + 4;
		}
		/*========Numbers=========*/
		if (buff[i] >= 49 && buff[i] <= 57){
			shift = 0;
			uint8_t nom_of_letter =	buff[i] - 49;
			nom_of_symbol = nom_of_letter + 30;
		}
		switch (buff[i]){
		case ' ':{shift = 0; nom_of_symbol = 0x2C;}break;
		case '!':{shift = 1; nom_of_symbol = 30;}break;
		case '@':{shift = 1; nom_of_symbol = 31;}break;
		case '#':{shift = 1; nom_of_symbol = 32;}break;
		case '$':{shift = 1; nom_of_symbol = 33;}break;
		case '%':{shift = 1; nom_of_symbol = 34;}break;
		case '^':{shift = 1; nom_of_symbol = 35;}break;
		case '&':{shift = 1; nom_of_symbol = 36;}break;
		case '*':{shift = 1; nom_of_symbol = 37;}break;
		case '(':{shift = 1; nom_of_symbol = 38;}break;
		case ')':{shift = 1; nom_of_symbol = 39;}break;
		case '.':{shift = 0; nom_of_symbol = 55;}break;
		case '0':{shift = 0; nom_of_symbol = 39;}break;
		}

		char uart_buff[100];
		sprintf(uart_buff, "Char: %c shift: %d, code: %d\r\n", buff[i], shift, nom_of_symbol);
		HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), 1000);

		if (shift == 1) keybHID.MODIFIER = 0x02;
		keybHID.KEYCODE1 = nom_of_symbol;
		USBD_HID_SendReport(&hUsbDeviceFS, &keybHID, sizeof(keybHID));
		HAL_Delay(15);
		if (shift == 1) keybHID.MODIFIER = 0x00;
		keybHID.KEYCODE1 = 0x00;
		USBD_HID_SendReport(&hUsbDeviceFS, &keybHID, sizeof(keybHID));
		HAL_Delay(15);
	}
}

void draw_disp(uint8_t *mode){
	ssd1306_Fill(Black);
	char account_name [15];
	for (uint8_t i = 0; i < 15; i++){
		account_name[i] = name_of_account[*mode][i];
	}
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(account_name, Font_11x18, White);

	/*
	switch (*mode){
	case 0: {
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("Choose password", Font_7x10, White);
	}break;
	case 1: {
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("password 1", Font_7x10, White);
	}break;
	case 2: {
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("password 2", Font_7x10, White);
	}break;
	case 3: {
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("password 3", Font_7x10, White);
	}break;
	case 4: {
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("password 4", Font_7x10, White);
	}break;
	case 5: {
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("password 5", Font_7x10, White);
	}break;
	default: *mode = 1;
	}
	*/
	char buff[16];
	ssd1306_SetCursor(0, 22);
	sprintf(buff, "Menu: %d", *mode);
	ssd1306_WriteString(buff, Font_6x8, White);
	ssd1306_UpdateScreen();
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
