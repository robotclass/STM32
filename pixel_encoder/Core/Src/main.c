/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define LED_Q 16
#define LED_SHIFT 4

#define SRC_ENC 0
#define SRC_POT 0

#define MODE_FLOOD 0
#define MODE_LEVEL 1
#define MODE_POINT 2

#define CMD_GET_EPOS		0xBA // получение позиции энкодера 0..15
#define CMD_GET_PPOS		0xBB // получение позиции потенциометра 0..4095
#define CMD_SET_MODE		0xCA // установка режима
#define CMD_SET_COLOR		0xCB // установка цвета
#define CMD_SET_BRIGHTNESS	0xCC // установка яркости
#define CMD_SET_SRC			0xCD // выбор источника сигнала
#define CMD_RESET			0xCE // сброс счётчика
#define CMD_RUN_TEST		0xCF // запуск RGB теста

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
struct pixel_color{
	uint8_t red,green,blue;
};

struct pixel_color pixels[LED_Q];

uint8_t current_brightness = 5;
struct pixel_color current_color = {255,0,0};
uint8_t current_mode = MODE_LEVEL;
uint8_t current_src = SRC_ENC;

uint8_t i2c_recv_buf[8];
uint8_t i2c_send_buf[8];
uint8_t packet_size = 0;

volatile uint8_t flag = 0;
volatile uint8_t transferDirection, transferRequested, i2c_data_ready;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t getHWIdx(uint8_t i);
void updateLeds();
void setPixelColor(uint8_t led, struct pixel_color color);
void setPixelColorRGB(uint8_t led, uint8_t r, uint8_t g, uint8_t b);
void testRGB();
void handleEncoder();
void packInt(uint8_t target[], volatile uint16_t source[], uint8_t ssize, uint8_t tstart);
void handleI2C();
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
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // запуск энкодера
  HAL_I2C_EnableListen_IT(&hi2c1); // запуск прерываний I2C

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  setPixelColor(LED_SHIFT, current_color);
  updateLeds();

  while (1)
  {
	  handleEncoder();
	  handleI2C();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 66;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 63;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CI_Pin|DI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CI_Pin DI_Pin */
  GPIO_InitStruct.Pin = CI_Pin|DI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void init(){
	HAL_GPIO_WritePin(CI_GPIO_Port, CI_Pin, RESET);
	HAL_GPIO_WritePin(DI_GPIO_Port, DI_Pin, RESET);
}

void transfer(uint8_t b){
	for (int8_t i = 7; i>=0; i--){
		HAL_GPIO_WritePin(DI_GPIO_Port, DI_Pin , (b >> i & 1));
		HAL_GPIO_WritePin(CI_GPIO_Port, CI_Pin, SET);
		HAL_GPIO_WritePin(CI_GPIO_Port, CI_Pin, RESET);
	}
}

// �?нициализация шины данных для светодиодов APA и отправка начального блока данных
//
void startFrame()
{
  init();
  transfer(0);
  transfer(0);
  transfer(0);
  transfer(0);
}

// Отправка конечного блока данных в шину APA
//
void endFrame(uint16_t count){
  // оптимизация
  // вместо отправки 4x 0xFF согласно спецификации
  // отправляем 0x0 в цикле

  for (uint16_t i = 0; i < (count + 14)/16; i++) transfer(0);
  init(); 	// сброс сигнальных линий
}

uint8_t getHWIdx(uint8_t i){
	uint8_t hwi;
	hwi = 16 - i; // инверсия циферблата
	hwi = hwi + LED_SHIFT; // сдвиг циферблата
	if(hwi > 15)
		hwi -= 16;
	return hwi;
}

// Отправка данных о цвете пикселей в шину APA
//
void updateLeds(){
  startFrame();
  uint8_t hwi;
  for(uint16_t i = 0; i < LED_Q; i++) {
	  hwi = getHWIdx(i);
	  transfer(0b11100000 | current_brightness); // яркость - 5 разрядное число от 0 до 31
	  transfer(pixels[hwi].blue);
	  transfer(pixels[hwi].green);
	  transfer(pixels[hwi].red);
  }
  endFrame(LED_Q);
}
void clearLeds(){
	for(uint16_t i = 0; i < LED_Q; i++) {
		pixels[i].red = 0;
		pixels[i].green = 0;
		pixels[i].blue = 0;
	}
	updateLeds();
}

void setPixelColorRGB(uint8_t led, uint8_t r, uint8_t g, uint8_t b){
	pixels[led].red	= r;
	pixels[led].green = g;
	pixels[led].blue = b;
}

void setPixelColor(uint8_t led, struct pixel_color color){
	pixels[led].red	= color.red;
	pixels[led].green = color.green;
	pixels[led].blue = color.blue;
}

void testRGB(){
	clearLeds();
	HAL_Delay(500);

	// закрашивание всех светодиодов последовательно в красный, зелёный и синий
	struct pixel_color rgb[3] = {
		{255,0,0},
		{0,255,0},
		{0,0,255}
	};
	for(uint8_t c = 0; c < 3; c++) {
		for(uint8_t i = 0; i < LED_Q; i++) {
			setPixelColor(i, rgb[c]);
		}
		updateLeds();
		HAL_Delay(500);
	}
	clearLeds();
}

void handleEncoder(){
	uint8_t enc_pos = (__HAL_TIM_GET_COUNTER(&htim3) >> 2 ) % 16;
	if( current_mode == MODE_FLOOD ){
		for(uint8_t i = 0; i < LED_Q; i++) {
			if( i <= enc_pos )
				setPixelColor(i, current_color);
			else
				setPixelColorRGB(i, 0,0,0);
		}
		updateLeds();
	}
	else if( current_mode == MODE_LEVEL ){
		for(uint8_t i = 0; i < LED_Q; i++) {
			if( i <= enc_pos ){
				setPixelColorRGB(i, enc_pos*17, 255-enc_pos*17, 0);
			} else {
				setPixelColorRGB(i, 0,0,0);
			}
		}
		updateLeds();
	}
	else if( current_mode == MODE_POINT ){
		for(uint8_t i = 0; i < LED_Q; i++) {
			if( i == enc_pos ){
				setPixelColor(i, current_color);
			} else {
				setPixelColorRGB(i, 0,0,0);
			}
		}
		updateLeds();
	}
}

void packInt(uint8_t target[], volatile uint16_t source[], uint8_t ssize, uint8_t tstart){
	for(uint8_t i=0; i<ssize; i++){
		target[tstart+i*2] = source[i] & 0xFF;
		target[tstart+i*2+1] = source[i]>>8;
	}
}

void handleI2C(){
	if(i2c_data_ready){
		i2c_data_ready = 0;

		switch(i2c_recv_buf[0]) {
    		case CMD_GET_EPOS:;
    			uint8_t enc_pos = (__HAL_TIM_GET_COUNTER(&htim3) >> 2 ) % 16;
    			i2c_send_buf[0] = enc_pos;
    			break;
    		case CMD_GET_PPOS:
    			/*
    			HAL_ADC_Stop_DMA(&hadc);

    			packInt(i2c_send_buf, adc_buff, 2, 0);

    			HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_buff, 2);
    			*/
    			break;
    		case CMD_SET_MODE:
    			current_mode = i2c_recv_buf[1];
    			break;
    		case CMD_SET_COLOR:
    			current_color.red = i2c_recv_buf[1];
    			current_color.green = i2c_recv_buf[2];
    			current_color.blue = i2c_recv_buf[3];
    			break;
    		case CMD_SET_BRIGHTNESS:
    			current_brightness = i2c_recv_buf[1];
    			break;
    		case CMD_SET_SRC:
    			current_src = i2c_recv_buf[1];
    			break;
    		case CMD_RUN_TEST:
    			testRGB();
    			break;
    		case CMD_RESET:
    			__HAL_TIM_SET_COUNTER(&htim3, 0);
    			break;
    		default:
    			i2c_send_buf[0] = 0xFF;
    			packet_size = 1;
		}
	}

}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
	UNUSED(AddrMatchCode);

	if(hi2c->Instance == I2C1) {
		transferRequested = 1;
		transferDirection = TransferDirection;

		if(transferDirection == TRANSFER_DIR_WRITE) {
			HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, i2c_send_buf, 4, I2C_LAST_FRAME);
		} else {
			HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, i2c_recv_buf, 4, I2C_FIRST_FRAME);
		}
	}
}
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c){
	transferRequested = 0;
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
	transferRequested = 0;
	i2c_data_ready = 1;
}
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c){
	HAL_I2C_EnableListen_IT(hi2c);
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
