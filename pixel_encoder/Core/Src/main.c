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
#define VERSION 2

#define LED_Q 16
#define LED_MAX LED_Q-1
#define LED_SHIFT 4

#define DEF_BRIGHTNESS 5
#define DEF_COLOR_R 255
#define DEF_COLOR_G 0
#define DEF_COLOR_B 0

#define SRC_ENC 0
#define SRC_POT 1

#define ENC_STOP_MAX 1
#define ENC_STOP_MIN 2
#define ENC_BTN_TO 200

#define MODE_FLOOD 0
#define MODE_LEVEL 1
#define MODE_POINT 2
#define MODE_TRAIL 3

#define CMD_GET_VERSION		0xB0 // версия прошивки
#define CMD_GET_SRC			0xB1 // получение источника сигнала 0 - энкодер, 1 - потенциометр
#define CMD_GET_STATE		0xB2 // получение состояния: байты 1 и 2 - позиция, байт 3 - состояние кнопки
#define CMD_SET_MODE		0xC0 // установка режима
#define CMD_SET_COLOR		0xC1 // установка цвета
#define CMD_SET_BRIGHTNESS	0xC2 // установка яркости
#define CMD_SET_POT_LPF		0xC3 // установка коэффициента ФНЧ для потенциометра 1..15
#define CMD_SET_ENC_LIMIT	0xC4 // установка предела энкодера
#define CMD_SET_ENC_MAX		0xC5 // установка диапазона энкодера
#define CMD_INIT			0xE0 // инициализация счётчика
#define CMD_RESET			0xE1 // сброс счётчика
#define CMD_TEST			0xE2 // запуск RGB теста

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
struct pixel_color{
	uint8_t red,green,blue;
};

struct pixel_color pixels[LED_Q];
struct pixel_color trail_pixels[3];

uint8_t current_brightness = DEF_BRIGHTNESS;
struct pixel_color current_color = {DEF_COLOR_R, DEF_COLOR_G, DEF_COLOR_B};
uint8_t current_mode = MODE_LEVEL;
uint8_t current_src = SRC_ENC;

// i2c
uint8_t i2c_recv_buf[8];
uint8_t i2c_send_buf[8];

// enc
uint16_t enc_pos = 0;
uint8_t enc_limit = 0;
uint8_t enc_max = 0;
uint8_t enc_stop = 0;
volatile uint32_t enc_btn_next = 0;

// pot
uint16_t pot_adc;
uint32_t pot_lpf = 0;
uint16_t pot_lpf_k = 1024;
volatile uint8_t flag_adc = 0;

//volatile uint8_t flag = 0;
volatile uint8_t transferDirection, transferRequested, i2c_data_ready;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t getHWIdx(uint8_t i);
void updateLeds();
void setPixelColor(uint8_t led, struct pixel_color color);
void setPixelColorRGB(uint8_t led, uint8_t r, uint8_t g, uint8_t b);
void testRGB();
void initPot( uint8_t init );
void handlePot();
void initEnc( uint8_t init );
void handleEnc();
void draw( uint8_t pos );
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_I2C_EnableListen_IT(&hi2c1); // запуск прерываний I2C

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  setPixelColor(LED_SHIFT, current_color);
  updateLeds();

  if( HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) ){
	  current_src = SRC_POT;
	  HAL_ADCEx_Calibration_Start(&hadc); // калибровка АЦП
	  HAL_ADC_Start_DMA(&hadc, (uint32_t*)&pot_adc, 1); // запуск DMA
	  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4); // запуск таймера для опроса АЦП
	  initPot(1);
  } else {
	  current_src = SRC_ENC;
	  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL); // запуск энкодера
	  initEnc(1);
  }

  while (1)
  {
	  if( current_src == SRC_POT ){
		  handlePot();
	  } else
	  if( current_src == SRC_ENC ){
		  handleEnc();
	  }
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
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC4;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
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
  uint8_t address = 0x30;
  if( HAL_GPIO_ReadPin(SW1_GPIO_Port, SW2_Pin) ){
	  address |= 0b00000001;
  }
  if( HAL_GPIO_ReadPin(SW1_GPIO_Port, SW3_Pin) ){
	  address |= 0b00000010;
  }

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = address<<1;//66;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4800-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 200-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim3.Init.Period = 65535;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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

  /*Configure GPIO pins : SW2_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW2_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW3_Pin */
  GPIO_InitStruct.Pin = SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SW3_GPIO_Port, &GPIO_InitStruct);

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

// инициализация шины данных для светодиодов APA и отправка начального блока данных
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

	if( current_src == SRC_ENC ){
		draw(0);
	}
}

void initPot( uint8_t init ){
	if( init ){
		current_brightness = DEF_BRIGHTNESS;
		current_color.red = DEF_COLOR_R;
		current_color.green = DEF_COLOR_G;
		current_color.blue = DEF_COLOR_B;
		current_mode = MODE_LEVEL;
	}
}

void handlePot(){
	if( flag_adc ){
		flag_adc = 0;
		HAL_ADC_Stop_DMA(&hadc);
		uint32_t fp_pot = (4095-pot_adc)<<10;
		HAL_ADC_Start_DMA(&hadc, (uint32_t*)&pot_adc, 1);

		pot_lpf = pot_lpf - ((pot_lpf*pot_lpf_k)>>10) + ((fp_pot*pot_lpf_k)>>10);
		draw( pot_lpf>>18 ); //10+8
	}
}

void initEnc( uint8_t init ){
	if( init ){
		enc_limit = 0;
		enc_max = 0;
		enc_stop = 0;
		current_brightness = DEF_BRIGHTNESS;
		current_color.red = DEF_COLOR_R;
		current_color.green = DEF_COLOR_G;
		current_color.blue = DEF_COLOR_B;
		current_mode = MODE_LEVEL;
	}
	enc_pos = 0;
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	draw(0);
}

void handleEnc(){

	// вышло время лага кнопки
	if( enc_btn_next && HAL_GetTick() > enc_btn_next ){
		enc_btn_next = 0;
	}

	uint16_t enc_pos_new = __HAL_TIM_GET_COUNTER(&htim3);

	// если значения не изменились
	if( enc_pos_new == enc_pos ){
		return;
	}

	// игнорировать значения некратные 4
	if( enc_pos_new & 0b00000011 ){
		return;
	}

	uint8_t reverse = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
	uint16_t enc_max_v = ((LED_Q)<<(2+enc_max))-4;

	if( enc_limit && enc_stop == ENC_STOP_MIN ){
		if( reverse ){
			return;
		} else {
			enc_stop = 0;
			enc_pos = 0;
			__HAL_TIM_SET_COUNTER(&htim3, enc_pos);
			draw( 0 );
			return;
		}
	}

	if( enc_limit && enc_stop == ENC_STOP_MAX ){
		if( reverse ){
			enc_stop = 0;
			enc_pos = enc_max_v;
			__HAL_TIM_SET_COUNTER(&htim3, enc_pos);
			draw( enc_pos>>(2+enc_max) );
			return;
		} else {
			return;
		}
	}

	if( enc_pos_new > enc_max_v ){
		if( enc_limit && !enc_stop ){
			if( reverse ){
				enc_pos = 0;
				enc_stop = ENC_STOP_MIN;
			} else {
				enc_pos = enc_max_v;
				enc_stop = ENC_STOP_MAX;
			}
		} else {
			if( reverse ){
				enc_pos = enc_max_v;
				__HAL_TIM_SET_COUNTER(&htim3, enc_max_v);
			} else {
				enc_pos = 0;
				__HAL_TIM_SET_COUNTER(&htim3, enc_pos);
			}
		}
	} else {
		enc_pos = enc_pos_new;
	}

	//draw( (((enc_pos<<10)/enc_max)*LED_MAX)>>10 );
	draw( enc_pos>>(2+enc_max));
}

void draw( uint8_t pos ){
	if( current_mode == MODE_FLOOD ){
		for(uint8_t i = 0; i < LED_Q; i++) {
			if( i <= pos )
				setPixelColor(i, current_color);
			else
				setPixelColorRGB(i, 0,0,0);
		}
		updateLeds();
	}
	else if( current_mode == MODE_LEVEL ){
		for(uint8_t i = 0; i < LED_Q; i++) {
			if( i <= pos ){
				setPixelColorRGB(i, pos*17, 255-pos*17, 0);
			} else {
				setPixelColorRGB(i, 0,0,0);
			}
		}
		updateLeds();
	}
	else if( current_mode == MODE_POINT ){
		for(uint8_t i = 0; i < LED_Q; i++) {
			if( i == pos ){
				setPixelColor(i, current_color);
			} else {
				setPixelColorRGB(i, 0,0,0);
			}
		}
		updateLeds();
	}
}

void handleI2C(){
	if(i2c_data_ready){
		i2c_data_ready = 0;

		switch(i2c_recv_buf[0]) {
			case CMD_GET_VERSION:
				i2c_send_buf[0] = VERSION;
				break;
			case CMD_GET_SRC:
    			i2c_send_buf[0] = current_src;
				break;
    		case CMD_GET_STATE:
    			if( current_src == SRC_POT ){
        			i2c_send_buf[0] = (pot_lpf>>10) & 0xFF;
        			i2c_send_buf[1] = (pot_lpf>>18) & 0xFF;
    			} else
    			if( current_src == SRC_ENC ){
    				uint16_t enc_pos4 = enc_pos>>2;
        			i2c_send_buf[0] = enc_pos4 & 0xFF;
        			i2c_send_buf[1] = (enc_pos4>>8) & 0xFF;
        			i2c_send_buf[2] = !!enc_btn_next;
    			}
    			break;
    		case CMD_SET_MODE:;
    			uint8_t mode = i2c_recv_buf[1];
    			if( mode == MODE_FLOOD || mode == MODE_LEVEL || mode == MODE_POINT ){
    				current_mode = mode;
    			}
    			break;
    		case CMD_SET_COLOR:
    			current_color.red = i2c_recv_buf[1];
    			current_color.green = i2c_recv_buf[2];
    			current_color.blue = i2c_recv_buf[3];
    			draw( enc_pos>>(2+enc_max)); // перерисовка текущего значения
    			break;
    		case CMD_SET_BRIGHTNESS:
    			current_brightness = i2c_recv_buf[1];
    			draw( enc_pos>>(2+enc_max)); // перерисовка текущего значения
    			break;
    		case CMD_SET_POT_LPF:
    			pot_lpf_k = i2c_recv_buf[1]<<6;
    			break;
    		case CMD_SET_ENC_LIMIT:
    			enc_limit = i2c_recv_buf[1];
				initEnc(0);
    			break;
    		case CMD_SET_ENC_MAX:
    			enc_max = i2c_recv_buf[1];
				initEnc(0);
    			break;
    		case CMD_TEST:
    			testRGB();
    			break;
    		case CMD_INIT:
    			if( current_src == SRC_POT ){
    				initPot(1);
    			} else
    			if( current_src == SRC_ENC ){
    				initEnc(1);
				}
    			break;
    		case CMD_RESET:
    			if( current_src == SRC_POT ){
    				initPot(0);
    			} else
    			if( current_src == SRC_ENC ){
    				initEnc(0);
				}
    			break;
    		default:
    			i2c_send_buf[0] = 0xFF;
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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    if(hadc->Instance == ADC1){
        flag_adc = 1;
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==BTN_Pin) {
		enc_btn_next = HAL_GetTick() + ENC_BTN_TO;
	} else{
		__NOP();
	}
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
