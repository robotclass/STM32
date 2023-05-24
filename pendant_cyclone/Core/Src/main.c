/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#define NUM_OF_LEDS 8
#define SOFT_CLICK_TIME 200

#define RED 0
#define GREEN 1
#define BLUE 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct pixel_color{
	uint8_t red,green,blue,brightness;
};

uint8_t brightMode = 5;
uint8_t animationMode = 0;
uint8_t breakAnimation = 0;
uint32_t softClickTimer = 0;

struct pixel_color rgb[] = {
		{255,0,0,5},
		{0,255,0,5},
		{0,0,255,5}
};

struct pixel_color pixels[NUM_OF_LEDS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void init(){
	HAL_GPIO_WritePin(APA_CLK_GPIO_Port, APA_CLK_Pin, RESET);
	HAL_GPIO_WritePin(APA_DT_GPIO_Port, APA_DT_Pin, RESET);
}

void transfer(uint8_t b){
	for (int8_t i = 7; i>=0; i--){
		HAL_GPIO_WritePin(APA_DT_GPIO_Port, APA_DT_Pin , (b >> i & 1));
		HAL_GPIO_WritePin(APA_CLK_GPIO_Port, APA_CLK_Pin, SET);
		HAL_GPIO_WritePin(APA_CLK_GPIO_Port, APA_CLK_Pin, RESET);
	}
}

// Инициализация шины данных для светодиодов APA и отправка начального блока данных
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

// Отправка данных о цвете пикселей в шину APA
//
void updateLeds(){
  startFrame();
  for(uint16_t i = 0; i < NUM_OF_LEDS; i++) {
	  transfer(0b11100000 | pixels[i].brightness); // яркость - 5 разрядное число
	  transfer(pixels[i].blue);
	  transfer(pixels[i].green);
	  transfer(pixels[i].red);
}
  endFrame(NUM_OF_LEDS);
}

void clearLeds(){
	for(uint16_t i = 0; i < NUM_OF_LEDS; i++) {
		pixels[i].red = 0;
		pixels[i].green = 0;
		pixels[i].blue = 0;
		pixels[i].brightness = 0;
	}
	updateLeds();
}

void setPixelColor(uint8_t led, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness){
	pixels[led].red	= r;
	pixels[led].green = g;
	pixels[led].blue = b;
	pixels[led].brightness = brightness;
}

// анимация при запуске
//
void startupAnimation(){
	clearLeds();
	HAL_Delay(500);

	// закрашивание всех светодиодов последовательно в красный, зелёный и синий
	for(uint8_t c = 0; c < 3; c++) {
		for(uint8_t i = 0; i < NUM_OF_LEDS; i++) {
			pixels[i] = rgb[c];
		}
		updateLeds();
		HAL_Delay(500);
	}
}

// эффект радуга
//
void rainbowWheel(uint8_t WheelPos, uint8_t led, uint8_t brightness) {
	WheelPos = 255 - WheelPos;
		if(WheelPos < 85) {
			setPixelColor(led, 255 - WheelPos * 3, 0, WheelPos * 3, brightness);
			return;
		}
		if(WheelPos < 170) {
		  WheelPos -= 85;
		  setPixelColor(led, 0, WheelPos * 3, 255 - WheelPos * 3, brightness);
		  return;
		}
		WheelPos -= 170;
		setPixelColor(led, WheelPos * 3, 255 - WheelPos * 3, 0, brightness);
		return;
		}


void mode_0(){
	uint16_t i, j;

	for(j=0; j<256; j++) {
		for(i=0; i< NUM_OF_LEDS; i++) {
			rainbowWheel(((i * 256 / (NUM_OF_LEDS)) + j) & 255, i, brightMode);
		}
		updateLeds();
		if(breakAnimation){
			breakAnimation=0;
			clearLeds();
			break;
		}
		HAL_Delay(1);
	}
}

//обработчик прерываний от кнопок
//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//animationMode+=1;
	if(GPIO_Pin==GPIO_PIN_2){ 									// проверяем что это прерывание на пин_2 (наша кнопка-1)
		if((HAL_GetTick()-softClickTimer) > SOFT_CLICK_TIME){	// устранение дребезга контактов
			animationMode+=1;
			softClickTimer=HAL_GetTick();						// обновление таймера дребезга
		}
		breakAnimation=1;
	}

	if(GPIO_Pin==GPIO_PIN_1){ 									// проверяем что это прерывание на пин_2 (наша кнопка-1)
		if((HAL_GetTick()-softClickTimer) > SOFT_CLICK_TIME){	// устранение дребезга контактов
			brightMode+=12;
			if (brightMode>31) brightMode=5;
			softClickTimer=HAL_GetTick();						// обновление таймера дребезга
		}
	}
}

// эффект комета
//
void CometRed(uint8_t WheelPos, uint8_t led, uint8_t brightness) {
	WheelPos = 255 - WheelPos;
		if(WheelPos < 85) {
			setPixelColor(led, WheelPos * 3, 0, 0, brightness);
			return;
		}
		if(WheelPos < 170) {
		  WheelPos -= 85;
		  setPixelColor(led, 255 - WheelPos * 3, 0, 0, brightness);
		  return;
		}
		//WheelPos -= 170;
		//setPixelColor(led, 0, 255 - WheelPos * 3, 0, brightness);
		return;
}

void CometGreen(uint8_t WheelPos, uint8_t led, uint8_t brightness) {
	WheelPos = 255 - WheelPos;
		if(WheelPos < 85) {
			setPixelColor(led, 0,WheelPos * 3, 0,  brightness);
			return;
		}
		if(WheelPos < 170) {
		  WheelPos -= 85;
		  setPixelColor(led, 0,	255 - WheelPos * 3, 0,  brightness);
		  return;
		}
		//WheelPos -= 170;
		//setPixelColor(led, 0, 255 - WheelPos * 3, 0, brightness);
		return;
}

void CometBlue(uint8_t WheelPos, uint8_t led, uint8_t brightness) {
	WheelPos = 255 - WheelPos;
		if(WheelPos < 85) {
			setPixelColor(led, 0, 0, WheelPos * 3, brightness);
			return;
		}
		if(WheelPos < 170) {
		  WheelPos -= 85;
		  setPixelColor(led, 0, 0, 255 - WheelPos * 3, brightness);
		  return;
		}
		//WheelPos -= 170;
		//setPixelColor(led, 0, 255 - WheelPos * 3, 0, brightness);
		return;
}

/*
void mode_1(){
	uint16_t i, j;

	for(j=0; j<256; j++) {					// этот цикл создает эффект движения по кругу
		if(breakAnimation){
			breakAnimation=0;
			clearLeds();
			break;
		}
		for(i=0; i< NUM_OF_LEDS; i++) {		// этот цикл меняет позицию "стартового"
		  uint8_t WheelPos = ((i * 256 / (NUM_OF_LEDS))+ j) & 255;
		  Comet(WheelPos, i,brightMode);
	  }
	  updateLeds();
	  HAL_Delay(1);
	}

}
*/

void mode_1(){
	uint16_t i, j;

	for(j=0; j<256; j++) {
		if(breakAnimation){
			breakAnimation=0;
			clearLeds();
			return;
		}
		for(i=0; i< NUM_OF_LEDS; i++) {		// этот цикл меняет позицию "стартового"
		  uint8_t WheelPos = ((i * 64 / (NUM_OF_LEDS))+ j) & 255;
		  CometRed(WheelPos, i,brightMode);
	  }
	  updateLeds();
	  HAL_Delay(1);
	}

}

void mode_2(){
	uint16_t i, j;

	for(j=0; j<256; j++) {					// этот цикл создает эффект движения по кругу
		if(breakAnimation){
			breakAnimation=0;
			clearLeds();
			return;
		}
		for(i=0; i< NUM_OF_LEDS; i++) {		// этот цикл меняет позицию "стартового"
		  uint8_t WheelPos = ((i * 64 / (NUM_OF_LEDS))+ j) & 255;
		  CometGreen(WheelPos, i,brightMode);
	  }
	  updateLeds();
	  HAL_Delay(1);
	}

}

void mode_3(){
	uint16_t i, j;

	for(j=0; j<256; j++) {					// этот цикл создает эффект движения по кругу
		if(breakAnimation){
			breakAnimation=0;
			clearLeds();
			return;
		}
		for(i=0; i< NUM_OF_LEDS; i++) {		// этот цикл меняет позицию "стартового"
		  uint8_t WheelPos = ((i * 64 / (NUM_OF_LEDS))+ j) & 255;
		  CometBlue(WheelPos, i,brightMode);
	  }
	  updateLeds();
	  HAL_Delay(1);
	}

}

void mode_4(){
	uint16_t i, j;

	for(j=0; j<256; j++) {					// этот цикл создает эффект движения по кругу
		if(breakAnimation){
			breakAnimation=0;
			clearLeds();
			return;
		}
		for(i=0; i< NUM_OF_LEDS; i++) {		// этот цикл меняет позицию "стартового"
		  uint8_t WheelPos = ((i * 1024 / (NUM_OF_LEDS))+ j) & 255; // интересный эффект 2
		  CometRed(WheelPos, i,brightMode);
	  }
	  updateLeds();
	  HAL_Delay(1);
	}

}

void mode_5(){
	uint16_t i, j;

	for(j=0; j<256; j++) {					// этот цикл создает эффект движения по кругу
		if(breakAnimation){
			breakAnimation=0;
			clearLeds();
			return;
		}
		for(i=0; i< NUM_OF_LEDS; i++) {		// этот цикл меняет позицию "стартового"
		  uint8_t WheelPos = ((i * 1024 / (NUM_OF_LEDS))+ j) & 255; // интересный эффект 2
		  CometGreen(WheelPos, i,brightMode);
	  }
	  updateLeds();
	  HAL_Delay(1);
	}

}

void mode_6(){
	uint16_t i, j;

	for(j=0; j<256; j++) {					// этот цикл создает эффект движения по кругу
		if(breakAnimation){
			breakAnimation=0;
			clearLeds();
			return;
		}
		for(i=0; i< NUM_OF_LEDS; i++) {		// этот цикл меняет позицию "стартового"
		  uint8_t WheelPos = ((i * 1024 / (NUM_OF_LEDS))+ j) & 255; // интересный эффект 2
		  CometBlue(WheelPos, i,brightMode);
	  }
	  updateLeds();
	  HAL_Delay(1);
	}

}

void mode_7(){			//красное мерцание циферблата
	uint16_t i, j;

	for(j=0; j<256; j++) {					// этот цикл создает эффект движения по кругу
		if(breakAnimation){
			breakAnimation=0;
			clearLeds();
			return;
		}
		for(i=0; i< NUM_OF_LEDS; i++) {		// этот цикл меняет позицию "стартового"
			setPixelColor(
					i,
					j,
					0,
					0,
					brightMode
			);
	  }
	  updateLeds();
	  HAL_Delay(1);
	}
	for(j=255; j>0; j--) {					// этот цикл создает эффект движения по кругу
			if(breakAnimation){
				breakAnimation=0;
				clearLeds();
				return;
			}
			for(i=0; i< NUM_OF_LEDS; i++) {		// этот цикл меняет позицию "стартового"
				setPixelColor(
						i,
						j,
						0,
						0,
						brightMode
				);
		  }
	  updateLeds();
	  HAL_Delay(1);
	}

}

void mode_8(){			//зеленое мерцание циферблата
	uint16_t i, j;

	for(j=0; j<256; j++) {					// этот цикл создает эффект движения по кругу
		if(breakAnimation){
			breakAnimation=0;
			clearLeds();
			return;
		}
		for(i=0; i< NUM_OF_LEDS; i++) {		// этот цикл меняет позицию "стартового"
			setPixelColor(
					i,
					0,
					j,
					0,
					brightMode
			);
	  }
	  updateLeds();
	  HAL_Delay(1);
	}
	for(j=255; j>0; j--) {					// этот цикл создает эффект движения по кругу
			if(breakAnimation){
				breakAnimation=0;
				clearLeds();
				return;
			}
			for(i=0; i< NUM_OF_LEDS; i++) {		// этот цикл меняет позицию "стартового"
				setPixelColor(
						i,
						0,
						j,
						0,
						brightMode
				);
		  }
	  updateLeds();
	  HAL_Delay(1);
	}

}

void mode_9(){			//синее мерцание циферблата
	uint16_t i, j;

	for(j=0; j<256; j++) {					// этот цикл создает эффект движения по кругу
		if(breakAnimation){
			breakAnimation=0;
			clearLeds();
			return;
		}
		for(i=0; i< NUM_OF_LEDS; i++) {		// этот цикл меняет позицию "стартового"
			setPixelColor(
					i,
					0,
					0,
					j,
					brightMode
			);
	  }
	  updateLeds();
	  HAL_Delay(1);
	}
	for(j=255; j>0; j--) {					// этот цикл создает эффект движения по кругу
			if(breakAnimation){
				breakAnimation=0;
				clearLeds();
				return;
			}
			for(i=0; i< NUM_OF_LEDS; i++) {		// этот цикл меняет позицию "стартового"
				setPixelColor(
						i,
						0,
						0,
						j,
						brightMode
				);
		  }
	  updateLeds();
	  HAL_Delay(1);
	}

}

/*
 * 		"Таймер" - раз в секунду зажигаем на индикаторе один пиксель пока индикатор не заполнится.
 * 		Потом заполняем индикатор новым цветом
 * 		Порядок заполнения индикатора:
 * 		Красный
 * 		Красный - Зеленый
 * 		Зеленый
 * 		Зеленый - Синий
 * 		Синий
 * 		Синий - Красный
 *
 * */

void mode_10(){			// анимация "таймер"
	uint16_t i,j;
	uint32_t delay = 0;
	// стартовые параметры цветов.
	uint16_t R=255;
	uint16_t G=0;
	uint16_t B=0;

	for(j=0;j<6;j++){		// переключаем режим свечения
		switch (j){
			case 0:			// заполнение красным. при зацикливании выглядит как удаление синего из красно-синего.
				break;

			case 1:
				G=255;		// добавляем к красному зеленый
				break;

			case 2:
				R=0;		// убираем от красно-зеленого красный
				break;

			case 3:
				B=255;		// добавляем к зеленому синий
				break;

			case 4:
				G=0;		// убираем от зелено-синего зеленый
				break;

			case 5:			// добавляем к синему красный
				R=255;
				break;

			default:
				break;
		}

		for(i=0; i< NUM_OF_LEDS; i++) {	// Заполнение индикатора
			setPixelColor(
					i,
					R,
					G,
					B,
					brightMode
			);
			updateLeds();

			delay=HAL_GetTick()+1000;		// устанавливаем время ожидания до следующего "тика"
			while(HAL_GetTick()<delay){		// мониторим прерывания выхода
				if(breakAnimation){
					breakAnimation=0;
					clearLeds();
					return;
				}
				HAL_Delay(1);
			}
		}
	}
}

void mode_11(){			// анимация "таймер"

	uint16_t i, j;

		for(j=0; j<256; j++) {					// этот цикл создает эффект движения по кругу
			if(breakAnimation){
				breakAnimation=0;
				clearLeds();
				return;
			}

			for(i=0; i< NUM_OF_LEDS/2; i++) {		// этот цикл меняет позицию "стартового"
				setPixelColor((j+i)%NUM_OF_LEDS,32*i, 0, 0, brightMode);
				HAL_Delay(25);
				//uint8_t WheelPos = ((i * 64 / (NUM_OF_LEDS))+ j) & 255;
				//fireball_Red(WheelPos, i,brightMode);
			}
			updateLeds();
			HAL_Delay(1);
		}
}

void mode_12(){			// анимация "таймер"

	uint16_t i, j;

		for(j=0; j<256; j++) {					// этот цикл создает эффект движения по кругу
			if(breakAnimation){
				breakAnimation=0;
				clearLeds();
				return;
			}

			for(i=0; i< NUM_OF_LEDS/2; i++) {		// этот цикл меняет позицию "стартового"
				setPixelColor((j+i)%NUM_OF_LEDS,0, 32*i, 0, brightMode);
				HAL_Delay(25);
				//uint8_t WheelPos = ((i * 64 / (NUM_OF_LEDS))+ j) & 255;
				//fireball_Red(WheelPos, i,brightMode);
			}
			updateLeds();
			HAL_Delay(1);
		}
}

void mode_13(){			// анимация "таймер"

	uint16_t i, j;

		for(j=0; j<256; j++) {					// этот цикл создает эффект движения по кругу
			if(breakAnimation){
				breakAnimation=0;
				clearLeds();
				return;
			}

			for(i=0; i< NUM_OF_LEDS/2; i++) {		// этот цикл меняет позицию "стартового"
				setPixelColor((j+i)%NUM_OF_LEDS,0, 0, 32*i, brightMode);
				HAL_Delay(25);
				//uint8_t WheelPos = ((i * 64 / (NUM_OF_LEDS))+ j) & 255;
				//fireball_Red(WheelPos, i,brightMode);
			}
			updateLeds();
			HAL_Delay(1);
		}
}

/*
 * 	Анимация создает падающий\растущий градиент
 * 	старая и не очень красивая
 */
void Gradient(	uint8_t WheelPos,		// смещение от начального цвета
				uint8_t led,			// Номер светодиода
				uint8_t Color_R, 		// Красный
				uint8_t Color_G, 		// Зеленый
				uint8_t Color_B, 		// Синий

				uint8_t brightness){	// Яркость
	WheelPos = 255 - WheelPos;
	setPixelColor(
			led, 					// Номер светодиода
			Color_R, 				// Красный
			Color_G, 				// Зеленый
			Color_B, 				// Синий
			brightness 				// Яркость
	);
	return;
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
  /* USER CODE BEGIN 2 */
  startupAnimation();		//запускаем стартовую анимацию
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// проигрываем нужную анимацию
	switch(animationMode){
		case 0:
			mode_0();
			break;

		case 1:
			mode_1();
			break;

		case 2:
			mode_2();
			break;

		case 3:
			mode_3();
			break;

		case 4:
			mode_4();
			break;

		case 5:
			mode_5();
			break;

		case 6:
			mode_6();
			break;

		case 7:
			mode_7();
			break;

		case 8:
			mode_8();
			break;

		case 9:
			mode_9();
			break;

		case 10:
			mode_10();
			break;

		case 11:
			mode_11();
			break;

		case 12:
			mode_12();
			break;

		case 13:
			mode_13();
			break;

		default:
			animationMode = 0;
			break;

	}


  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  HAL_GPIO_WritePin(GPIOA, APA_CLK_Pin|APA_DT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : APA_CLK_Pin APA_DT_Pin */
  GPIO_InitStruct.Pin = APA_CLK_Pin|APA_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BNT_1_Pin */
  GPIO_InitStruct.Pin = BNT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BNT_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_2_Pin */
  GPIO_InitStruct.Pin = BTN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  while (1) {};
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
