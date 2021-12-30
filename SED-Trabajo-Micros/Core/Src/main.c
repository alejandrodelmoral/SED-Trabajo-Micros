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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

// Pulsadores
volatile int pulsador_luces_ON = 0;
volatile int pulsador_luces_OFF = 0;
volatile int pulsador_puerta = 0;
volatile int pulsador_temp = 0;
volatile int pulsador_alarma = 0;

// Bluetooth
char readBuf[1];

// Iluminación
int encendidas = 0;
int luces_ON = 0;
int luces_OFF = 0;
uint32_t LDR_valor;
uint32_t iluminacion;

// Puerta
int abierta = 0;
int abriendo = 0;
int cerrando = 0;
uint32_t puerta_temp;

// Temperatura
uint32_t Temp_valor;
float temp = 0;
float temperatura_medida;
int medida_temp = 0;
int midiendo_temp = 0;
int no_midiendo_temp = 0;
uint32_t temperatura_temp;

// Alarma
float dist  = 0;
int alarma_ON = 0;
uint8_t frec_zumb = 200;
uint32_t alarma_temp;

// Detector de sonidos
uint32_t Sonidos_valor;

// Zumbador pasivo
uint8_t valor = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Bluetooth
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart6.Instance)
		HAL_UART_Receive_IT(&huart6, (uint8_t*)readBuf, 1);
}

// Pulsadores
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0) { // Temperatura
		pulsador_temp = 1;
	}

	if(GPIO_Pin == GPIO_PIN_1) { // Iluminación
		pulsador_luces_ON = 1;
	}

	if(GPIO_Pin == GPIO_PIN_2) { // Iluminación
		pulsador_luces_OFF = 1;
	}

	if(GPIO_Pin == GPIO_PIN_3) { // Puerta
		pulsador_puerta = 1;
	}

	if(GPIO_Pin == GPIO_PIN_4) { // Alarma
		pulsador_alarma = 1;
	}
}

// Debouncer
int debouncer(volatile int* button_int, GPIO_TypeDef* GPIO_port, uint16_t GPIO_number)
{
	static uint8_t button_count = 0;
	static int counter = 0;

	if(*button_int == 1) {
		if(button_count == 0) {
			counter = HAL_GetTick();
			button_count++;
		}
		if(HAL_GetTick() - counter >= 20) {
			counter = HAL_GetTick();
			if(HAL_GPIO_ReadPin(GPIO_port, GPIO_number) != 1) {
				button_count = 1;
			}
			else {
				button_count++;
			}
			if(button_count == 4) {
				button_count = 0;
				*button_int = 0;
				return 1;
			}
		}
	}
	return 0;
}

// Servomotor SG90
void moverServo(TIM_HandleTypeDef* htim, int grados)
{
	const int MAX = 20; // Max valor de frecuencia son 20ms -> 1/50Hz
	float ms = grados/90.0f + 0.5f; // De los valores min y max del servo (0.5ms son 0º, 2.5ms son 180º) -> Ecuacion recta
	float duty = ms/(float)MAX; // Porcentaje del ciclo encendido
	uint32_t CCR = htim->Instance->ARR * duty; // El valor del CCR es ARR * duty, el ARR es del propio htim
	htim->Instance->CCR2 = CCR; // Se asigna el CCR calculado al CCR2 del htim
}

// Ultrasonidos HC-SR04
float calcularDistancia(uint64_t time)
{
	const float Vsonido = 340.0f; // Velocidad sonido = 340m/s
	float dist = (float)time * Vsonido / 2.0f / 10000.0f; // El 1/10000 es por el cambio de conversion de s a us y de m a cm
	return dist; // En cm
}

void leerUltrasonido(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1); // Activar TRIG
	HAL_Delay(0.01); // Esperar 10 us = 0.01 ms
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0); // Desactivar TRIG
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static uint64_t timeRise = 0, timeFall = 0, timeDif = 0;
	static uint8_t EdgeCapture = 0;	// 0 si va a detectar flanco de subida, 1 si va a detectar falcno de bajada

	if (!EdgeCapture) // Si se va a detectar el flanco de subida
	{
		EdgeCapture = 1; // Se cambia para detectar despues el flanco de bajada

		timeRise = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Se guarda el instsnte de tiempo en el que se ha alcanzado el flanco de subida

		__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING); // Cambio del tipo de flanco para detectar flancos de bajada
	}
	else // Si se va a detectar el flanco de bajada
	{
		EdgeCapture = 0; // Se cambia para detectar otro flanco de subida

		timeFall = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Se guarda el instsnte de tiempo en el que se ha alcanzado el flanco de bajada

		__HAL_TIM_SET_COUNTER(htim, 0);	// Se pone a cero el contador de tiempo para la siguiente deteccion del flanco de subida
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING); // Cambio del tipo de flanco para detectar flancos de subida

		timeDif = timeFall - timeRise; // El tiempo necesario para calcular la distancia es la resta del tiempo de flancos
		dist = calcularDistancia(timeDif); // Se calcula la distancia en cm
	}
}

// Sensor de temperatura
float medirTemperatura(void)
{
	HAL_ADC_Start(&hadc2);
	if (HAL_ADC_PollForConversion(&hadc2, 100) == HAL_OK)
	{
		Temp_valor = HAL_ADC_GetValue(&hadc2);
	}
	HAL_ADC_Stop(&hadc2);

	uint32_t R_NTC = 10000.0 / (1023.0 / Temp_valor - 1.0);
	temp = 1.0 / ((1.0 / (25 + 273.15)) + (1.0 / 3950.0) * (log(R_NTC / 10000.0))) - 273.15;

	return temp;
}

// Sensor LDR
uint32_t medirLDR(void)
{
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
		LDR_valor = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);

	return LDR_valor;
}

// Control de la iluminación
void luces(void)
{
	iluminacion = medirLDR();

	if(iluminacion <= 60 || readBuf[0] == 'A' || debouncer(&pulsador_luces_ON, GPIOC, GPIO_PIN_1)) // Si hay poca luminosidad, o si se usa Bluetooth o si se pulsa el botón
	{
		if(encendidas == 0) // Si no están encendidas
		 {
			luces_ON = 1; // Activación del flag para encender
		 }

		 else // Si están encendidas
		 {
			 luces_OFF = 1; // Activación del flag para apagar
		 }

		 readBuf[0] = 0; // Reinicio del Bluetooth
		 pulsador_luces_ON = 0; // Reinicio del pulsador
	}

	if(encendidas == 0 && luces_ON == 1) // Si están apagadas y activado el flag para encender, se encienden
	{
		encendidas = 1; // Encendidas
		luces_ON = 0; // Desactivación del flag para encender
	}

	if(encendidas == 1 && luces_OFF == 0) // Si están encendidas y desactivado el flag para apagar, se apagan
	{
		luces_OFF = 1; // Activación del flag para apagar
	}

	if(encendidas == 1 && luces_OFF == 1) // Si están encendidas y activado el flag para apagar, se apagan
	{
		encendidas = 0; // Apagadas
		luces_OFF = 0; // Desactivación del flag para apagar
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, encendidas); // LED encendido cuando se encienden las luces
}

// Control de la puerta
void puerta(void)
{
	if(readBuf[0] == 'C' || debouncer(&pulsador_puerta, GPIOC, GPIO_PIN_3)) // Si se usa Bluetooth o si se pulsa el botón
	{
		 if(abierta == 0) // Si la puerta está cerrada
		 {
			 abriendo = 1; // Activación del flag para abrir puerta
		 }

		 else // Si la puerta está abierta
		 {
			 cerrando = 1; // Activación del flag para cerrar puerta
			 puerta_temp = 0; // Reinicio del tiempo de la puerta abierta
		 }

		 readBuf[0] = 0; // Reinicio del Bluetooth
		 pulsador_puerta = 0; // Reinicio del pulsador
	}

	if(abierta == 0 && abriendo == 1) // Si la puerta está cerrada y activado el flag para abrir la puerta, se abre
	{
		moverServo(&htim2, 90); // Se abre la puerta
		abierta = 1; // Puerta ya abierta
		abriendo = 0; // Desactivación del flag para abrir puerta
		puerta_temp = HAL_GetTick(); // Se coge el tiempo actual
	}

	if(abierta == 1 && cerrando == 0 && (HAL_GetTick() - puerta_temp) > 5000) // Si la puerta está abierta, desactivado el flag para cerrar la puerta y han pasado 5s, se cierra
	{
		cerrando = 1; // Activación del flag para cerrar puerta
		puerta_temp = 0; // Reinicio del tiempo de la puerta abierta
	}

	if(abierta == 1 && cerrando == 1) // Si la puerta está abierta y activado el flag para cerrar la puerta, se cierra
	{
		moverServo(&htim2, 0); // Se cierra la puerta
		abierta = 0; // Puerta ya cerrada
		cerrando = 0; // Desactivación del flag para cerrar puerta
		puerta_temp = 0; // Reinicio del tiempo de la puerta abierta
	}
}

// Control de la temperatura
void temperatura(void)
{
	if(readBuf[0] == 'D' || debouncer(&pulsador_temp, GPIOA, GPIO_PIN_0)) // Si se usa Bluetooth o si se pulsa el botón
	{
		 if(medida_temp == 0) // Si no se está midiendo
		 {
			 midiendo_temp = 1; // Activación del flag para medir
		 }

		 else // Si se está midiendo
		 {
			 no_midiendo_temp = 1; // Activación del flag para no medir
			 temperatura_temp = 0; // Reinicio del tiempo de la medición
		 }

		 readBuf[0] = 0; // Reinicio del Bluetooth
		 pulsador_temp = 0; // Reinicio del pulsador
	}

	if(medida_temp == 0 && midiendo_temp == 1) // Si no se está midiendo y activado el flag para medir, se mide
	{
		temperatura_medida = medirTemperatura(); // Se mide la temperatura

		medida_temp = 1; // Midiendo
		midiendo_temp = 0; // Desactivación del flag para medir
		temperatura_temp = HAL_GetTick(); // Se coge el tiempo actual
	}

	if(medida_temp == 1 && no_midiendo_temp == 0 && (HAL_GetTick() - temperatura_temp) > 10000) // Si está midiendo, desactivado el flag para no medir y han pasado 10s, no mide
	{
		no_midiendo_temp = 1; // Activación del flag para no medir
		temperatura_temp = 0; // Reinicio del tiempo de la medición
	}

	if(medida_temp == 1 && no_midiendo_temp == 1) // Si está midiendo y activado el flag para no medir, no mide
	{
		medida_temp = 0; // No midiendo
		no_midiendo_temp = 0; // Desactivación del flag para no medir
		temperatura_temp = 0; // Reinicio del tiempo de la medición
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, medida_temp); // LED encendido cuando se mide temperatura
}

// Control de la alarma
void alarma(void)
{
	leerUltrasonido(GPIOA, GPIO_PIN_10); // Leer Ultrasonido conectado a pin A10
	HAL_Delay(100); // Espera para volver a disparar, NO menor a 100ms !!!

	if(dist < 10) // Si la distancia al sensor es menor de 10cm
	{
		alarma_ON = 1; // Activación de la alarma
		alarma_temp = HAL_GetTick(); // Se coge el tiempo actual
		htim2.Instance->CCR3 = frec_zumb; // Se enciende la alarma
	}

	if(alarma_ON == 1) // Si está activada la alarma
	{
		if(readBuf[0] == 'E' || debouncer(&pulsador_alarma, GPIOA, GPIO_PIN_4) || HAL_GetTick() - alarma_temp > 5000) // Si se usa Bluetooth, o si se pulsa el botón o han pasado 5s, se desactiva
		{
			alarma_ON = 0; // Desactivación de la alarma
			htim2.Instance->CCR3 = frec_zumb; // Se apaga la alarma
			alarma_temp = 0; // Reinicio del tiempo de la alarma
			readBuf[0] = 0; // Reinicio del Bluetooth
			pulsador_alarma = 0; // Reinicio del pulsador
		}
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, alarma_ON); // LED encendido cuando suena la alarma
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
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_ADC3_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart6, (uint8_t*)readBuf, 1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  puerta(); // Control puerta
	  luces(); // Control iluminación
	  temperatura(); // Control temperatura
	  alarma(); // Control alarma

	  /*
	  // Ejemplo del detector de sonidos
	  HAL_ADC_Start(&hadc3);
	  if (HAL_ADC_PollForConversion(&hadc3, 100) == HAL_OK)
	  {
		  Sonidos_valor = HAL_ADC_GetValue(&hadc3);
	  }
	  HAL_ADC_Stop(&hadc3);

	  // Ejemplo zumbador pasivo
	  while(valor < 255)
	  {
		  htim2.Instance->CCR3 = valor;
		  valor += 20;
		  HAL_Delay(1000);
	  }

	  valor = 0;
	  */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_10B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_10B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 45-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 900-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 692-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_iluminaci_n_Pin|LED_temperatura_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_alarma_Pin|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Pulsador_luces_ON_Pin Pulsador_luces_OFF_Pin Pulsador_puerta_Pin */
  GPIO_InitStruct.Pin = Pulsador_luces_ON_Pin|Pulsador_luces_OFF_Pin|Pulsador_puerta_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 Pulsador_alarma_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|Pulsador_alarma_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_iluminaci_n_Pin LED_temperatura_Pin */
  GPIO_InitStruct.Pin = LED_iluminaci_n_Pin|LED_temperatura_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_alarma_Pin TRIG_Pin */
  GPIO_InitStruct.Pin = LED_alarma_Pin|TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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

