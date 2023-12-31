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
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_TIMEOUT 1000
#define TIME_GAME 60 // Temps pour defuser la bombe
#define BUFFER_SIZE 10
#define PI 3 // valeur exacte de pi
#define DELAY_DEBOUNCE 300

// Define pour le MP3 - a utilise avec la fonction void play_track(uint8_t track_nb);
// Corresponds à la position de la bande sons dans la mémoire de l'interface haut parleur
#define BIP 1
#define SOUND_START_BOMB 2
#define BOMB_DEFUSED 3
#define BOMB_EXPLODED 4
#define SOUND_PUSH_BUTTON 5
#define BOMB_HAS_BEEN_PLANTED 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// region[rgba(1, 70, 70, 0.15)]  <= colore la région (Colored Region extension for vscode)

uint32_t seed;
uint16_t adcData[2];
volatile bool adcOk = false;

uint8_t time_in_second = TIME_GAME;
uint8_t flag_bipbip = 0;
uint8_t freqence_bipbip = 0;

bool bombPlanted = false;
uint8_t buttonPlantCurrentIndex = 0;
uint8_t buttonOrderPlant[4] = {1, 2, 3, 4}; // Séquence de boutons pour planter la bombe

volatile bool buttonOk = false;
uint8_t buttonCurrentIndex = 0;
uint8_t buttonOrderDefuse[4];
uint32_t buttonElapsed[4] = {0, 0, 0, 0}; // Tableau pour le debounce des boutons
typedef enum                              // Enumération des états du jeu
{
  ETAT_INITIALISATION,
  ETAT_JEU,
  ETAT_VICTOIRE,
  ETAT_DEFAITE
} EtatJeu;

EtatJeu etat = ETAT_INITIALISATION;
// endregion
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM10_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void ledUpdate(uint16_t adcValue, TIM_HandleTypeDef *htim, uint32_t Channel);
void BCD_SendCommand(uint8_t addr, uint8_t data);
void BCD_Init(uint16_t time_in_second);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
int BCD_updateClock(uint16_t time_in_second);
void secondToClockDisplay(uint16_t time_in_second);
void play();
void play_track(uint8_t track_nb);
void randomButtonSequence();

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
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_TIM11_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  printf("Starting\r\n");

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3); // Timer bipbip

  BCD_Init(0);
  play_track(SOUND_START_BOMB);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // region[rgba(52, 73, 94, 0.1)]
  while (1)
  {
    switch (etat)
    {
      // Initialisation
    case ETAT_INITIALISATION:
    {
      printf("ETAT_INITIALISATION\r\n");
      if (bombPlanted == true)
      {
        play_track(BOMB_HAS_BEEN_PLANTED);
        HAL_Delay(1500);          // Délai pour jouer le son de la bombe plantée
        BCD_Init(time_in_second); // Clignotement de l'afficheur et préparation à l'affichage
        randomButtonSequence();   // Générer une nouvelle séquence si nécessaire
        bombPlanted = false;
        etat = ETAT_JEU;
      }
      break;
    }
      // Jeu
    case ETAT_JEU:
    {
      HAL_TIM_Base_Start_IT(&htim10); // Timer décompteur
      if (etat == ETAT_JEU)
      {
        printf("ETAT_JEU\r\n");
        if (time_in_second == 0)
        {
          etat = ETAT_DEFAITE;
        }
        if (buttonOk)
        {
          HAL_GPIO_TogglePin(LED_5_GPIO_Port, LED_5_Pin);
          HAL_Delay(200);
          if (adcOk)
          {
            etat = ETAT_VICTOIRE;
          }
        }
        // Gestion fréquence des bip bip
        if (time_in_second > 15)  {freqence_bipbip = 200;}
        if (time_in_second <= 15) {freqence_bipbip = 100;}
        if (time_in_second <= 10) {freqence_bipbip = 50;}
        if (time_in_second <= 5)   {freqence_bipbip = 30;}
        if (flag_bipbip > freqence_bipbip)
        {
          play_track(BIP);
          flag_bipbip = 0;
        }
      }
      break;
    }
    // Victoire
    case ETAT_VICTOIRE:
    {
      if (etat == ETAT_VICTOIRE)
      {
        printf("ETAT_VICTOIRE\r\n");
        HAL_TIM_Base_Stop_IT(&htim10);
        play_track(BOMB_DEFUSED);
        etat = ETAT_INITIALISATION;
      }
      break;
    }
    // Défaite
    case ETAT_DEFAITE:
    {
      if (etat == ETAT_DEFAITE)
      {
        printf("ETAT_DEFAITE\r\n");
        HAL_TIM_Base_Stop_IT(&htim10);
        play_track(BOMB_EXPLODED);
        etat = ETAT_INITIALISATION;
      }
    }
    default:
      break;
    }
  }
  // endregion
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 2;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */
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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 499;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 15999;
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
 * @brief TIM9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim9, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 31999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim10, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);
}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN_4_Pin BTN_3_Pin */
  GPIO_InitStruct.Pin = BTN_4_Pin | BTN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_5_Pin */
  GPIO_InitStruct.Pin = LED_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_1_Pin BTN_2_Pin */
  GPIO_InitStruct.Pin = BTN_1_Pin | BTN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
  ITM_SendChar(ch);
  return ch;
}

// Fonction random
// region[rgba(49, 120, 80, 0.12)]
// Générateur linéaire congruentiel aléatoire
void randomGLC()
{
  const uint32_t a = 1664525;
  const uint32_t c = 1013904223;
  const uint32_t m = 0xFFFF;

  seed = (a * (seed) + c) % m;
}

/**
 * Mélange les valeurs de buttonOrderDefuse
 */
void randomButtonSequence()
{
  uint8_t numbers[] = {1, 2, 3, 4};

  for (uint8_t i = 1; i < 4; i++)
  {
    int j = (seed % (i + 1));
    uint8_t temp = numbers[i];
    numbers[i] = numbers[j];
    numbers[j] = temp;
  }

  // Copiez les valeurs mélangées dans buttonOrderDefuse
  for (uint8_t i = 0; i < 4; i++)
  {
    buttonOrderDefuse[i] = numbers[i];
  }
}
// endregion

// Gestion de l'ADC
// region[rgba(90, 130, 0, 0.1)]
void ledUpdate(uint16_t data, TIM_HandleTypeDef *Timer, uint32_t Channel)
{
  // normaliser la valeur de l'ADC pour qu'elle varie de 0.0 à 1.0
  double dataNorm = (double)data / 0xFFF;

  double radian_part = 2 * PI * dataNorm;

  // Calculer la valeur de la sinusoïde, ajustée pour varier de 0 à 1
  double sinusValue = (sin(radian_part / 2));

  // Ajuster pour la plage PWM complète
  uint16_t pwmValue = 0xFFFF * sinusValue;

  // Définir la valeur PWM
  __HAL_TIM_SET_COMPARE(Timer, Channel, pwmValue);
}

// endregion

// Gestion des boutons
// region[rgba(0, 0, 255, 0.1)]

// Vérifie si le bouton appuyé est dans la séquence de boutons pour désamorcer la bombe
void checkUserInput(uint8_t userInput)
{
  if (userInput == buttonOrderDefuse[buttonCurrentIndex]) // Vérifier si le bouton appuyé correspond
  {
    buttonCurrentIndex++;
    HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_SET);

    if (buttonCurrentIndex == 4) // 4 itérations correctes = met le flag à true
    {
      buttonOk = true;
    }
  }
  else // reset le compteur si le bouton appuyé ne correspond pas
  {
    buttonCurrentIndex = 0;
    HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET);
  }
}

// Vérifie si le bouton appuyé est dans la séquence de boutons pour planter la bombe
void checkButtonOrderPlant(uint8_t pressedButton)
{
  if (pressedButton == buttonOrderPlant[buttonPlantCurrentIndex])
  {
    buttonPlantCurrentIndex++;
    HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_SET);

    if (buttonPlantCurrentIndex == 1)
    {
      BCD_SendCommand(0x04, 0x01);
    }
    if (buttonPlantCurrentIndex == 2)
    {
      BCD_SendCommand(0x04, 0x02);
      BCD_SendCommand(0x03, 0x01);
    }
    if (buttonPlantCurrentIndex == 3)
    {
      BCD_SendCommand(0x04, 0x03);
      BCD_SendCommand(0x03, 0x02);
      BCD_SendCommand(0x02, 0x01);
    }

    if (buttonPlantCurrentIndex == 4)
    {
      BCD_SendCommand(0x04, 0x04);
      BCD_SendCommand(0x03, 0x03);
      BCD_SendCommand(0x02, 0x02);
      BCD_SendCommand(0x01, 0x01);

      bombPlanted = true; // Flag indiquant que la bombe a été plantée

      HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET);
      buttonPlantCurrentIndex = 0; // Réinitialiser pour la prochaine utilisation
    }
  }
  else
  {
    buttonPlantCurrentIndex = 0;

    HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET);
  }
}

// interruption pour les boutons
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
  case BTN_1_Pin:
    if (HAL_GetTick() > (buttonElapsed[0] + DELAY_DEBOUNCE))
    {
      printf("btn 1\r\n");
      buttonElapsed[0] = HAL_GetTick();
      play_track(SOUND_PUSH_BUTTON);
      if (etat == ETAT_INITIALISATION)
      {
        checkButtonOrderPlant(1);
      }
      else
      {
        checkUserInput(1);
      }
    }
    break;
  case BTN_2_Pin:
    if (HAL_GetTick() > (buttonElapsed[1] + DELAY_DEBOUNCE))
    {
      printf("btn 2\r\n");
      buttonElapsed[1] = HAL_GetTick();
      play_track(SOUND_PUSH_BUTTON);
      if (etat == ETAT_INITIALISATION)
      {
        checkButtonOrderPlant(2);
      }
      else
      {
        checkUserInput(2);
      }
    }
    break;
  case BTN_3_Pin:
    if (HAL_GetTick() > (buttonElapsed[2] + DELAY_DEBOUNCE))
    {
      printf("btn 3\r\n");
      buttonElapsed[2] = HAL_GetTick();
      play_track(SOUND_PUSH_BUTTON);
      if (etat == ETAT_INITIALISATION)
      {
        checkButtonOrderPlant(3);
      }
      else
      {
        checkUserInput(3);
      }
    }
    break;
  case BTN_4_Pin:
    if (HAL_GetTick() > (buttonElapsed[3] + DELAY_DEBOUNCE))
    {
      printf("btn 4\r\n");
      buttonElapsed[3] = HAL_GetTick();
      play_track(SOUND_PUSH_BUTTON);
      if (etat == ETAT_INITIALISATION)
      {
        checkButtonOrderPlant(4);
      }
      else
      {
        checkUserInput(4);
      }
    }
    break;
  default:
    break;
  }
}
// endregion

// Fonction BCD
// region[rgba(150, 30, 200, 0.08)]
// Envoi de la commande à l'afficheur
void BCD_SendCommand(uint8_t addr, uint8_t data)
{
  uint8_t mot[2] = {addr, data};

  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, mot, 2, SPI_TIMEOUT);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
}

// Initialisation de l'affichage de l'horloge
void BCD_Init(uint16_t time_in_second)
{

  BCD_SendCommand(0x0C, 0x01); // shutdown pour reset/economie energie
  BCD_SendCommand(0x09, 0x0F); // decode permet utiliser tab predefinie au lieu seg/seg
  BCD_SendCommand(0x0B, 0x03); // scanlimit
  BCD_SendCommand(0x0A, 0x01); // regle intensité lumineuse

  // flash de l'afficheur
  for (int i = 0; i < 3; i++)
  {
    BCD_SendCommand(0xFF, 0xFF);
    HAL_Delay(50);
    BCD_SendCommand(0xFF, 0x00);
    HAL_Delay(50);
  }

  uint8_t seconds = 0;
  uint8_t diz_seconds = 0;
  uint8_t minutes = 0;
  uint8_t diz_minutes = 0;

  minutes = time_in_second / 60;
  seconds = time_in_second % 60;

  if (minutes >= 10)
  {
    diz_minutes = minutes / 10;
    minutes = minutes % 10;
  }
  if (seconds >= 10)
  {
    diz_seconds = seconds / 10;
    seconds = seconds % 10;
  }

  BCD_SendCommand(0x01, diz_minutes);
  BCD_SendCommand(0x02, minutes);
  BCD_SendCommand(0x03, diz_seconds);
  BCD_SendCommand(0x04, seconds);
}

// Mise à jour de l'affichage de l'horloge
int BCD_updateClock(uint16_t time_in_second)
{

  if (time_in_second == 0)
  {
    return 0;
  }

  time_in_second--;

  uint8_t seconds = 0;
  uint8_t diz_seconds = 0;
  uint8_t minutes = 0;
  uint8_t diz_minutes = 0;

  minutes = time_in_second / 60;
  seconds = time_in_second % 60;

  if (seconds >= 10)
  {
    diz_seconds = seconds / 10;
    seconds = seconds % 10;
  }
  if (minutes >= 10)
  {
    diz_minutes = minutes / 10;
    minutes = minutes % 10;
  }

  BCD_SendCommand(0x01, diz_minutes);
  BCD_SendCommand(0x02, minutes);
  BCD_SendCommand(0x03, diz_seconds);
  BCD_SendCommand(0x04, seconds);

  return time_in_second;
}
// endregion

// Gestion du son
void play(void)
{
  uint8_t array[4] = {0xAA, 0x02, 0x00, 0xAC};
  HAL_UART_Transmit(&huart4, array, sizeof(array), 1000);
}

// Gestion des sons pré-enregistrés
void play_track(uint8_t track_nb)
{
  uint8_t array[6] = {0xAA, 0x07, 0x02, 0x00, track_nb, 0xB3 + track_nb};
  HAL_UART_Transmit(&huart4, array, sizeof(array), 1000);
}

// Gestion du temps
//  region[rgba(180, 100, 0, 0.1)]
void secondToClockDisplay(uint16_t time_in_second)
{
}

// interruption timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) // Timer de 5ms
  {
    // Démarrer la conversion ADC
    HAL_ADC_Start_DMA(&hadc, (uint32_t *)adcData, 2);
    // update PWM des leds
    ledUpdate(adcData[0], &htim9, TIM_CHANNEL_2);
    ledUpdate(adcData[1], &htim11, TIM_CHANNEL_1);

    // génération de la seed
    randomGLC();

    // Gestion du bipbip
    flag_bipbip++;

    // vérification valeur ADC
    if ((adcData[0] > 0xF100) && (adcData[1] > 0xF100))
    {
      adcOk = true;
      printf("adcOk\r\n");
    }
  }

  if (htim->Instance == TIM10) // Timer de 1s
  {
    time_in_second = BCD_updateClock(time_in_second);
  }

  if (htim->Instance == TIM3)
  {
  }
}
// endregion
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

#ifdef USE_FULL_ASSERT
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
