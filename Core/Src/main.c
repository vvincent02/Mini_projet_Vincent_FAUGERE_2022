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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stdio.h"
#include "string.h"
#include "HorombeRGB565.h"
#include "background.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_WIDTH 480
#define LCD_HEIGHT 272
#define BACKGROUND_COLOR LCD_COLOR_DARKBLUE
#define W_PALET 73 // largeur du palet (impair)
#define H_PALET 5 // hauteur du palet (impair)
#define PALET_COLOR LCD_COLOR_LIGHTCYAN
#define Y_PALET 240 // position du palet sur la verticale
#define RAD_BALL 3 // rayon de la bille
#define BRICK_WIDTH 21 // nombre impair
#define BRICK_HEIGHT 9 // nombre impair
#define BALL_COLOR LCD_COLOR_WHITE
#define SCORE_COLOR LCD_COLOR_WHITE
#define GAME_OVER_COLOR LCD_COLOR_RED
#define BRICK_BONUS_COLOR_EXT LCD_COLOR_ORANGE // couleur de la brique bonus pour l'extension du palet

#define PALET_EXTENSION_COEF 1.5 // coefficient d'agrandissement du palet
#define DELAY_EXTENSION_BONUS 15000 // durée du bonus d'extension du palet

#define INITIAL_BILLE_VELOCITY 0.1 // velocity in pixel/ms
#define K_INFLUENCE_ON_DIRECTION 2 // influence de la vitesse du palet sur la direction de rebond (selon x) (en ms) (x_dir += K*palet_vel à chaque rebond)
#define K_SENSITIVITY 0.03 // en (pixel/ms) / (deg/s) (palet_vel = K*d(theta_gyro)/dt)
#define GYRO_SENSITIVITY_OFFSET 30 // valeur minimale à dépasser pour pouvoir bouger le palet
#define MAX_X_Y_DIRECTION 0.95 // valeur maximale d'une des composantes du vecteur de direction de la bille

#define TRANSITION_DELAY 6000 // durée minimale entre deux décalages des briques vers le bas


/* --- IU register address --- */
#define IU_ADDRESS 0x28
#define IU_ST_RESULT 0x36
#define UNIT_SEL_ADD 0x3B
#define GYR_DATA_LSB 0x16

// offset range
#define GYR_OFFSET_LSB 0x61
#define MAX_OFFSET_RANGE 0x64 // de 2000 à 32000 en hexadécimal

// operating_mode
#define OPR_MODE_ADD 0x3D
#define GYROONLY 0b0011
/* --- */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

SDRAM_HandleTypeDef hsdram1;

osThreadId defaultTaskHandle;
osThreadId paletHandle;
osThreadId billeHandle;
osThreadId game_over_tskHandle;
osThreadId display_scoreHandle;
osThreadId bricksHandle;
osMessageQId on_paletHandle;
osSemaphoreId LCD_mutexHandle;
/* USER CODE BEGIN PV */

osThreadId bille2Handle;

uint8_t game_over = 0;
uint8_t kill_all = 0;
uint8_t create_all = 0;

// timer pour les affichages
RTC_TimeTypeDef time;
RTC_DateTypeDef date;

// données reçues par le gyro
uint8_t rx_data [2] = {0,0};

uint16_t x_palet;
float palet_vel = 0; // en pixel/ms
uint16_t score_value = 0;
uint16_t w_palet = W_PALET;
int32_t t0 = -DELAY_EXTENSION_BONUS; // instant démarrage du bonus extension du palet

uint32_t mat [LCD_WIDTH/BRICK_WIDTH] [LCD_HEIGHT/BRICK_HEIGHT]; // matrix of brick information

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_UART7_Init(void);
static void MX_I2C1_Init(void);
static void MX_RNG_Init(void);
void StartDefaultTask(void const * argument);
void palet_(void const * argument);
void bille_(void const * argument);
void game_over_tsk_(void const * argument);
void display_score_(void const * argument);
void bricks_(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t inertial_unit_is_ok ();

void display_brick (uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint32_t color);

uint32_t get_time (RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC3_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_UART7_Init();
  MX_I2C1_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  	BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
    BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+ BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4);
    BSP_LCD_DisplayOn();
    BSP_LCD_SelectLayer(0);
    BSP_LCD_Clear(BACKGROUND_COLOR);
    BSP_LCD_SetBackColor(BACKGROUND_COLOR);
    BSP_LCD_SelectLayer(1);
    BSP_LCD_Clear(BACKGROUND_COLOR);
    BSP_LCD_SetBackColor(BACKGROUND_COLOR);
    BSP_LCD_SetFont(&Font12);

    BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

    // interruptions
    HAL_GPIO_EXTI_IRQHandler(BP1_Pin);
    HAL_I2C_EV_IRQHandler(&hi2c1);

    uint8_t tx_data [7];
    /* --- init and set gyro parameters --- */
    if (!inertial_unit_is_ok())	{ // test si la centrale est opérationnelle
    	return -1; // problème avec la centrale intertielle
    }

    // configuration de operating_mode (OPR_MODE)
    tx_data [0] = OPR_MODE_ADD;
    tx_data [1] = GYROONLY;
    if (HAL_I2C_Master_Transmit(&hi2c1, IU_ADDRESS<<1, tx_data, 2, portMAX_DELAY) != HAL_OK)	{
        return -1;
    }

    // réglage unité vitesse de rotation (deg/s : 0b0x - rad/s : 0b1x)
    tx_data [0] = UNIT_SEL_ADD;
    tx_data [1] = 0b00;
    if (HAL_I2C_Master_Transmit(&hi2c1, IU_ADDRESS<<1, tx_data, 2, portMAX_DELAY) != HAL_OK)	{
    	return -1;
    }

    // réglage offset range (plage vitesse de rotation max) pour chaque axe
    tx_data [0] = GYR_OFFSET_LSB;
    tx_data [1] = MAX_OFFSET_RANGE & 0xFF;
    tx_data [2] = (MAX_OFFSET_RANGE & 0xFF00)>>8;
    tx_data [3] = MAX_OFFSET_RANGE & 0xFF;
    tx_data [4] = (MAX_OFFSET_RANGE & 0xFF00)>>8;
    tx_data [5] = MAX_OFFSET_RANGE & 0xFF;
    tx_data [6] = (MAX_OFFSET_RANGE & 0xFF00)>>8;
    if (HAL_I2C_Master_Transmit(&hi2c1, IU_ADDRESS<<1, tx_data, 7, portMAX_DELAY) != HAL_OK)	{
        return -1;
    }
    /* --------------------------- */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of LCD_mutex */
  osSemaphoreDef(LCD_mutex);
  LCD_mutexHandle = osSemaphoreCreate(osSemaphore(LCD_mutex), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of on_palet */
  osMessageQDef(on_palet, 4, uint8_t);
  on_paletHandle = osMessageCreate(osMessageQ(on_palet), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of palet */
  osThreadDef(palet, palet_, osPriorityHigh, 0, 1024);
  paletHandle = osThreadCreate(osThread(palet), NULL);

  /* definition and creation of bille */
  osThreadDef(bille, bille_, osPriorityHigh, 0, 1024);
  billeHandle = osThreadCreate(osThread(bille), NULL);

  /* definition and creation of game_over_tsk */
  osThreadDef(game_over_tsk, game_over_tsk_, osPriorityLow, 0, 128);
  game_over_tskHandle = osThreadCreate(osThread(game_over_tsk), NULL);

  /* definition and creation of display_score */
  osThreadDef(display_score, display_score_, osPriorityLow, 0, 1024);
  display_scoreHandle = osThreadCreate(osThread(display_score), NULL);

  /* definition and creation of bricks */
  osThreadDef(bricks, bricks_, osPriorityNormal, 0, 4096);
  bricksHandle = osThreadCreate(osThread(bricks), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

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
  hi2c1.Init.Timing = 0x00C0EAFF;
  hi2c1.Init.OwnAddress1 = 0;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00C0EAFF;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 40;
  hltdc.Init.VerticalSync = 9;
  hltdc.Init.AccumulatedHBP = 53;
  hltdc.Init.AccumulatedVBP = 11;
  hltdc.Init.AccumulatedActiveW = 533;
  hltdc.Init.AccumulatedActiveH = 283;
  hltdc.Init.TotalWidth = 565;
  hltdc.Init.TotalHeigh = 285;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 272;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xC0000000;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm B
  */
  sAlarm.Alarm = RTC_ALARM_B;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the TimeStamp
  */
  if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_POS1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED14_Pin|LED15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED16_GPIO_Port, LED16_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED13_Pin|LED17_Pin|LED11_Pin|LED12_Pin
                          |LED2_Pin|LED18_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_D2_Pin */
  GPIO_InitStruct.Pin = QSPI_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TXD1_Pin RMII_TXD0_Pin RMII_TX_EN_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin|RMII_TXD0_Pin|RMII_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D2_Pin
                           ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D2_Pin
                          |ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDIF_RX0_Pin */
  GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
  HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDMMC_CK_Pin SDMMC_D3_Pin SDMMC_D2_Pin PC9
                           PC8 */
  GPIO_InitStruct.Pin = SDMMC_CK_Pin|SDMMC_D3_Pin|SDMMC_D2_Pin|GPIO_PIN_9
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BP2_Pin PA6 */
  GPIO_InitStruct.Pin = BP2_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED14_Pin LED15_Pin */
  GPIO_InitStruct.Pin = LED14_Pin|LED15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_NCS_Pin */
  GPIO_InitStruct.Pin = QSPI_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(QSPI_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_INT_Pin */
  GPIO_InitStruct.Pin = Audio_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_P_Pin OTG_FS_N_Pin */
  GPIO_InitStruct.Pin = OTG_FS_P_Pin|OTG_FS_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SAI2_MCLKA_Pin SAI2_SCKA_Pin SAI2_FSA_Pin SAI2_SDA_Pin */
  GPIO_InitStruct.Pin = SAI2_MCLKA_Pin|SAI2_SCKA_Pin|SAI2_FSA_Pin|SAI2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI2_SDB_Pin */
  GPIO_InitStruct.Pin = SAI2_SDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(SAI2_SDB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin LED16_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|LED16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG9 RMII_RXER_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_9|RMII_RXER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDMMC_CMD_Pin */
  GPIO_InitStruct.Pin = SDMMC_CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(SDMMC_CMD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TP3_Pin PH13 NC2_Pin */
  GPIO_InitStruct.Pin = TP3_Pin|GPIO_PIN_13|NC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DISP_Pin */
  GPIO_InitStruct.Pin = LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_DISP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED13_Pin LED17_Pin LED11_Pin LED12_Pin
                           LED2_Pin LED18_Pin */
  GPIO_InitStruct.Pin = LED13_Pin|LED17_Pin|LED11_Pin|LED12_Pin
                          |LED2_Pin|LED18_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : BP1_Pin */
  GPIO_InitStruct.Pin = BP1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BP1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_D1_Pin QSPI_D3_Pin QSPI_D0_Pin */
  GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)	{
	/* gestion reprise du jeu */
	if (game_over)	{
		create_all = 1;
		HAL_GPIO_EXTI_IRQHandler(GPIO_Pin); // clear flags
	}
}

/* --- interruption envoi/réception I2C centrale intertielle --- */
void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef * hi2c)	{
	// dès que le buffer a été transmis, la fonction est exécutée
	HAL_I2C_Master_Receive_IT(hi2c, (IU_ADDRESS<<1) | 1, rx_data, 2);
}

/* ------------------------------------------------------------- */

uint8_t inertial_unit_is_ok ()	{
	uint8_t tx_data = IU_ST_RESULT;
	if (HAL_I2C_Master_Transmit(&hi2c1, IU_ADDRESS<<1, &tx_data, 1, portMAX_DELAY) != HAL_OK)	{
		return 0;
	}
	if (HAL_I2C_Master_Receive(&hi2c1, (IU_ADDRESS<<1) + 1, rx_data, 1, portMAX_DELAY) != HAL_OK)	{
		return 0;
	}
	if (rx_data [0] != 0x0F)	{
		return 0;
	}
	return 1;
}

void display_brick (uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint32_t color)	{
	BSP_LCD_SetTextColor(color);
	BSP_LCD_FillRect(x + 1, y + 1, width - 2, height - 2);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawRect(x, y, width - 1, height - 1);
}

uint32_t get_time (RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *time, RTC_DateTypeDef *date)	{
	HAL_RTC_GetTime(hrtc, time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, date, RTC_FORMAT_BIN);
	return (time->SecondFraction-time->SubSeconds)*1000/(time->SecondFraction+1) + 1000*time->Seconds + 60000*time->Minutes + 3600000*time->Hours;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_palet_ */
/**
* @brief Function implementing the palet thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_palet_ */
void palet_(void const * argument)
{
  /* USER CODE BEGIN palet_ */
	float x = LCD_WIDTH/2;
	uint16_t x_old = LCD_WIDTH/2;
	uint8_t tx_data = GYR_DATA_LSB;
	uint32_t time_old = 0;
	uint32_t time_new;

	taskENTER_CRITICAL();
	x_palet = x;
	taskEXIT_CRITICAL();

	taskENTER_CRITICAL();
	time_old = get_time(&hrtc, &time, &date);
	taskEXIT_CRITICAL();

	/* Infinite loop */
  for(;;)
  {
	  /* --- calcul de la nouvelle position --- */
	  taskENTER_CRITICAL() ;
	  if (((int16_t)(rx_data[0] + (rx_data[1]<<8)) >= GYRO_SENSITIVITY_OFFSET) || (((int16_t)(rx_data[0] + (rx_data[1]<<8)) <= -GYRO_SENSITIVITY_OFFSET)))	{ //permet d'éviter de prendre en compte le bruit (au repos)
		  palet_vel = K_SENSITIVITY*(int16_t)(rx_data[0] + (rx_data[1]<<8))/16.0;
	  }
	  else	{
		  palet_vel = 0;
	  }
	  taskEXIT_CRITICAL();
	  /* --- récupération du temps depuis l'affichage de la position précédente --- */
	  taskENTER_CRITICAL();
	  time_new = get_time(&hrtc, &time, &date);
	  taskEXIT_CRITICAL();
	  /* --- */

	  x += palet_vel*(time_new-time_old);
	  time_old = time_new;

	  /*--- gestion du palet sur les bords ---*/
	  if (x < w_palet/2 + H_PALET/2)	{
		  x = w_palet/2 + H_PALET/2;
	  }
	  if (x > LCD_WIDTH - w_palet/2 - H_PALET/2 - 1)	{
		  x = LCD_WIDTH - w_palet/2 - H_PALET/2 - 1;
	  }
	  /* --- */

	  HAL_I2C_Master_Transmit_IT(&hi2c1, IU_ADDRESS<<1, &tx_data, 1); // demande de récupération des données
	  /* --- */

	  /* --- affichage palet sur écran --- */
	  xSemaphoreTake(LCD_mutexHandle, portMAX_DELAY);

	  //  si on a déplacé le palet
	  if (x_old != (uint16_t)x)	{
		  taskENTER_CRITICAL();
		  x_palet = (uint16_t)x;
		  taskEXIT_CRITICAL();
		  // on efface le précédent
		  BSP_LCD_SetTextColor(BACKGROUND_COLOR);
		  BSP_LCD_FillCircle(x_old - w_palet/2, Y_PALET, H_PALET/2);
		  BSP_LCD_FillCircle(x_old + w_palet/2, Y_PALET, H_PALET/2);
		  BSP_LCD_FillRect(x_old - w_palet/2, Y_PALET - H_PALET/2, w_palet, H_PALET);

		  x_old = (uint16_t)x;
	  }

	  /* --- gestion agrandissement du palet --- */
	  if (time_new - t0 <= DELAY_EXTENSION_BONUS)	{
		  taskENTER_CRITICAL();
		  w_palet = PALET_EXTENSION_COEF*W_PALET;
		  if (w_palet%2 == 0)	w_palet ++; // on assure une largeur impaire de cette façon
		  taskEXIT_CRITICAL();
		  // correction de la position x après agrandissement (si le palet dépasse des bords de l'écran)
		  if (x - w_palet/2 < 1)	{
			  x = w_palet/2 + 1;
		  }
		  if (x + w_palet/2 > LCD_WIDTH)	{
			  x = LCD_WIDTH - w_palet/2;
		  }
	  }
	  else	{
		  // on efface le précédent palet plus grand lors du la fin du bonus d'agrandissement
		  BSP_LCD_SetTextColor(BACKGROUND_COLOR);
		  BSP_LCD_FillCircle(x_old - w_palet/2, Y_PALET, H_PALET/2);
		  BSP_LCD_FillCircle(x_old + w_palet/2, Y_PALET, H_PALET/2);
		  BSP_LCD_FillRect(x_old - w_palet/2, Y_PALET - H_PALET/2, w_palet, H_PALET);

		  taskENTER_CRITICAL();
		  w_palet = W_PALET;
		  taskEXIT_CRITICAL();
	  }
	  /* --- */

	  // on affiche à la nouvelle position
	  BSP_LCD_SetTextColor(PALET_COLOR);
	  BSP_LCD_FillCircle((uint16_t)x - w_palet/2, Y_PALET, H_PALET/2);
	  BSP_LCD_FillCircle((uint16_t)x + w_palet/2, Y_PALET, H_PALET/2);
	  BSP_LCD_FillRect((uint16_t)x - w_palet/2, Y_PALET - H_PALET/2, w_palet, H_PALET);

	  xSemaphoreGive(LCD_mutexHandle);
	  /* --- */

	  osDelay(10);
  }
  /* USER CODE END palet_ */
}

/* USER CODE BEGIN Header_bille_ */
/**
* @brief Function implementing the bille thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_bille_ */
void bille_(void const * argument)
{
  /* USER CODE BEGIN bille_ */
	float x = LCD_WIDTH/2;
	float y = LCD_HEIGHT - 2*(LCD_HEIGHT - Y_PALET);
	uint16_t x_old = x;
	uint16_t y_old = y;
	float x_dir;
	float y_dir; //x_dir^2 + y_dir^2 = 1
	float vel = INITIAL_BILLE_VELOCITY;
	uint32_t time_old = 0;
	uint32_t time_new;
	uint8_t on_palet_flag_value = 1;
	uint16_t xO; //coordonnées du coin de la brique où le contact a lieu
	uint16_t yO;

	/* --- variables associées aux briques --- */
	uint8_t n_x = LCD_WIDTH/BRICK_WIDTH; // number of bricks on x axis
	uint8_t n_y = LCD_HEIGHT/BRICK_HEIGHT; // number of bricks on y axis
	uint16_t pas_x = LCD_WIDTH/n_x; // pas entre le centre de deux briques consécutives (selon x)
	uint16_t pas_y = LCD_HEIGHT/n_y; // pas entre le centre de deux briques consécutives (selon y)
	/* --- */

	taskENTER_CRITICAL();
	time_old = get_time(&hrtc, &time, &date);
	taskEXIT_CRITICAL();

	x_dir = (2*((int)time_old % 2) - 1)*(float)(time_old & 0xFF)/(10/7*0xFF); // nombre 'pseudo' aléatoire de départ entre -0.72 et 0.72 (i.e -/+ 45°)
	y_dir = -sqrtf(1-x_dir*x_dir);
  /* Infinite loop */
  for(;;)
  {
	 vel += 0.000005;

	  /* --- récupération du temps depuis l'affichage de la position précédente --- */
	  taskENTER_CRITICAL();
	  time_new = get_time(&hrtc, &time, &date);
	  taskEXIT_CRITICAL();
	  /* --- */

	x += x_dir*vel*(time_new-time_old);
	y += y_dir*vel*(time_new-time_old);
	time_old = time_new;

	/* --- gestion des rebonds sur les bords --- */
	if (y - RAD_BALL <= 0)	{ // rebond bord du haut
		y = RAD_BALL;
		y_dir = -y_dir;
	}
	if (y - RAD_BALL > LCD_HEIGHT)	{ // game over
		game_over = 1;
		kill_all = 1;
	}
	if ((x - RAD_BALL <= 0) || (x + RAD_BALL >= LCD_WIDTH))	{ // rebond bord gauche ou droit
		if (x - RAD_BALL < 0){
			x = RAD_BALL;
		}
		if (x + RAD_BALL > LCD_WIDTH)	{
			x = LCD_WIDTH - RAD_BALL;
		}
		x_dir = -x_dir;
	}
	/* --- */

	/* --- gestion des rebonds sur le palet --- */
	if ((x >= x_palet - w_palet/2) && (x <= x_palet + w_palet/2))	{
		// rebond dessus du palet
		if ((y + RAD_BALL >= Y_PALET - H_PALET/2) && (y + RAD_BALL <= Y_PALET + H_PALET/2))	{
			xQueueSend(on_paletHandle, &on_palet_flag_value, portMAX_DELAY);
			y = Y_PALET - H_PALET/2 - RAD_BALL;
			// si le palet n'est pas sur les extrémités gauche ou droite de l'écran
			if ((x_palet != w_palet/2 + H_PALET/2) && (x_palet != LCD_WIDTH - w_palet/2 - H_PALET/2 - 1))	{
				x_dir += K_INFLUENCE_ON_DIRECTION*palet_vel;
			}
			if (x_dir < -MAX_X_Y_DIRECTION)	x_dir = -MAX_X_Y_DIRECTION;
			if (x_dir > MAX_X_Y_DIRECTION)	x_dir = MAX_X_Y_DIRECTION;
			y_dir = -sqrtf(1-x_dir*x_dir);
		}
	}
	/* --- */

	/* --- gestion des rebonds sur les briques --- */
	for (uint16_t i = 0; i < n_x; i++)	{
		for (uint16_t j = 0; j < n_y; j++)	{
			if (mat [i] [j] != 0)	{ // si la brique existe à l'emplacement (i,j)
				if ((x - (pas_x/2 + i*pas_x) >= -(BRICK_WIDTH/2 + RAD_BALL)) && (x - (pas_x/2 + i*pas_x) <= (BRICK_WIDTH/2 + RAD_BALL)))	{ // condition de contact selon x
					if ((y - (pas_y/2 + j*pas_y) >= -(BRICK_HEIGHT/2 + RAD_BALL)) && (y - (pas_y/2 + j*pas_y) <= (BRICK_HEIGHT/2 + RAD_BALL)))	{ // condition de contact selon y
						// ici, la bille est en contact avec la brique (i,j)

						// destruction de la brique
						if (mat [i] [j] == BRICK_BONUS_COLOR_EXT)	{
							taskENTER_CRITICAL();
							t0 = get_time (&hrtc, &time, &date); // démarrage extension du palet
							taskEXIT_CRITICAL();
						}
						mat [i] [j] = 0;
						xSemaphoreTake(LCD_mutexHandle, portMAX_DELAY);
						BSP_LCD_SetTextColor(BACKGROUND_COLOR);
						BSP_LCD_FillRect(pas_x/2 + i*pas_x - BRICK_WIDTH/2, pas_y/2 + j*pas_y - BRICK_HEIGHT/2, BRICK_WIDTH, BRICK_HEIGHT);
						xSemaphoreGive(LCD_mutexHandle);
						score_value++;

						if ((x >= pas_x/2 + i*pas_x - BRICK_WIDTH/2) && (x <= pas_x/2 + i*pas_x + BRICK_WIDTH/2))	{ // contact haut/bas sur la brique
							if ((y - RAD_BALL < pas_y/2 + j*pas_y + BRICK_HEIGHT/2) && (y - RAD_BALL > pas_y/2 + j*pas_y - BRICK_HEIGHT/2))	{ //contact bas sur la brique
								y = pas_y/2 + j*pas_y + BRICK_HEIGHT/2 + RAD_BALL;
							}
							else if ((y + RAD_BALL > pas_y/2 + j*pas_y - BRICK_HEIGHT/2) && (y + RAD_BALL < pas_y/2 + j*pas_y + BRICK_HEIGHT/2))	{ //contact haut sur la brique
								y = pas_y/2 + j*pas_y - BRICK_HEIGHT/2 - RAD_BALL;
							}
							y_dir = -y_dir;
						}
						else if ((y >= pas_y/2 + j*pas_y - BRICK_HEIGHT/2) && (y <= pas_y/2 + j*pas_y + BRICK_HEIGHT/2))	{ // contact latéral gauche/droit sur la brique
							if ((x - RAD_BALL < pas_x/2 + i*pas_x + BRICK_WIDTH/2) && (x - RAD_BALL > pas_x/2 + i*pas_x - BRICK_WIDTH/2))	{ //contact droit sur la brique
								x = pas_x/2 + i*pas_x + BRICK_WIDTH/2 + RAD_BALL;
							}
							else if ((x + RAD_BALL > pas_x/2 + i*pas_x - BRICK_WIDTH/2) && (x + RAD_BALL < pas_x/2 + i*pas_x + BRICK_WIDTH/2))	{ //contact gauche sur la brique
								x = pas_x/2 + i*pas_x - BRICK_WIDTH/2 - RAD_BALL;
							}
							x_dir = -x_dir;
						}
						else	{ // contact sur un "coin"
							// calcul de la nouvelle direction de la trajectoire suite au rebond sur le coin + replacement de la bille suite au contact détecté
							if (x > pas_x/2 + i*pas_x + BRICK_WIDTH/2)	{ // contact sur coin droit supérieur ou inférieur
								xO = pas_x/2 + i*pas_x + BRICK_WIDTH/2;
								if (y > pas_y/2 + j*pas_y + BRICK_HEIGHT/2)	{ // contact coin droit inférieur
									yO = pas_y/2 + j*pas_y + BRICK_HEIGHT/2;
									x_dir = (x - xO)/sqrtf((x - xO)*(x - xO) + (y - yO)*(y - yO));
									if (x_dir > MAX_X_Y_DIRECTION)	x_dir = MAX_X_Y_DIRECTION;
									if (x_dir < sqrtf(1-MAX_X_Y_DIRECTION*MAX_X_Y_DIRECTION)) x_dir = sqrtf(1-MAX_X_Y_DIRECTION*MAX_X_Y_DIRECTION);
									y_dir = sqrtf(1 - x_dir*x_dir);

									y = yO + sqrtf(2)/2*RAD_BALL;

								}
								if (y < pas_y/2 + j*pas_y - BRICK_HEIGHT/2)	{ // contact coin droit supérieur
									yO = pas_y/2 + j*pas_y - BRICK_HEIGHT/2;
									x_dir = (x - xO)/sqrtf((x - xO)*(x - xO) + (y - yO)*(y - yO));
									if (x_dir > MAX_X_Y_DIRECTION)	x_dir = MAX_X_Y_DIRECTION;
									if (x_dir < sqrtf(1-MAX_X_Y_DIRECTION*MAX_X_Y_DIRECTION)) x_dir = sqrtf(1-MAX_X_Y_DIRECTION*MAX_X_Y_DIRECTION);
									y_dir = -sqrtf(1 - x_dir*x_dir);
									y = yO - sqrtf(2)/2*RAD_BALL;
								}
								x = xO + sqrtf(2)/2*RAD_BALL;
							}
							if (x < pas_x/2 + i*pas_x - BRICK_WIDTH/2)	{ // contact sur coin gauche supérieur inférieur
								xO = pas_x/2 + i*pas_x - BRICK_WIDTH/2;
								if (y > pas_y/2 + j*pas_y + BRICK_HEIGHT/2)	{ // contact coin gauche inférieur
									yO = pas_y/2 + j*pas_y + BRICK_HEIGHT/2;
									x_dir = (x - xO)/sqrtf((x - xO)*(x - xO) + (y - yO)*(y - yO));
									if (x_dir > MAX_X_Y_DIRECTION)	x_dir = MAX_X_Y_DIRECTION;
									if (x_dir < sqrtf(1-MAX_X_Y_DIRECTION*MAX_X_Y_DIRECTION)) x_dir = sqrtf(1-MAX_X_Y_DIRECTION*MAX_X_Y_DIRECTION);
									y_dir = sqrtf(1 - x_dir*x_dir);
									y = yO + sqrtf(2)/2*RAD_BALL;
								}
								if (y < pas_y/2 + j*pas_y - BRICK_HEIGHT/2)	{ // contact coin gauche supérieur
									yO = pas_y/2 + j*pas_y - BRICK_HEIGHT/2;
									x_dir = (x - xO)/sqrtf((x - xO)*(x - xO) + (y - yO)*(y - yO));
									if (x_dir > MAX_X_Y_DIRECTION)	x_dir = MAX_X_Y_DIRECTION;
									if (x_dir < sqrtf(1-MAX_X_Y_DIRECTION*MAX_X_Y_DIRECTION)) x_dir = sqrtf(1-MAX_X_Y_DIRECTION*MAX_X_Y_DIRECTION);
									y_dir = -sqrtf(1 - x_dir*x_dir);
									y = yO - sqrtf(2)/2*RAD_BALL;
								}
								x = xO - sqrtf(2)/2*RAD_BALL;
							}
						}
					}
				}
			}
		}
	}
	/* --- */

	xSemaphoreTake(LCD_mutexHandle, portMAX_DELAY);

	/* --- effaçage ancienne position bille --- */
	if (((uint16_t)x != x_old) || ((uint16_t)y != y_old))	{
		BSP_LCD_SetTextColor(BACKGROUND_COLOR);
		//BSP_LCD_SetBackColor(BACKGROUND_COLOR);
		BSP_LCD_FillCircle(x_old, y_old, RAD_BALL);
	}
	/* --- */

	/* --- affichage nouvelle position bille --- */
	BSP_LCD_SetTextColor(BALL_COLOR);
	//BSP_LCD_SetBackColor(BALL_COLOR);
	BSP_LCD_FillCircle((uint16_t)x, (uint16_t)y, RAD_BALL);
	/* --- */

	xSemaphoreGive(LCD_mutexHandle);

	x_old = x;
	y_old = y;

	osDelay(3);
  }
  /* USER CODE END bille_ */
}

/* USER CODE BEGIN Header_game_over_tsk_ */
/**
* @brief Function implementing the game_over_tsk thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_game_over_tsk_ */
void game_over_tsk_(void const * argument)
{
  /* USER CODE BEGIN game_over_tsk_ */
  /* Infinite loop */
  for(;;)
  {
	  if (kill_all)	{
		  vTaskDelete(billeHandle);
		  vTaskDelete(paletHandle);
		  vTaskDelete(bricksHandle);
		  BSP_LCD_Clear(BACKGROUND_COLOR);
		  kill_all = 0;
	  }
	  if (create_all)	{
		  	taskENTER_CRITICAL();
		  	score_value = 0;
		  	t0 = -DELAY_EXTENSION_BONUS;
			/* definition and creation of palet */
			osThreadDef(palet, palet_, osPriorityNormal, 0, 1024);
			paletHandle = osThreadCreate(osThread(palet), NULL);

			/* definition and creation of bille */
			osThreadDef(bille, bille_, osPriorityNormal, 0, 1024);
			billeHandle = osThreadCreate(osThread(bille), NULL);

			/* definition and creation of bricks */
			osThreadDef(bricks, bricks_, osPriorityNormal, 0, 4096);
			bricksHandle = osThreadCreate(osThread(bricks), NULL);

			create_all = 0;
			game_over = 0;

			BSP_LCD_Clear(BACKGROUND_COLOR);
			taskEXIT_CRITICAL();
	  }
	  // affichage écran game_over
	  if (game_over)	{
		  BSP_LCD_SetBackColor(BACKGROUND_COLOR);
		  BSP_LCD_SetTextColor(SCORE_COLOR);
		  BSP_LCD_DisplayStringAt(0, LCD_HEIGHT/2 - 10, "G A M E  O V E R", CENTER_MODE);
		  BSP_LCD_DisplayStringAt(0, LCD_HEIGHT/2 + 10, "Press BP1 to play again", CENTER_MODE);
	  }
	  osDelay(500);
  }
  /* USER CODE END game_over_tsk_ */
}

/* USER CODE BEGIN Header_display_score_ */
/**
* @brief Function implementing the display_score thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_display_score_ */
void display_score_(void const * argument)
{
  /* USER CODE BEGIN display_score_ */
	char txt [40];
  /* Infinite loop */
  for(;;)
  {
	  sprintf (txt, "Score : %4u", score_value);

	  xSemaphoreTake(LCD_mutexHandle, portMAX_DELAY);

	  BSP_LCD_SetTextColor(SCORE_COLOR);
	  BSP_LCD_DisplayStringAt(0, LCD_HEIGHT - 15, txt, RIGHT_MODE);

	  xSemaphoreGive(LCD_mutexHandle);

    osDelay(200);
  }
  /* USER CODE END display_score_ */
}

/* USER CODE BEGIN Header_bricks_ */
/**
* @brief Function implementing the bricks thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_bricks_ */
void bricks_(void const * argument)
{
  /* USER CODE BEGIN bricks_ */
	uint8_t n_x = LCD_WIDTH/BRICK_WIDTH; // number of bricks on x axis
	uint8_t n_y = LCD_HEIGHT/BRICK_HEIGHT; // number of bricks on y axis
	uint16_t pas_x = LCD_WIDTH/n_x; // pas entre le centre de deux briques consécutives (selon x)
	uint16_t pas_y = LCD_HEIGHT/n_y; // pas entre le centre de deux briques consécutives (selon y)
	for (uint16_t i = 0; i < n_x; i++)	{
		for (uint16_t j = 0; j < n_y; j++)	{
			mat [i] [j] = 0;
		}
	}
	uint32_t time_old = 0;
	uint32_t time_new;
	uint8_t on_palet_flag = 0; // vrai lorsque la bille vient de taper le palet

	uint32_t rand_nbr;

	taskENTER_CRITICAL();
	time_old = get_time(&hrtc, &time, &date);
	taskEXIT_CRITICAL();

  /* Infinite loop */
  for(;;)
  {
	  /* --- récupération du temps actuel --- */
	  taskENTER_CRITICAL();
	  time_new = get_time(&hrtc, &time, &date);
	  taskEXIT_CRITICAL();
	  /* --- */

	  /* --- décalage des briques vers le bas et affichage --- */
	  on_palet_flag = 0;
	  xQueueReceive(on_paletHandle, &on_palet_flag, 1);
	  if ((time_new - time_old >= TRANSITION_DELAY) && (on_palet_flag))	{
		  // décalage puis affichage de la ligne n_y à la ligne 2
		  for (uint16_t i = 0; i < n_x; i++)	{
			  for (uint16_t j = n_y - 1; j >= 1; j--)	{
				  if (mat [i] [j] != mat [i] [j-1])	{ // si il y a un changement de brique lors du décalage (couleur différente inclue)
					  mat [i] [j] = mat [i] [j-1]; // mise à jour de la valeur
					  xSemaphoreTake(LCD_mutexHandle, portMAX_DELAY);
					  if (mat [i] [j] != 0)	{ // affichage brique
						  display_brick(pas_x/2 + i*pas_x - BRICK_WIDTH/2, pas_y/2 + j*pas_y - BRICK_HEIGHT/2, BRICK_WIDTH, BRICK_HEIGHT, mat [i] [j]);
					  }
					  else	{ // effaçage brique
						  BSP_LCD_SetTextColor(BACKGROUND_COLOR);
						  BSP_LCD_FillRect(pas_x/2 + i*pas_x - BRICK_WIDTH/2, pas_y/2 + j*pas_y - BRICK_HEIGHT/2, BRICK_WIDTH, BRICK_HEIGHT);
					  }
					  xSemaphoreGive(LCD_mutexHandle);
				  }
				  if ((mat [i] [j] != 0) && (pas_y/2 + j*pas_y >= Y_PALET - (H_PALET/2 + 2*RAD_BALL + BRICK_HEIGHT/2)))	{
					  game_over = 1;
					  kill_all = 1;
				  }
			  }
		  }
		  // création de la nouvelle ligne (ligne 0) et affichage
		  HAL_RNG_GenerateRandomNumber(&hrng, &rand_nbr);
		  for (uint16_t i = 0; i < n_x; i++)	{
			  mat [i] [0] = (rand_nbr>>i)&1; // mise à jour (soit 0 soit 1 (une couleur sera choisie))
			  if (mat [i] [0] == 1)	{ // choix aléatoire couleur
				  switch (rand_nbr&0b0111)	{
					  case 0 :
						  mat [i] [0] = LCD_COLOR_BLUE;
					  break;
					  case 1 :
						  mat [i] [0] = LCD_COLOR_GRAY;
					  break;
					  case 2:
						  mat [i] [0] = LCD_COLOR_GREEN;
					  break;
					  case 3 :
						  mat [i] [0] = LCD_COLOR_MAGENTA;
					  break;
					  case 4 :
						  mat [i] [0] = LCD_COLOR_RED;
					  break;
					  case 5 :
						  mat [i] [0] = LCD_COLOR_WHITE;
					  break;
					  case 6 :
						  mat [i] [0] = LCD_COLOR_YELLOW;
					  break;
					  case 7 :
						  mat [i] [0] = LCD_COLOR_CYAN;
					  break;
					  default :
					  break;
				  }
			  }
			  if ((rand_nbr&(0b1100<<28))>>28 == 0b1100)	{ // une chance sur 4 d'avoir une brique bonus dans la nouvelle ligne de brique pour élargir le palet
				  if (i == (rand_nbr&(n_x-1)))	{ // choix aléatoire de la brique bonus sur la nouvelle ligne
					  mat [i] [0] = BRICK_BONUS_COLOR_EXT;
				  }
			  }

			  xSemaphoreTake(LCD_mutexHandle, portMAX_DELAY);
			  if (mat [i] [0] != 0)	{ // affichage brique
				  display_brick(pas_x/2 + i*pas_x - BRICK_WIDTH/2, pas_y/2 - BRICK_HEIGHT/2, BRICK_WIDTH, BRICK_HEIGHT, mat [i] [0]);
			  }
			  else 	{ // effaçage brique
				  BSP_LCD_SetTextColor(BACKGROUND_COLOR);
				  BSP_LCD_FillRect(pas_x/2 + i*pas_x - BRICK_WIDTH/2, pas_y/2 - BRICK_HEIGHT/2, BRICK_WIDTH, BRICK_HEIGHT);
			  }
			  xSemaphoreGive(LCD_mutexHandle);
		  }
		  time_old = time_new;
	  }
	  /* ---------------------------------------- */
    osDelay(10);
  }
  /* USER CODE END bricks_ */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

