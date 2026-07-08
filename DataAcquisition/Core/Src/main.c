/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define YES 1
#define NO 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
SAI_HandleTypeDef hsai_BlockA2;
SAI_HandleTypeDef hsai_BlockB2;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;
DMA_HandleTypeDef hdma_sai2_a;
DMA_HandleTypeDef hdma_sai2_b;

SD_HandleTypeDef hsd1;

/* USER CODE BEGIN PV */

/* ----- BUFFER SECTION  ----- */
#define BUFFER_SIZE 4096*4	//Circular Buffer Array Size

volatile uint8_t IS_BUFFER_HALF_FULL_SAI1 = NO;
volatile uint8_t IS_BUFFER_FULL_SAI1      = NO;

volatile uint8_t IS_BUFFER_HALF_FULL_SAI2 = NO;
volatile uint8_t IS_BUFFER_FULL_SAI2      = NO;

/* ----- SAI 2 - BUFFERS PARA SAI C y D ----- */

uint32_t audio_buffer_SAI2A[BUFFER_SIZE];
uint32_t audio_buffer_SAI2B[BUFFER_SIZE];

/* ----- SAI 1 - BUFFERS PARA SAI A y B ----- */

uint32_t audio_buffer_SAI1A[BUFFER_SIZE];
uint32_t audio_buffer_SAI1B[BUFFER_SIZE];

/* ----- Debug Variables ----- */
HAL_StatusTypeDef err_A2 = HAL_ERROR, err_B2 = HAL_ERROR;

/* ----- SD Card Section ----- */

HAL_SD_CardInfoTypeDef mi_tarjeta_sd;
uint8_t sd_estado = 0; // 1 = Éxito, 0 = Error

uint32_t i = 0;

uint32_t ndtr = 0;
uint32_t datos_nuevos_pre = 0;
uint32_t datos_nuevos_post = 0;

volatile uint8_t button_pressed = 0;

/* 1. Declaras la variable del tipo de dato correcto */
HAL_StatusTypeDef estado_inicializacion;

FATFS fs;         // Objeto del sistema de archivos
FIL fil_A;          // Objeto del archivo
FIL fil_B;          // Objeto del archivo
FIL fil_C;          // Objeto del archivo
FIL fil_D;          // Objeto del archivo
FRESULT res_A, res_B,res_C, res_D;      // Variable para guardar el resultado de las operaciones
UINT bytesWrittenA, bytesWrittenB,bytesWrittenC,bytesWrittenD; // Para saber cuántos bytes se escribieron realmente

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SAI1_Init(void);
static void MX_SAI2_Init(void);
/* USER CODE BEGIN PFP */

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_DMA_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_SAI1_Init();
  MX_SAI2_Init();
  /* USER CODE BEGIN 2 */

  // 1. Montar la tarjeta SD

  res_A = f_mount(&fs, "", 1);
	if (res_A == FR_OK) {
			// 2. Primer par de micrófonos
		    res_A = f_open(&fil_A, "mics_1_2.raw", FA_CREATE_ALWAYS | FA_WRITE);
		    // 3. Segundo par de micrófonos
		    res_B = f_open(&fil_B, "mics_3_4.raw", FA_CREATE_ALWAYS | FA_WRITE);

		    res_C = f_open(&fil_C, "mics_5_6.raw", FA_CREATE_ALWAYS | FA_WRITE);

		    res_D = f_open(&fil_D, "mics_7_8.raw", FA_CREATE_ALWAYS | FA_WRITE);


		    if (res_A != FR_OK || res_B != FR_OK || res_C != FR_OK || res_D != FR_OK) {
		            Error_Handler();
		        }


		if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
		    {
			Error_Handler();
		    }
	} else {

		Error_Handler();
	}

	// Inicializar el esclavo interno, luego el maestro

	HAL_SAI_Receive_DMA(&hsai_BlockA2, (uint8_t *)audio_buffer_SAI2A, BUFFER_SIZE);
	HAL_SAI_Receive_DMA(&hsai_BlockB2, (uint8_t *)audio_buffer_SAI2B, BUFFER_SIZE);

	HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t *)audio_buffer_SAI1A, BUFFER_SIZE);
	HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t *)audio_buffer_SAI1B, BUFFER_SIZE);

  // Se pretende realizar una primera adquisición y desechar
  while(IS_BUFFER_HALF_FULL_SAI1 == NO);

  IS_BUFFER_HALF_FULL_SAI1 = NO;
  IS_BUFFER_HALF_FULL_SAI2 = NO;
  IS_BUFFER_FULL_SAI1 = NO;
  IS_BUFFER_FULL_SAI2 = NO;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  		// --- EVENTO 1: MITAD DE BUFFER ---
  		// Usamos SAI1 como el reloj maestro del evento; si él está a la mitad, SAI2 también lo está.
  		if(IS_BUFFER_HALF_FULL_SAI1 == YES || IS_BUFFER_HALF_FULL_SAI2 == YES) {

  			// 1. Procesamiento y enmascaramiento de los 4 buffers en una sola pasada de CPU
  			for (uint32_t i = 0; i < (BUFFER_SIZE / 2); i++) {
  				audio_buffer_SAI1A[i] &= 0xFFFFFF00;
  				audio_buffer_SAI1B[i] &= 0xFFFFFF00;
  				audio_buffer_SAI2A[i] &= 0xFFFFFF00;
  				audio_buffer_SAI2B[i] &= 0xFFFFFF00;
  			}

  			// 2. Escritura consecutiva en ráfaga a la SD de los 4 archivos
  			res_A = f_write(&fil_A, &audio_buffer_SAI1A[0], (BUFFER_SIZE / 2) * sizeof(uint32_t), &bytesWrittenA);
  			res_B = f_write(&fil_B, &audio_buffer_SAI1B[0], (BUFFER_SIZE / 2) * sizeof(uint32_t), &bytesWrittenB);
  			res_C = f_write(&fil_C, &audio_buffer_SAI2A[0], (BUFFER_SIZE / 2) * sizeof(uint32_t), &bytesWrittenC);
  			res_D = f_write(&fil_D, &audio_buffer_SAI2B[0], (BUFFER_SIZE / 2) * sizeof(uint32_t), &bytesWrittenD);

  			// 3. Verificación conjunta de errores
  			if (res_A != FR_OK || res_B != FR_OK || res_C != FR_OK || res_D != FR_OK ||
  				bytesWrittenA == 0 || bytesWrittenB == 0 || bytesWrittenC == 0 || bytesWrittenD == 0) {
  				Error_Handler(); // Si el archivo 5, 6, 7 u 8 falla, aquí se detendrá el código en modo Debug
  			}

  			// 4. Bajamos ambas banderas simultáneamente
  			IS_BUFFER_HALF_FULL_SAI1 = NO;
  			IS_BUFFER_HALF_FULL_SAI2 = NO;
  		}

  		// --- EVENTO 2: BUFFER COMPLETO ---
  		if(IS_BUFFER_FULL_SAI1 == YES || IS_BUFFER_FULL_SAI2 == YES) {

  			// 1. Procesamiento de la segunda mitad de los 4 buffers
  			for (uint32_t i = BUFFER_SIZE / 2; i < BUFFER_SIZE; i++) {
  				audio_buffer_SAI1A[i] &= 0xFFFFFF00;
  				audio_buffer_SAI1B[i] &= 0xFFFFFF00;
  				audio_buffer_SAI2A[i] &= 0xFFFFFF00;
  				audio_buffer_SAI2B[i] &= 0xFFFFFF00;
  			}

  			// 2. Escritura consecutiva en ráfaga a la SD
  			res_A = f_write(&fil_A, &audio_buffer_SAI1A[BUFFER_SIZE / 2], (BUFFER_SIZE / 2) * sizeof(uint32_t), &bytesWrittenA);
  			res_B = f_write(&fil_B, &audio_buffer_SAI1B[BUFFER_SIZE / 2], (BUFFER_SIZE / 2) * sizeof(uint32_t), &bytesWrittenB);
  			res_C = f_write(&fil_C, &audio_buffer_SAI2A[BUFFER_SIZE / 2], (BUFFER_SIZE / 2) * sizeof(uint32_t), &bytesWrittenC);
  			res_D = f_write(&fil_D, &audio_buffer_SAI2B[BUFFER_SIZE / 2], (BUFFER_SIZE / 2) * sizeof(uint32_t), &bytesWrittenD);

  			if (res_A != FR_OK || res_B != FR_OK || res_C != FR_OK || res_D != FR_OK ||
  				bytesWrittenA == 0 || bytesWrittenB == 0 || bytesWrittenC == 0 || bytesWrittenD == 0) {
  				Error_Handler();
  			}

  			// 3. Bajamos ambas banderas
  			IS_BUFFER_FULL_SAI1 = NO;
  			IS_BUFFER_FULL_SAI2 = NO;
  		}

  		// --- CONTROL DE BOTÓN (CIERRE SEGURO) ---
  		if (button_pressed) {
  			f_close(&fil_A);
  			f_close(&fil_B);
  			f_close(&fil_C);
  			f_close(&fil_D);
  			f_mount(NULL, "", 0);

  			HAL_SAI_DMAStop(&hsai_BlockA1);
  			HAL_SAI_DMAStop(&hsai_BlockB1);
  			HAL_SAI_DMAStop(&hsai_BlockA2);
  			HAL_SAI_DMAStop(&hsai_BlockB2);
  			break;
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

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_SAI2A
                              |RCC_PERIPHCLK_SAI2B;
  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 40;
  PeriphClkInitStruct.PLL3.PLL3P = 5;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL3;
  PeriphClkInitStruct.Sai2BClockSelection = RCC_SAI2BCLKSOURCE_PLL3;
  PeriphClkInitStruct.Sai2AClockSelection = RCC_SAI2ACLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_32;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_8K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_OUTBLOCKA_ENABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.PdmInit.Activation = DISABLE;
  hsai_BlockA1.Init.PdmInit.MicPairsNbr = 0;
  hsai_BlockA1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockA1.FrameInit.FrameLength = 64;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 32;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 2;
  hsai_BlockA1.SlotInit.SlotActive = 0x00000003;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB1.Init.DataSize = SAI_DATASIZE_32;
  hsai_BlockB1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB1.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_OUTBLOCKA_ENABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB1.Init.PdmInit.Activation = DISABLE;
  hsai_BlockB1.Init.PdmInit.MicPairsNbr = 0;
  hsai_BlockB1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockB1.FrameInit.FrameLength = 64;
  hsai_BlockB1.FrameInit.ActiveFrameLength = 32;
  hsai_BlockB1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockB1.SlotInit.FirstBitOffset = 0;
  hsai_BlockB1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockB1.SlotInit.SlotNumber = 2;
  hsai_BlockB1.SlotInit.SlotActive = 0x00000003;
  if (HAL_SAI_Init(&hsai_BlockB1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA2.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockA2.Init.DataSize = SAI_DATASIZE_32;
  hsai_BlockA2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA2.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  hsai_BlockA2.Init.Synchro = SAI_SYNCHRONOUS_EXT_SAI1;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA2.Init.PdmInit.Activation = DISABLE;
  hsai_BlockA2.Init.PdmInit.MicPairsNbr = 0;
  hsai_BlockA2.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockA2.FrameInit.FrameLength = 64;
  hsai_BlockA2.FrameInit.ActiveFrameLength = 32;
  hsai_BlockA2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA2.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockA2.SlotInit.FirstBitOffset = 0;
  hsai_BlockA2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA2.SlotInit.SlotNumber = 2;
  hsai_BlockA2.SlotInit.SlotActive = 0x00000003;
  if (HAL_SAI_Init(&hsai_BlockA2) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB2.Instance = SAI2_Block_B;
  hsai_BlockB2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB2.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB2.Init.DataSize = SAI_DATASIZE_32;
  hsai_BlockB2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB2.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  hsai_BlockB2.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB2.Init.PdmInit.Activation = DISABLE;
  hsai_BlockB2.Init.PdmInit.MicPairsNbr = 0;
  hsai_BlockB2.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockB2.FrameInit.FrameLength = 64;
  hsai_BlockB2.FrameInit.ActiveFrameLength = 32;
  hsai_BlockB2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB2.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockB2.SlotInit.FirstBitOffset = 0;
  hsai_BlockB2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockB2.SlotInit.SlotNumber = 2;
  hsai_BlockB2.SlotInit.SlotActive = 0x00000003;
  if (HAL_SAI_Init(&hsai_BlockB2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */

  /* USER CODE END SAI2_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */
    hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
    // 2. Inicializamos la tarjeta de forma segura en 1-Bit

    estado_inicializacion = HAL_SD_Init(&hsd1);
        if ( estado_inicializacion != HAL_OK)
        {
            Error_Handler();
        }
        //HAL_Delay(10);
        // 3. Una vez inicializada con éxito, AHORA SÍ le decimos a la tarjeta
        // y al STM32 que se muevan juntos a 4-Bits mediante la función nativa:
        if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
        {
            Error_Handler();
        }
        hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(B1_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(B1_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ----- SAI1 and SAI2 IRQ Section  -----*/

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
    if (hsai->Instance == SAI1_Block_A) {
    	IS_BUFFER_HALF_FULL_SAI1 = YES;
    }
    if (hsai->Instance == SAI2_Block_A) {
		IS_BUFFER_HALF_FULL_SAI2 = YES;
    }
}
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
    if (hsai->Instance == SAI1_Block_A) {
    	IS_BUFFER_FULL_SAI1 = YES;
    }
    if (hsai->Instance == SAI2_Block_A) {
		IS_BUFFER_FULL_SAI2 = YES;
	}
}


/* ----- Interface IRQ Section  -----*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 1. Verificamos cuál pin generó la interrupción
    if (GPIO_Pin == B1_Pin) // 'B1_Pin' es el nombre que le diste en CubeMX
    {
        // 2. Aquí pones tu lógica
        // Por ejemplo: cambiar una bandera para detener la grabación
    	button_pressed = YES;
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
