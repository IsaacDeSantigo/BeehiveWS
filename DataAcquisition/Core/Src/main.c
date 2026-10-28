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
typedef enum
{
  SAI_MODE_DMA,
  SAI_MODE_IT
} SAI_ModeTypedef;

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

MDMA_HandleTypeDef hmdma_mdma_channel0_dma1_stream0_tc_0;
MDMA_LinkNodeTypeDef node_mdma_channel0_dma1_stream0_tc_1;
MDMA_HandleTypeDef hmdma_mdma_channel1_dma1_stream1_tc_0;
MDMA_LinkNodeTypeDef node_mdma_channel1_dma1_stream1_tc_1;
MDMA_HandleTypeDef hmdma_mdma_channel2_dma1_stream2_tc_0;
MDMA_LinkNodeTypeDef node_mdma_channel2_dma1_stream2_tc_1;
MDMA_HandleTypeDef hmdma_mdma_channel3_dma1_stream3_tc_0;
MDMA_LinkNodeTypeDef node_mdma_channel3_dma1_stream3_tc_1;
/* USER CODE BEGIN PV */



/* ----- BUFFER SECTION  ----- */
#define BUFFER_SIZE 8192

volatile uint8_t IS_BUFFER_SAI1A_M0AR_FULL = NO;
volatile uint8_t IS_BUFFER_SAI1A_M1AR_FULL = NO;

volatile uint8_t IS_BUFFER_SAI1B_M0AR_FULL = NO;
volatile uint8_t IS_BUFFER_SAI1B_M1AR_FULL = NO;

volatile uint8_t IS_BUFFER_SAI2A_M0AR_FULL = NO;
volatile uint8_t IS_BUFFER_SAI2A_M1AR_FULL = NO;

volatile uint8_t IS_BUFFER_SAI2B_M0AR_FULL = NO;
volatile uint8_t IS_BUFFER_SAI2B_M1AR_FULL = NO;

/*
 *
 * This buffer are allocated in AHB_SRAM1 and AHB_SRAM2
 * 		Boundary Address 		Arm Cortex - M7
 * 0x3000 0000 - 0x3000 FFFF		AHB SRAM1
 * 0x3001 0000 - 0x3001 FFFF		AHB SRAM2
 *
 *														*/

/* ----- SAI 1 - BUFFERS FOR SAI A y B ----- */

__attribute__((section(".audio_buffers"))) uint32_t audio_buffer_SAI1A_M0AR[BUFFER_SIZE/2];
__attribute__((section(".audio_buffers"))) uint32_t audio_buffer_SAI1A_M1AR[BUFFER_SIZE/2];

__attribute__((section(".audio_buffers"))) uint32_t audio_buffer_SAI1B_M0AR[BUFFER_SIZE/2];
__attribute__((section(".audio_buffers"))) uint32_t audio_buffer_SAI1B_M1AR[BUFFER_SIZE/2];

/* ----- SAI 2 - BUFFERS FOR SAI C y D ----- */

__attribute__((section(".audio_buffers"))) uint32_t audio_buffer_SAI2A_M0AR[BUFFER_SIZE/2];
__attribute__((section(".audio_buffers"))) uint32_t audio_buffer_SAI2A_M1AR[BUFFER_SIZE/2];

__attribute__((section(".audio_buffers"))) uint32_t audio_buffer_SAI2B_M0AR[BUFFER_SIZE/2];
__attribute__((section(".audio_buffers"))) uint32_t audio_buffer_SAI2B_M1AR[BUFFER_SIZE/2];

/*
 *
 * This buffer are allocated in AXI SRAM1 and AXI SRAM 2
 * 		Boundary Address 		Arm Cortex - M7
 * 0x2400 0000 - 0x2403 FFFF		AXI SRAM1
 * 0x2404 0000 - 0x2407 FFFF		AXI SRAM2
 *
 *														*/

/* ----- Global Buffer ----- */

__attribute__((section(".axi_ring_buffer"))) uint32_t audio_global_buffer_ping[BUFFER_SIZE*8];
__attribute__((section(".axi_ring_buffer"))) uint32_t audio_global_buffer_pong[BUFFER_SIZE*8];

/* ----- Nodes Definition ----- */

__attribute__((section(".axi_ring_buffer"))) __ALIGN_BEGIN MDMA_LinkNodeTypeDef mdma_nodes[32] __ALIGN_END;

/* Arreglo de punteros a los Handles del MDMA */

MDMA_HandleTypeDef* const mdma_handles[4] = {
    &hmdma_mdma_channel0_dma1_stream0_tc_0,
    &hmdma_mdma_channel1_dma1_stream1_tc_0,
    &hmdma_mdma_channel2_dma1_stream2_tc_0,
    &hmdma_mdma_channel3_dma1_stream3_tc_0
};

/* Arreglos de punteros a los Búferes Origen (DMA1) */
uint32_t* const src_ping[4] = {
    audio_buffer_SAI1A_M0AR, audio_buffer_SAI1B_M0AR,
    audio_buffer_SAI2A_M0AR, audio_buffer_SAI2B_M0AR
};

uint32_t* const src_pong[4] = {
    audio_buffer_SAI1A_M1AR, audio_buffer_SAI1B_M1AR,
    audio_buffer_SAI2A_M1AR, audio_buffer_SAI2B_M1AR
};

/* Variables de control para la Tarjeta SD */
volatile uint8_t  mdma_node_counter   = 0;
volatile uint8_t  sd_write_ready_flag = 0;
volatile uint32_t sd_buffer_to_write  = 0;

uint8_t sd_buffer_to_write_test = 0;

/* ----- Interface Section ----- */

volatile uint8_t is_sd_ready = NO;
volatile uint8_t was_button_pressed = NO;

/* ----- State Machine ----- */

typedef enum{
	IDLE,
	SETTING_PERIPHERALS,
	RECORDING,
	STOP_RECORDING
} t_state;

t_state state = IDLE;

/* ----- Debug Variables ----- */
HAL_StatusTypeDef err_A2 = HAL_ERROR, err_B2 = HAL_ERROR;

volatile uint32_t ndtr_A1, ndtr_B1, ndtr_A2, ndtr_B2;
volatile uint32_t recibidos_A1, recibidos_B1, recibidos_A2, recibidos_B2;

uint32_t bytes_to_write = (BUFFER_SIZE*8) * sizeof(uint32_t);


uint32_t dt = 0, t1 = 0, t2 = 0, dt_max = 0;

UINT bw; // Variable genérica para los bytes escritos

/* ----- SD Card Section ----- */

uint32_t i = 0;

uint32_t ndtr = 0;
uint32_t datos_nuevos_pre = 0;
uint32_t datos_nuevos_post = 0;


/* 1. Declaras la variable del tipo de dato correcto */
HAL_StatusTypeDef estado_inicializacion;

//FATFS fs;         // Objeto del sistema de archivos
FATFS fs __attribute__((aligned(32)));
//FIL fil_ALL;          // Objeto del archivo
FIL fil_ALL __attribute__((aligned(32)));

FRESULT res_ALL = FR_LOCKED;
FRESULT res_A = FR_LOCKED;    // Variable para almacenar el estado de las operaciones de FatFs

FRESULT SD_Status = FR_OK;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_MDMA_Init(void);
static void MX_SAI1_Init(void);
static void MX_SAI2_Init(void);
static void MX_SDMMC1_SD_Init(void);
/* USER CODE BEGIN PFP */

HAL_StatusTypeDef HAL_SAI_Receive_DMA_MultiBuffer(SAI_HandleTypeDef *hsai, uint32_t DstAddress, uint32_t SecondMemAddress, uint16_t Size );
uint32_t Custom_SAI_InterruptFlag(const SAI_HandleTypeDef *hsai, SAI_ModeTypedef mode);
void Custom_SAI_DMARxCpltM0(DMA_HandleTypeDef *hdma);
void Custom_SAI_DMARxCpltM1(DMA_HandleTypeDef *hdma);
void Configurar_Topologia_Ping_Pong_MDMA(void);
void Arrancar_Sistema_MDMA(void);

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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_MDMA_Init();
  MX_SAI1_Init();
  MX_SAI2_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		// 1. Obtenemos el valor crudo del registro NDTR de los 4 canales (Datos restantes)
		ndtr_A1 = __HAL_DMA_GET_COUNTER(hsai_BlockA1.hdmarx);
		ndtr_B1 = __HAL_DMA_GET_COUNTER(hsai_BlockB1.hdmarx);
		ndtr_A2 = __HAL_DMA_GET_COUNTER(hsai_BlockA2.hdmarx);
		ndtr_B2 = __HAL_DMA_GET_COUNTER(hsai_BlockB2.hdmarx);

		// 2. Calculamos los datos que ya aterrizaron exitosamente en la RAM
		recibidos_A1 = BUFFER_SIZE - ndtr_A1;
		recibidos_B1 = BUFFER_SIZE - ndtr_B1;
		recibidos_A2 = BUFFER_SIZE - ndtr_A2;
		recibidos_B2 = BUFFER_SIZE - ndtr_B2;

	  switch( state ){
	  	  case IDLE:

			HAL_GPIO_WritePin(LD_GREEN_GPIO_Port,LD_GREEN_Pin, GPIO_PIN_SET);

			if (was_button_pressed == YES){
				was_button_pressed = NO;

				HAL_GPIO_WritePin(GPIOB, LD_RED_Pin|LD_YELLOW_Pin, GPIO_PIN_RESET);

				state = SETTING_PERIPHERALS;
			}

	  		break;
	  	  case SETTING_PERIPHERALS:

	  		HAL_GPIO_WritePin(LD_GREEN_GPIO_Port,LD_GREEN_Pin, GPIO_PIN_SET);

	  		/* ----- SD Card Initialization  ----- */

			SD_Status = f_mount(&fs, "", 1);

			if (SD_Status == FR_OK) {

				is_sd_ready = (f_open(&fil_ALL, "test.raw", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) ? NO : YES;

			} else {
				is_sd_ready = NO;
			}


			if( is_sd_ready == YES ){

				/* ----- MDMA Initialization ----- */

				Configurar_Topologia_Ping_Pong_MDMA();
				Arrancar_Sistema_MDMA();

				/* ----- SAI1 and SAI2 Initialization ----- */

				HAL_SAI_Receive_DMA_MultiBuffer(
					&hsai_BlockA2,
					(uint32_t)audio_buffer_SAI2A_M0AR,
					(uint32_t)audio_buffer_SAI2A_M1AR,
					BUFFER_SIZE/2
				);

				HAL_SAI_Receive_DMA_MultiBuffer(
					&hsai_BlockB2,
					(uint32_t)audio_buffer_SAI2B_M0AR,
					(uint32_t)audio_buffer_SAI2B_M1AR,
					BUFFER_SIZE/2
				);

				HAL_SAI_Receive_DMA_MultiBuffer(
					&hsai_BlockB1,
					(uint32_t)audio_buffer_SAI1B_M0AR,
					(uint32_t)audio_buffer_SAI1B_M1AR,
					BUFFER_SIZE/2
				);

				HAL_SAI_Receive_DMA_MultiBuffer(
					&hsai_BlockA1,
					(uint32_t)audio_buffer_SAI1A_M0AR,
					(uint32_t)audio_buffer_SAI1A_M1AR,
					BUFFER_SIZE/2
				);

				HAL_GPIO_WritePin(LD_GREEN_GPIO_Port,LD_GREEN_Pin, GPIO_PIN_RESET);
				state = RECORDING;
			} else {
				HAL_GPIO_WritePin(GPIOB,LD_RED_Pin, GPIO_PIN_SET);
				state = IDLE;
			}

	  		  break;
	  	  case RECORDING:

			if(sd_write_ready_flag){

				sd_write_ready_flag = 0;
				if(sd_buffer_to_write_test == 0){
					HAL_GPIO_TogglePin(GPIOB,LD_YELLOW_Pin);
					t2 = DWT->CYCCNT;

					res_A = f_write(&fil_ALL,&audio_global_buffer_ping[0], bytes_to_write, &bw);

					t1 = DWT->CYCCNT;
					dt = t1 - t2;

					if( dt > dt_max){
						dt_max = dt;
					}
					sd_buffer_to_write_test = 1;
				} else {

			  		HAL_GPIO_TogglePin(GPIOB,LD_YELLOW_Pin);

					res_A = f_write(&fil_ALL,&audio_global_buffer_pong[0], bytes_to_write, &bw);
					sd_buffer_to_write_test = 0;

				}

			}

			if (was_button_pressed == YES){
				was_button_pressed = NO;
				HAL_GPIO_WritePin(GPIOB,LD_YELLOW_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LD_GREEN_GPIO_Port,LD_GREEN_Pin, GPIO_PIN_SET);
				state = STOP_RECORDING;
			}

	  		  break;

	  	  case STOP_RECORDING:

	  		f_close(&fil_ALL);
			f_mount(NULL, "", 0);

			HAL_SAI_DMAStop(&hsai_BlockA1);
			HAL_SAI_DMAStop(&hsai_BlockB1);
			HAL_SAI_DMAStop(&hsai_BlockA2);
			HAL_SAI_DMAStop(&hsai_BlockB2);

			state = IDLE;

	  		  break;

	  	  default:
	  		  state = IDLE;
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
  RCC_OscInitStruct.PLL.PLLN = 35;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
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
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_SAI2A
                              |RCC_PERIPHCLK_SAI2B;
  PeriphClkInitStruct.PLL3.PLL3M = 4;
  PeriphClkInitStruct.PLL3.PLL3N = 8;
  PeriphClkInitStruct.PLL3.PLL3P = 1;
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
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_HF;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_8K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_OUTBLOCKA_ENABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.PdmInit.Activation = DISABLE;
  hsai_BlockA1.Init.PdmInit.MicPairsNbr = 0;
  hsai_BlockA1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockA1.FrameInit.FrameLength = 64;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 32;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
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
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_HF;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_OUTBLOCKA_ENABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB1.Init.PdmInit.Activation = DISABLE;
  hsai_BlockB1.Init.PdmInit.MicPairsNbr = 0;
  hsai_BlockB1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockB1.FrameInit.FrameLength = 64;
  hsai_BlockB1.FrameInit.ActiveFrameLength = 32;
  hsai_BlockB1.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockB1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockB1.SlotInit.FirstBitOffset = 0;
  hsai_BlockB1.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
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
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_HF;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA2.Init.PdmInit.Activation = DISABLE;
  hsai_BlockA2.Init.PdmInit.MicPairsNbr = 0;
  hsai_BlockA2.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockA2.FrameInit.FrameLength = 64;
  hsai_BlockA2.FrameInit.ActiveFrameLength = 32;
  hsai_BlockA2.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockA2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA2.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockA2.SlotInit.FirstBitOffset = 0;
  hsai_BlockA2.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
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
  hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_HF;
  hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB2.Init.PdmInit.Activation = DISABLE;
  hsai_BlockB2.Init.PdmInit.MicPairsNbr = 0;
  hsai_BlockB2.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockB2.FrameInit.FrameLength = 64;
  hsai_BlockB2.FrameInit.ActiveFrameLength = 32;
  hsai_BlockB2.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockB2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB2.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockB2.SlotInit.FirstBitOffset = 0;
  hsai_BlockB2.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
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
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * Enable MDMA controller clock
  * Configure MDMA for global transfers
  *   hmdma_mdma_channel0_dma1_stream0_tc_0
  *   node_mdma_channel0_dma1_stream0_tc_1
  *   hmdma_mdma_channel1_dma1_stream1_tc_0
  *   node_mdma_channel1_dma1_stream1_tc_1
  *   hmdma_mdma_channel2_dma1_stream2_tc_0
  *   node_mdma_channel2_dma1_stream2_tc_1
  *   hmdma_mdma_channel3_dma1_stream3_tc_0
  *   node_mdma_channel3_dma1_stream3_tc_1
  */
static void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */
  MDMA_LinkNodeConfTypeDef nodeConfig;

  /* Configure MDMA channel MDMA_Channel0 */
  /* Configure MDMA request hmdma_mdma_channel0_dma1_stream0_tc_0 on MDMA_Channel0 */
  hmdma_mdma_channel0_dma1_stream0_tc_0.Instance = MDMA_Channel0;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.Request = MDMA_REQUEST_DMA1_Stream0_TC;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.Priority = MDMA_PRIORITY_VERY_HIGH;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.SourceInc = MDMA_SRC_INC_WORD;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.DestinationInc = MDMA_DEST_INC_WORD;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_WORD;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_WORD;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.DataAlignment = MDMA_DATAALIGN_LEFT;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_32BEATS;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.DestBurst = MDMA_DEST_BURST_32BEATS;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel0_dma1_stream0_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel0_dma1_stream0_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel0_dma1_stream0_tc_0, 0x40020008, 0x00000020) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize MDMA link node according to specified parameters */
  nodeConfig.Init.Request = MDMA_REQUEST_DMA1_Stream0_TC;
  nodeConfig.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  nodeConfig.Init.Priority = MDMA_PRIORITY_VERY_HIGH;
  nodeConfig.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  nodeConfig.Init.SourceInc = MDMA_SRC_INC_WORD;
  nodeConfig.Init.DestinationInc = MDMA_DEST_INC_WORD;
  nodeConfig.Init.SourceDataSize = MDMA_SRC_DATASIZE_WORD;
  nodeConfig.Init.DestDataSize = MDMA_DEST_DATASIZE_WORD;
  nodeConfig.Init.DataAlignment = MDMA_DATAALIGN_LEFT;
  nodeConfig.Init.BufferTransferLength = 128;
  nodeConfig.Init.SourceBurst = MDMA_SOURCE_BURST_32BEATS;
  nodeConfig.Init.DestBurst = MDMA_DEST_BURST_32BEATS;
  nodeConfig.Init.SourceBlockAddressOffset = 0;
  nodeConfig.Init.DestBlockAddressOffset = 0;
  nodeConfig.PostRequestMaskAddress = 0x40020008;
  nodeConfig.PostRequestMaskData = 0x00000020;
  nodeConfig.SrcAddress = 0;
  nodeConfig.DstAddress = 0;
  nodeConfig.BlockDataLength = 16384;
  nodeConfig.BlockCount = 1;
  if (HAL_MDMA_LinkedList_CreateNode(&node_mdma_channel0_dma1_stream0_tc_1, &nodeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN mdma_channel0_dma1_stream0_tc_1 */


  /* USER CODE END mdma_channel0_dma1_stream0_tc_1 */

  /* Connect a node to the linked list */
  if (HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel0_dma1_stream0_tc_0, &node_mdma_channel0_dma1_stream0_tc_1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Make the linked list circular by connecting the last node to the first */
  if (HAL_MDMA_LinkedList_EnableCircularMode(&hmdma_mdma_channel0_dma1_stream0_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel1 */
  /* Configure MDMA request hmdma_mdma_channel1_dma1_stream1_tc_0 on MDMA_Channel1 */
  hmdma_mdma_channel1_dma1_stream1_tc_0.Instance = MDMA_Channel1;
  hmdma_mdma_channel1_dma1_stream1_tc_0.Init.Request = MDMA_REQUEST_DMA1_Stream1_TC;
  hmdma_mdma_channel1_dma1_stream1_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel1_dma1_stream1_tc_0.Init.Priority = MDMA_PRIORITY_HIGH;
  hmdma_mdma_channel1_dma1_stream1_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel1_dma1_stream1_tc_0.Init.SourceInc = MDMA_SRC_INC_WORD;
  hmdma_mdma_channel1_dma1_stream1_tc_0.Init.DestinationInc = MDMA_DEST_INC_WORD;
  hmdma_mdma_channel1_dma1_stream1_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_WORD;
  hmdma_mdma_channel1_dma1_stream1_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_WORD;
  hmdma_mdma_channel1_dma1_stream1_tc_0.Init.DataAlignment = MDMA_DATAALIGN_LEFT;
  hmdma_mdma_channel1_dma1_stream1_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel1_dma1_stream1_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_32BEATS;
  hmdma_mdma_channel1_dma1_stream1_tc_0.Init.DestBurst = MDMA_DEST_BURST_32BEATS;
  hmdma_mdma_channel1_dma1_stream1_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel1_dma1_stream1_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel1_dma1_stream1_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel1_dma1_stream1_tc_0, 0x40020008, 0x00000800) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize MDMA link node according to specified parameters */
  nodeConfig.Init.Request = MDMA_REQUEST_DMA1_Stream1_TC;
  nodeConfig.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  nodeConfig.Init.Priority = MDMA_PRIORITY_HIGH;
  nodeConfig.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  nodeConfig.Init.SourceInc = MDMA_SRC_INC_WORD;
  nodeConfig.Init.DestinationInc = MDMA_DEST_INC_WORD;
  nodeConfig.Init.SourceDataSize = MDMA_SRC_DATASIZE_WORD;
  nodeConfig.Init.DestDataSize = MDMA_DEST_DATASIZE_WORD;
  nodeConfig.Init.DataAlignment = MDMA_DATAALIGN_LEFT;
  nodeConfig.Init.BufferTransferLength = 128;
  nodeConfig.Init.SourceBurst = MDMA_SOURCE_BURST_32BEATS;
  nodeConfig.Init.DestBurst = MDMA_DEST_BURST_32BEATS;
  nodeConfig.Init.SourceBlockAddressOffset = 0;
  nodeConfig.Init.DestBlockAddressOffset = 0;
  nodeConfig.PostRequestMaskAddress = 0x40020008;
  nodeConfig.PostRequestMaskData = 0x00000800;
  nodeConfig.SrcAddress = 0;
  nodeConfig.DstAddress = 0;
  nodeConfig.BlockDataLength = 16384;
  nodeConfig.BlockCount = 1;
  if (HAL_MDMA_LinkedList_CreateNode(&node_mdma_channel1_dma1_stream1_tc_1, &nodeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN mdma_channel1_dma1_stream1_tc_1 */

  /* USER CODE END mdma_channel1_dma1_stream1_tc_1 */

  /* Connect a node to the linked list */
  if (HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel1_dma1_stream1_tc_0, &node_mdma_channel1_dma1_stream1_tc_1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Make the linked list circular by connecting the last node to the first */
  if (HAL_MDMA_LinkedList_EnableCircularMode(&hmdma_mdma_channel1_dma1_stream1_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel2 */
  /* Configure MDMA request hmdma_mdma_channel2_dma1_stream2_tc_0 on MDMA_Channel2 */
  hmdma_mdma_channel2_dma1_stream2_tc_0.Instance = MDMA_Channel2;
  hmdma_mdma_channel2_dma1_stream2_tc_0.Init.Request = MDMA_REQUEST_DMA1_Stream2_TC;
  hmdma_mdma_channel2_dma1_stream2_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel2_dma1_stream2_tc_0.Init.Priority = MDMA_PRIORITY_HIGH;
  hmdma_mdma_channel2_dma1_stream2_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel2_dma1_stream2_tc_0.Init.SourceInc = MDMA_SRC_INC_WORD;
  hmdma_mdma_channel2_dma1_stream2_tc_0.Init.DestinationInc = MDMA_DEST_INC_WORD;
  hmdma_mdma_channel2_dma1_stream2_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_WORD;
  hmdma_mdma_channel2_dma1_stream2_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_WORD;
  hmdma_mdma_channel2_dma1_stream2_tc_0.Init.DataAlignment = MDMA_DATAALIGN_LEFT;
  hmdma_mdma_channel2_dma1_stream2_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel2_dma1_stream2_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_32BEATS;
  hmdma_mdma_channel2_dma1_stream2_tc_0.Init.DestBurst = MDMA_DEST_BURST_32BEATS;
  hmdma_mdma_channel2_dma1_stream2_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel2_dma1_stream2_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel2_dma1_stream2_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel2_dma1_stream2_tc_0, 0x40020008, 0x00200000) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize MDMA link node according to specified parameters */
  nodeConfig.Init.Request = MDMA_REQUEST_DMA1_Stream2_TC;
  nodeConfig.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  nodeConfig.Init.Priority = MDMA_PRIORITY_HIGH;
  nodeConfig.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  nodeConfig.Init.SourceInc = MDMA_SRC_INC_WORD;
  nodeConfig.Init.DestinationInc = MDMA_DEST_INC_WORD;
  nodeConfig.Init.SourceDataSize = MDMA_SRC_DATASIZE_WORD;
  nodeConfig.Init.DestDataSize = MDMA_DEST_DATASIZE_WORD;
  nodeConfig.Init.DataAlignment = MDMA_DATAALIGN_LEFT;
  nodeConfig.Init.BufferTransferLength = 128;
  nodeConfig.Init.SourceBurst = MDMA_SOURCE_BURST_32BEATS;
  nodeConfig.Init.DestBurst = MDMA_DEST_BURST_32BEATS;
  nodeConfig.Init.SourceBlockAddressOffset = 0;
  nodeConfig.Init.DestBlockAddressOffset = 0;
  nodeConfig.PostRequestMaskAddress = 0x40020008;
  nodeConfig.PostRequestMaskData = 0x00200000;
  nodeConfig.SrcAddress = 0;
  nodeConfig.DstAddress = 0;
  nodeConfig.BlockDataLength = 16384;
  nodeConfig.BlockCount = 1;
  if (HAL_MDMA_LinkedList_CreateNode(&node_mdma_channel2_dma1_stream2_tc_1, &nodeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN mdma_channel2_dma1_stream2_tc_1 */

  /* USER CODE END mdma_channel2_dma1_stream2_tc_1 */

  /* Connect a node to the linked list */
  if (HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel2_dma1_stream2_tc_0, &node_mdma_channel2_dma1_stream2_tc_1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Make the linked list circular by connecting the last node to the first */
  if (HAL_MDMA_LinkedList_EnableCircularMode(&hmdma_mdma_channel2_dma1_stream2_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel3 */
  /* Configure MDMA request hmdma_mdma_channel3_dma1_stream3_tc_0 on MDMA_Channel3 */
  hmdma_mdma_channel3_dma1_stream3_tc_0.Instance = MDMA_Channel3;
  hmdma_mdma_channel3_dma1_stream3_tc_0.Init.Request = MDMA_REQUEST_DMA1_Stream3_TC;
  hmdma_mdma_channel3_dma1_stream3_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel3_dma1_stream3_tc_0.Init.Priority = MDMA_PRIORITY_HIGH;
  hmdma_mdma_channel3_dma1_stream3_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel3_dma1_stream3_tc_0.Init.SourceInc = MDMA_SRC_INC_WORD;
  hmdma_mdma_channel3_dma1_stream3_tc_0.Init.DestinationInc = MDMA_DEST_INC_WORD;
  hmdma_mdma_channel3_dma1_stream3_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_WORD;
  hmdma_mdma_channel3_dma1_stream3_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_WORD;
  hmdma_mdma_channel3_dma1_stream3_tc_0.Init.DataAlignment = MDMA_DATAALIGN_LEFT;
  hmdma_mdma_channel3_dma1_stream3_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel3_dma1_stream3_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_32BEATS;
  hmdma_mdma_channel3_dma1_stream3_tc_0.Init.DestBurst = MDMA_DEST_BURST_32BEATS;
  hmdma_mdma_channel3_dma1_stream3_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel3_dma1_stream3_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel3_dma1_stream3_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel3_dma1_stream3_tc_0, 0x40020008, 0x08000000) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize MDMA link node according to specified parameters */
  nodeConfig.Init.Request = MDMA_REQUEST_DMA1_Stream3_TC;
  nodeConfig.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  nodeConfig.Init.Priority = MDMA_PRIORITY_HIGH;
  nodeConfig.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  nodeConfig.Init.SourceInc = MDMA_SRC_INC_WORD;
  nodeConfig.Init.DestinationInc = MDMA_DEST_INC_WORD;
  nodeConfig.Init.SourceDataSize = MDMA_SRC_DATASIZE_WORD;
  nodeConfig.Init.DestDataSize = MDMA_DEST_DATASIZE_WORD;
  nodeConfig.Init.DataAlignment = MDMA_DATAALIGN_LEFT;
  nodeConfig.Init.BufferTransferLength = 128;
  nodeConfig.Init.SourceBurst = MDMA_SOURCE_BURST_32BEATS;
  nodeConfig.Init.DestBurst = MDMA_DEST_BURST_32BEATS;
  nodeConfig.Init.SourceBlockAddressOffset = 0;
  nodeConfig.Init.DestBlockAddressOffset = 0;
  nodeConfig.PostRequestMaskAddress = 0x40020008;
  nodeConfig.PostRequestMaskData = 0x08000000;
  nodeConfig.SrcAddress = 0;
  nodeConfig.DstAddress = 0;
  nodeConfig.BlockDataLength = 16384;
  nodeConfig.BlockCount = 1;
  if (HAL_MDMA_LinkedList_CreateNode(&node_mdma_channel3_dma1_stream3_tc_1, &nodeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN mdma_channel3_dma1_stream3_tc_1 */

  /* USER CODE END mdma_channel3_dma1_stream3_tc_1 */

  /* Connect a node to the linked list */
  if (HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel3_dma1_stream3_tc_0, &node_mdma_channel3_dma1_stream3_tc_1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Make the linked list circular by connecting the last node to the first */
  if (HAL_MDMA_LinkedList_EnableCircularMode(&hmdma_mdma_channel3_dma1_stream3_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD_RED_Pin|LD_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD_GREEN_GPIO_Port, LD_GREEN_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : LD_RED_Pin LD_YELLOW_Pin */
  GPIO_InitStruct.Pin = LD_RED_Pin|LD_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD_GREEN_Pin */
  GPIO_InitStruct.Pin = LD_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD_GREEN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(B1_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(B1_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


HAL_StatusTypeDef HAL_SAI_Receive_DMA_MultiBuffer(SAI_HandleTypeDef *hsai, uint32_t DstAddress, uint32_t SecondMemAddress, uint16_t Size ){

	if ((Size == 0U))
	{
		return  HAL_ERROR;
	}

	if(hsai->State == HAL_SAI_STATE_READY)
	{
		__HAL_LOCK(hsai);

		hsai->XferSize = Size;					/*!< SAI transfer size */
		hsai->XferCount = Size;					/*!< SAI transfer counter */
		hsai->ErrorCode = HAL_SAI_ERROR_NONE;	/*!< SAI Error code */
		hsai->State = HAL_SAI_STATE_BUSY_RX;	/*!< SAI communication state */

		/* Set the SAI Rx DMA transfer complete callback */
		hsai->hdmarx->XferCpltCallback = Custom_SAI_DMARxCpltM0;
		hsai->hdmarx->XferM1CpltCallback = Custom_SAI_DMARxCpltM1;

		/* Set the DMA error callback */
		hsai->hdmarx->XferErrorCallback = NULL;

		/* Set the DMA Rx abort callback */
		hsai->hdmarx->XferAbortCallback = NULL;

	    /* Enable the Rx DMA Stream */
		if (HAL_DMAEx_MultiBufferStart_IT(hsai->hdmarx, (uint32_t)&hsai->Instance->DR, DstAddress, SecondMemAddress, hsai->XferSize) != HAL_OK)
		{
		  __HAL_UNLOCK(hsai);
		  return  HAL_ERROR;
		}

		/* Enable the interrupts for error handling */
		__HAL_SAI_ENABLE_IT(hsai, Custom_SAI_InterruptFlag(hsai, SAI_MODE_DMA));

		/* Enable SAI Rx DMA Request */
		hsai->Instance->CR1 |= SAI_xCR1_DMAEN;

		/* Check if the SAI is already enabled */
		if ((hsai->Instance->CR1 & SAI_xCR1_SAIEN) == 0U)
		{
		  /* Enable SAI peripheral */
		  __HAL_SAI_ENABLE(hsai);
		}

		/* Process Unlocked */
		__HAL_UNLOCK(hsai);

		return HAL_OK;

	}
	else
	{
		return HAL_BUSY;
	}

}

void Custom_SAI_DMARxCpltM0(DMA_HandleTypeDef *hdma)
{
	SAI_HandleTypeDef *hsai = (SAI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

	if (hsai->Instance == SAI1_Block_A) {
		IS_BUFFER_SAI1A_M0AR_FULL = YES;
	}
	else if (hsai->Instance == SAI1_Block_B) {
		IS_BUFFER_SAI1B_M0AR_FULL = YES;
	}
	else if (hsai->Instance == SAI2_Block_A) {
		IS_BUFFER_SAI2A_M0AR_FULL = YES;
	}
	else if (hsai->Instance == SAI2_Block_B) {
		IS_BUFFER_SAI2B_M0AR_FULL = YES;
	}
}

void Custom_SAI_DMARxCpltM1(DMA_HandleTypeDef *hdma)
{
	SAI_HandleTypeDef *hsai = (SAI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

	if (hsai->Instance == SAI1_Block_A) {
		IS_BUFFER_SAI1A_M1AR_FULL = YES;
	}
	else if (hsai->Instance == SAI1_Block_B) {
		IS_BUFFER_SAI1B_M1AR_FULL = YES;
	}
	else if (hsai->Instance == SAI2_Block_A) {
		IS_BUFFER_SAI2A_M1AR_FULL = YES;
	}
	else if (hsai->Instance == SAI2_Block_B) {
		IS_BUFFER_SAI2B_M1AR_FULL = YES;
	}
}


uint32_t Custom_SAI_InterruptFlag(const SAI_HandleTypeDef *hsai, SAI_ModeTypedef mode)
{
  uint32_t tmpIT = SAI_IT_OVRUDR;

  if (mode == SAI_MODE_IT)
  {
    tmpIT |= SAI_IT_FREQ;
  }

  if ((hsai->Init.Protocol == SAI_AC97_PROTOCOL) &&
      ((hsai->Init.AudioMode == SAI_MODESLAVE_RX) || (hsai->Init.AudioMode == SAI_MODEMASTER_RX)))
  {
    tmpIT |= SAI_IT_CNRDY;
  }

  if ((hsai->Init.AudioMode == SAI_MODESLAVE_RX) || (hsai->Init.AudioMode == SAI_MODESLAVE_TX))
  {
    tmpIT |= SAI_IT_AFSDET | SAI_IT_LFSDET;
  }
  else
  {
    /* hsai has been configured in master mode */
    tmpIT |= SAI_IT_WCKCFG;
  }
  return tmpIT;
}


void Configurar_Topologia_Ping_Pong_MDMA(void)
{
    MDMA_LinkNodeConfTypeDef nodeConfig = {0};
    const uint32_t dma1_clear_masks[4] = {0x00000020, 0x00000800, 0x00200000, 0x08000000};

    /* NUEVO: Arreglo de Requests específicos del DMAMUX */
        const uint32_t mdma_requests[4] = {
            MDMA_REQUEST_DMA1_Stream0_TC,
            MDMA_REQUEST_DMA1_Stream1_TC,
            MDMA_REQUEST_DMA1_Stream2_TC,
            MDMA_REQUEST_DMA1_Stream3_TC
        };


    /* 1. Configuración Base del Nodo (Constante para todos) */
    /* El parámetro "Request" se sobrescribirá por el del Handle interno, pero lo dejamos genérico */
    nodeConfig.Init.TransferTriggerMode  = MDMA_BLOCK_TRANSFER;
    nodeConfig.Init.Endianness           = MDMA_LITTLE_ENDIANNESS_PRESERVE;
    nodeConfig.Init.SourceInc            = MDMA_SRC_INC_WORD;
    nodeConfig.Init.DestinationInc       = MDMA_DEST_INC_WORD;
    nodeConfig.Init.SourceDataSize       = MDMA_SRC_DATASIZE_WORD;
    nodeConfig.Init.DestDataSize         = MDMA_DEST_DATASIZE_WORD;
    nodeConfig.Init.DataAlignment        = MDMA_DATAALIGN_PACKENABLE;
    nodeConfig.Init.BufferTransferLength = 128;
    nodeConfig.Init.SourceBurst          = MDMA_SOURCE_BURST_16BEATS;
    nodeConfig.Init.DestBurst            = MDMA_DEST_BURST_16BEATS;
    nodeConfig.Init.SourceBlockAddressOffset = 0;
    nodeConfig.Init.DestBlockAddressOffset = 0;

    nodeConfig.BlockDataLength           = 16384; /* 16 KB por disparo */
    nodeConfig.BlockCount                = 1;     /* 1 solo bloque por disparo (hardware espera N-1) */

    /* 2. Construcción Topológica */
    for (uint8_t ch = 0; ch < 4; ch++)
    {
        /* "Amnesia" al HAL: Borramos cualquier nodo autogenerado por el IDE */
        mdma_handles[ch]->FirstLinkedListNodeAddress = 0;
        mdma_handles[ch]->LastLinkedListNodeAddress  = 0;
        mdma_handles[ch]->LinkedListNodeCounter      = 0;

        /* --- NUEVO: Asignación dinámica de Prioridad en el Bus AXI --- */
		if (ch == 0) {
			nodeConfig.Init.Priority = MDMA_PRIORITY_VERY_HIGH; /* Canal 0 (SAI 1A Maestro) */
		} else {
			nodeConfig.Init.Priority = MDMA_PRIORITY_HIGH;      /* Canales 1, 2 y 3 (Esclavos) */
		}

	    nodeConfig.Init.Request = mdma_requests[ch];
        nodeConfig.PostRequestMaskAddress = 0x40020008; /* DMA1_LIFCR */
        nodeConfig.PostRequestMaskData    = dma1_clear_masks[ch];

        for (uint8_t n = 0; n < 8; n++)
        {
            uint8_t global_node_idx = (ch * 8) + n;

            /* A) Lógica de Origen (Alternar Ping/Pong del DMA1) */
            /* Si 'n' es par (0, 2, 4, 6), leemos del búfer Ping. Si es impar, del Pong. */
            nodeConfig.SrcAddress = (n % 2 == 0) ? (uint32_t)src_ping[ch] : (uint32_t)src_pong[ch];

            /* B) Lógica de Destino (AXI SRAM Intercalada) */
            uint32_t base_dest;
            uint8_t  local_n;

            if (n < 4) {
                /* Nodos 0 a 3 apuntan a la Mitad A (Ping Global) */
                base_dest = (uint32_t)audio_global_buffer_ping;
                local_n = n;
            } else {
                /* Nodos 4 a 7 apuntan a la Mitad B (Pong Global) */
                base_dest = (uint32_t)audio_global_buffer_pong;
                local_n = n - 4; // Re-escalamos de 0 a 3 para el offset
            }

            /* Cálculo de Offsets (En Bytes) */
            uint32_t offset_salto = local_n * 65536; // 64 KB de salto temporal por cada ciclo de 4 canales
            uint32_t offset_canal = ch * 16384;      // 16 KB de separación espacial entre SAIs

            nodeConfig.DstAddress = base_dest + offset_salto + offset_canal;

            /* C) Creación e Inserción Física */
            HAL_MDMA_LinkedList_CreateNode(&mdma_nodes[global_node_idx], &nodeConfig);
            HAL_MDMA_LinkedList_AddNode(mdma_handles[ch], &mdma_nodes[global_node_idx], 0);
        }

        /* D) Cerrar el círculo para este canal */
        HAL_MDMA_LinkedList_EnableCircularMode(mdma_handles[ch]);
    }

    /* 3. Mantenimiento de Caché (Obligatorio) */
    /* Si .ram_cd es cacheable en tu configuración MPU, forzamos la escritura en RAM física */
//    SCB_CleanDCache_by_Addr((uint32_t*)mdma_nodes, sizeof(mdma_nodes));
}

void Arrancar_Sistema_MDMA(void)
{
    for (uint8_t ch = 0; ch < 4; ch++)
    {
        MDMA_Channel_TypeDef* ch_regs = mdma_handles[ch]->Instance;
        uint8_t nodo_cero_idx = ch * 8; // El índice donde vive el Nodo 0 de este canal

        /* 1. Limpiar cualquier bandera de interrupción residual (Seguridad) */
        ch_regs->CIFCR = MDMA_CIFCR_CTEIF | MDMA_CIFCR_CCTCIF | MDMA_CIFCR_CBRTIF | MDMA_CIFCR_CBTIF | MDMA_CIFCR_CTEIF;

        /* 2. Cargar físicamente los parámetros del Nodo 0 en el silicio */
        ch_regs->CTCR   = mdma_nodes[nodo_cero_idx].CTCR;
        ch_regs->CBNDTR = mdma_nodes[nodo_cero_idx].CBNDTR;
        ch_regs->CSAR   = mdma_nodes[nodo_cero_idx].CSAR;
        ch_regs->CDAR   = mdma_nodes[nodo_cero_idx].CDAR;
        ch_regs->CBRUR  = mdma_nodes[nodo_cero_idx].CBRUR;
        ch_regs->CMAR   = mdma_nodes[nodo_cero_idx].CMAR;
        ch_regs->CMDR   = mdma_nodes[nodo_cero_idx].CMDR;

        /* 3. Apuntar el Link Register (CLAR) hacia el Nodo 1 */
        /* Como ya enlazamos los nodos con el HAL, el Nodo 0 ya tiene guardada la dirección del Nodo 1 en su propio CLAR */
        ch_regs->CLAR   = mdma_nodes[nodo_cero_idx].CLAR;

        /* --- NUEVO: Configurar interrupciones SOLO para el Canal Maestro (0) --- */
		if (ch == 0)
		{
			/* Activar Transfer Complete Interrupt (TCIE) y Transfer Error (TEIE) Modificar comentario */
			ch_regs->CCR |= (MDMA_CCR_BTIE | MDMA_CCR_TEIE);

			/* Habilitar la línea de interrupción global del MDMA en el NVIC (Controlador de Interrupciones) */
			/* Nota: Asegúrate de configurar una prioridad más baja (número más alto)
			   que las interrupciones del sistema operativo si usas FreeRTOS */
			HAL_NVIC_SetPriority(MDMA_IRQn, 5, 0);
			HAL_NVIC_EnableIRQ(MDMA_IRQn);
		}

        /* 4. Activar el canal (Se queda dormido escuchando al DMAMUX) */
        ch_regs->CCR |= MDMA_CCR_EN;
    }
}

/* ----- Interface IRQ Section  -----*/
//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == B1_Pin)
    {
    	was_button_pressed = YES;
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
