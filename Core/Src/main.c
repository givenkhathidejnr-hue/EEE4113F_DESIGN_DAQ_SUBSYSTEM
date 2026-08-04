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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "daq.h"
#include "string.h"
#include <stdio.h>
#include "stdbool.h"

#include "INA219.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Buffer for raw ADC samples (Oversampling/Averaging) */
#define ADC_WINDOW_SIZE 256
#define BUFFER_SIZE 100
FaultType result;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CRC_HandleTypeDef hcrc;
DAC_HandleTypeDef hdac1;
SMBUS_HandleTypeDef hsmbus1;
I2C_HandleTypeDef hi2c2;
IWDG_HandleTypeDef hiwdg;
RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* ── Extern globals defined in daq.c ─────────────────────────────────── */
extern DaqContext    	g_daq;
extern FaultManager  	g_fault_mgr;
extern PendingFault 	g_pending_fault;
extern AcqState     	g_acq;
extern uint8_t 			s_buf_head;

/* ── Pending record passed from SYS_PROCESS to SYS_LOG ──────────────── */
static MeasurementRecord s_pending_record;

/* Alert Response Address (ARA) */
static uint8_t s_ara_response_addr;

/* INA219 context structure */
INA219_t ina219 = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C1_SMBUS_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_CRC_Init(void);
static void MX_IWDG_Init(void);
static void MX_I2C2_Init(void);
void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}


/* ══════════════════════════════════════════════════════════════════════
 *  EXTI dispatch 
 *
 *  Routing each pin to its dedicated callback 
 * ══════════════════════════════════════════════════════════════════════ */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		  case GPIO_PIN_9:   /* PB9 — detection domain load switch fault  */
			  DAQ_EXTI_Detection_Callback();
			  break;
		  case GPIO_PIN_6:    /* PB6  — excitation domain load switch fault */
			  DAQ_EXTI_Excitation_Callback();
			  break;
		  case GPIO_PIN_7:    /* PA7  — SD card domain load switch fault    */
			  DAQ_EXTI_SDCard_Callback();
			  break;
		  default:
			  break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        DAQ_TIM_PeriodElapsed_Callback();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        DAQ_ADC_ConvCplt_Callback();
    }
}

/**
 * @brief SMBus Error Callback
 * Triggered by the I2C1_ER_IRQn when the SMBA (Alert) line is pulled low.
 */
void HAL_SMBUS_ErrorCallback(SMBUS_HandleTypeDef *hsmbus)
{
    if (hsmbus->Instance == I2C1) {
        /* Check if the SMBus Alert flag is set */
        if (__HAL_SMBUS_GET_FLAG(hsmbus, SMBUS_FLAG_ALERT) != RESET) {

            /* 1. Start the Alert Response Address (ARA) sequence. */
            HAL_SMBUS_Master_Receive_IT(hsmbus, 0x0C, &s_ara_response_addr, 1, SMBUS_FIRST_AND_LAST_FRAME_NO_PEC);

            /* 2. Clear the flag so the interrupt doesn't re-trigger immediately */
            __HAL_SMBUS_CLEAR_FLAG(hsmbus, SMBUS_FLAG_ALERT);
        }
    }
}

/**
 * @brief SMBus Master Rx Complete Callback
 * Triggered once the ARA response (the alerting device's address) is received.
 */
void HAL_SMBUS_MasterRxCpltCallback(SMBUS_HandleTypeDef *hsmbus)
{
    if (hsmbus->Instance == I2C1) {
        /* The response contains the 7-bit address of the device that triggered the alert.
         * MCP9808 address is 0x18.*/
        if ((s_ara_response_addr >> 1) == (MCP9808_I2C_ADDR >> 1)) {
            /* Latch the fault. The state machine will handle this at the next check. */
            g_pending_fault.type      = FAULT_TEMP_ALERT;
            g_pending_fault.subsystem = SUBSYS_DETECTION;

            /* If we are in the middle of a high-precision acquisition, mark it as faulted */
            if (g_acq.phase != ACQ_IDLE) {
                g_acq.fault = FAULT_TEMP_ALERT;
            }
        }
    }
}

/**
  * @brief  Wake Up Timer callback
  * @param  hrtc: RTC handle
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	/* Called when CPU wakes from RTC wakeup interrupt */
	(void)hrtc;
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
  /* ── Cold-boot detection ──────────────────────────────────────────
   * RCC_FLAG_BORRST (brown-out reset), not PWR_FLAG_SB: this system
   * only ever uses Stop 2 mode, never Standby, so PWR_FLAG_SB (set only
   * on a wake-from-Standby) would never be set regardless of reset
   * cause -- that check would zero g_daq/g_fault_mgr/etc. on every
   * single reset, including an IWDG watchdog reset, defeating the whole
   * point of keeping them in .noinit across exactly that case.
   * RCC_FLAG_BORRST is set specifically by a genuine power-on/brown-out
   * event and stays clear across a watchdog/software/pin reset, so it
   * actually distinguishes "fresh power-on" the way this check needs.
   */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET) {
	  memset(&g_daq,           0, sizeof(DaqContext));
	  memset(&g_fault_mgr,     0, sizeof(FaultManager));
	  memset(&g_pending_fault, 0, sizeof(PendingFault));
	  memset(&g_acq,           0, sizeof(AcqState));
	  memset(&s_pending_record,0, sizeof(MeasurementRecord));

	  s_buf_head 		= 0;
	  g_daq.state 		= SYS_INIT;
	  g_daq.cycle_count = 1;
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_I2C1_SMBUS_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_IWDG_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* Configure MCP9808 alert limits */
  FaultType alert_cfg = configure_mcp9808_alert();

  /* Non fatal if configuration failed */
  if (alert_cfg != FAULT_NONE) {
	  FaultLog boot_warn = {
		  .timestamp      = 0u,
		  .fault          = alert_cfg,
		  .subsystem      = SUBSYS_TEMP_SENSOR,
		  .state_at_fault = SYS_INIT,
		  .retry_count    = 0u
	  };
	  log_fault_to_sd(&boot_warn);
  }

  // Enable mcp alert interrupts
  HAL_SMBUS_EnableAlert_IT(&hsmbus1);

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* GLOBAL FAULT CHECK: If a fault was latched by EXTI, handle it now */
	  if (g_pending_fault.type != FAULT_NONE) {
		  fault_handler(g_pending_fault.type, g_pending_fault.subsystem);
		  continue; // Restart the loop
	  }


	  /* ── State machine ──────────────────────────────────────────────── */

	  switch (g_daq.state)
	  {
		  /* ── SYS_INIT ─────────────────────────────────────────────
		   * kick the watchdog, then run the health check. A clean check advances to power sequencing.
		   */
	  	  case SYS_INIT:
		  {
			  /* 1. Refresh watchdog timer */
			  iwdg_kick();

			  /* 2. Re-calibrate ADC */
			  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

			  /* 3. Confirm there is no power fault */
			  result = check_power_health();
			  if (result == FAULT_NONE) {
				  g_daq.state = SYS_POWER_SEQ;
			  } else {
				  /* g_pending_fault.subsystem was set by health check */
				  fault_handler(result, g_pending_fault.subsystem);
			  }
			  break;
		  }

		  /* ── SYS_POWER_SEQ ────────────────────────────────────────
		   * Enable detection domain and wait for it to settle before
		   * any optical excitation is applied.
		   */
		  case SYS_POWER_SEQ:
		  {
			  result = init_power_domain(DETECT_EN_PORT, DETECT_EN_PIN,
			                             DETECT_FLT_PORT, DETECT_FLT_PIN,
			                             DELAY_DETECT_SETTLE_MS,
			                             FAULT_LOAD_SWITCH_DETECTION);
			  if (result == FAULT_NONE) {
				  g_daq.state = SYS_ACQUIRE;
			  } else {
				  fault_handler(result, SUBSYS_DETECTION);
			  }
			  break;
		  }

		  /* ── SYS_ACQUIRE ──────────────────────────────────────────
		   */
		  case SYS_ACQUIRE:
		  {
			  result = start_acquisition();
			  if (result != FAULT_NONE) {
				  fault_handler(result, SUBSYS_EXCITATION);
				  break;
			  }


			  /* Poll until complete or fault — kick IWDG each iteration */
			  while (!g_acq.complete) {
				  iwdg_kick();

				  /* Check for temperature alert while waiting */
				  if (g_pending_fault.type != FAULT_NONE) {
					  HAL_TIM_Base_Stop_IT(&htim2);
					  fault_handler(g_pending_fault.type, g_pending_fault.subsystem);
					  break;
				  }

				  HAL_Delay(10u);   /* Short yield — ISR will set complete */
			  }

			  /* Clear interrupt flags to prevent false triggering */
              __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
              __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);

			  /* Check if the ISR flagged a mid-window fault */
			  if (g_acq.fault != FAULT_NONE) {
			      
				  SubSystem sys = (g_pending_fault.subsystem != SUBSYS_PROCESSOR)
								  ? g_pending_fault.subsystem
								  : SUBSYS_EXCITATION;
				  fault_handler(g_acq.fault, sys);
				  break;
			  }

			  /* ── SUCCESS ── */
			  /* Data is collected in g_acq buffers. All validation, math, and logging happens next. */
			  g_daq.state = SYS_PROCESS;

			  break;
		  }

		  /* ── SYS_PROCESS ──────────────────────────────────────────
		   * Compute differential mean, apply temperature compensation,
		   * and compute CRC. */
		  case SYS_PROCESS:
		  {
			  /* process measurement buffers */
			  result = process_measurement(g_acq.buf_on,
										   g_acq.buf_off,
										   ACQUISITION_PAIRS,
										   &s_pending_record);

			  /* Check to see if there were no faults before continuing
			   * A temperature sensor fault is non-fatal — the record is still logged with op_status flagged.
			   * */
			  if (result == FAULT_NONE || result == FAULT_TEMP_SENSOR) {
				  g_daq.state = SYS_LOG;
			  } else {
				  fault_handler(result, SUBSYS_PROCESSOR);
			  }
			  break;
		  }

		  /* ── SYS_LOG ──────────────────────────────────────────────
		   * Push record to RAM buffer. If the buffer is full,
		   * log_measurement() triggers a flush internally.
		   */
		  case SYS_LOG:
		  {
			  result = log_measurement(&s_pending_record);
			  if (result == FAULT_NONE) {
				  g_fault_mgr.retry_count = 0;   /* Clean cycle: reset retries */
				  g_daq.state = SYS_STOP;

			  } else {
				  fault_handler(result, SUBSYS_SDCARD);
			  }
			  break;
		  }

		  /* ── SYS_STOP ─────────────────────────────────────────────
		   * Kick watchdog. enter_stop_mode() kicks once more internally
		   */
		  case SYS_STOP:
			  iwdg_kick();

			#if DAQ_DEBUG
			  debug_dump_sd();
			#endif

			  /* 7. Enter Ultra-Low Power Stop 2 */
			  g_daq.cycle_count++;
			  enter_stop_mode();

			  /* --- SYSTEM SLEEPS UNTIL RTC ALARM --- */

			  /* 8. Logic resumes here after wake. Restore state. */
			  g_daq.state = SYS_INIT;
			  break;

		  default:
			  g_daq.state = SYS_STOP;
			  break;
	  }
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_SMBUS_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hsmbus1.Instance = I2C1;
  hsmbus1.Init.Timing = 0x10B17DB5;
  hsmbus1.Init.AnalogFilter = SMBUS_ANALOGFILTER_ENABLE;
  hsmbus1.Init.OwnAddress1 = 2;
  hsmbus1.Init.AddressingMode = SMBUS_ADDRESSINGMODE_7BIT;
  hsmbus1.Init.DualAddressMode = SMBUS_DUALADDRESS_DISABLE;
  hsmbus1.Init.OwnAddress2 = 0;
  hsmbus1.Init.OwnAddress2Masks = SMBUS_OA2_NOMASK;
  hsmbus1.Init.GeneralCallMode = SMBUS_GENERALCALL_DISABLE;
  hsmbus1.Init.NoStretchMode = SMBUS_NOSTRETCH_DISABLE;
  hsmbus1.Init.PacketErrorCheckMode = SMBUS_PEC_DISABLE;
  hsmbus1.Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_SMBUS_SLAVE;
  hsmbus1.Init.SMBusTimeout = 0x0000830D;
  if (HAL_SMBUS_Init(&hsmbus1) != HAL_OK)
  {
    Error_Handler();
  }

  /** configuration Alert Mode
  */
  if (HAL_SMBUS_EnableAlert_IT(&hsmbus1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10B17DB5;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  /* 1. Check if the "Magic Number" 0x32F3 is in Backup Register 0 */
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F3)
  {

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x13;
  sTime.Minutes = 0x15;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_MAY;
  sDate.Date = 0x13;
  sDate.Year = 0x26;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  /* 3. Write the Magic Number so we skip this next time we wake up */
  	  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F3);
  }

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
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  htim2.Init.Prescaler = 64000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, TEST_PIN_Pin|LD2_Pin|SD_EN_Pin|DET_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EXC_EN_GPIO_Port, EXC_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TEST_PIN_Pin */
  GPIO_InitStruct.Pin = TEST_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEST_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin SD_EN_Pin DET_EN_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|SD_EN_Pin|DET_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_FAULT_Pin DET_FAULT_Pin */
  GPIO_InitStruct.Pin = SD_FAULT_Pin|DET_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXC_EN_Pin */
  GPIO_InitStruct.Pin = EXC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(EXC_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXC_FAULT_Pin */
  GPIO_InitStruct.Pin = EXC_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXC_FAULT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
