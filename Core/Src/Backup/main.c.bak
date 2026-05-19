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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CHANNEL_COUNT     7U
#define ADC_MAX_VALUE         65535.0f
#define ADC_VREF              3.3f

/* Voltage divider values */
#define VOLTAGE_R_TOP         47000.0f
#define VOLTAGE_R_BOTTOM      10000.0f

/* NTC divider values: 3.3V -> 10k pull-up -> ADC node -> NTC -> GND */
#define NTC_PULLUP_OHM        10000.0f
#define NTC_R0_OHM            10000.0f
#define NTC_T0_K              298.15f
#define NTC_BETA              3950.0f

/* Hall current sensor placeholders */
#define CURRENT_ZERO_V        1.65f
#define CURRENT_SENS_V_PER_A  0.066f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc_raw[ADC_CHANNEL_COUNT];

volatile uint8_t adc_sequence_ready = 0;

typedef struct
{
  float adc_pin_v[ADC_CHANNEL_COUNT];

  float voltage_1;
  float voltage_2;
  float voltage_3;

  float temperature_1;
  float temperature_2;
  float temperature_3;

  float current;
} MeasurementData;

MeasurementData data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static float ADC_RawToPinVoltage(uint16_t raw);
static float VoltageDivider_ToInputVoltage(float vadc);
static float NTC_ToTemperatureC(float vadc);
static float HallSensor_ToCurrentA(float vadc);
static void Process_ADC_Data(void);
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

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

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
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
	{
	  Error_Handler();
	}

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, ADC_CHANNEL_COUNT) != HAL_OK)
	{
	  Error_Handler();
	}

	if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
	{
	  Error_Handler();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (adc_sequence_ready)
		{
		  adc_sequence_ready = 0;
		  Process_ADC_Data();

		  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

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
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static float ADC_RawToPinVoltage(uint16_t raw)
{
  return ((float)raw * ADC_VREF) / ADC_MAX_VALUE;
}

static float VoltageDivider_ToInputVoltage(float vadc)
{
  return vadc * (VOLTAGE_R_TOP + VOLTAGE_R_BOTTOM) / VOLTAGE_R_BOTTOM;
}

static float NTC_ToTemperatureC(float vadc)
{
  float r_ntc;
  float temp_k;

  if (vadc <= 0.01f)
  {
    return -273.15f;
  }

  if (vadc >= (ADC_VREF - 0.01f))
  {
    return -273.15f;
  }

  r_ntc = (NTC_PULLUP_OHM * vadc) / (ADC_VREF - vadc);

  temp_k = 1.0f / ((1.0f / NTC_T0_K) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0_OHM));

  return temp_k - 273.15f;
}

static float HallSensor_ToCurrentA(float vadc)
{
  return (vadc - CURRENT_ZERO_V) / CURRENT_SENS_V_PER_A;
}

static void Process_ADC_Data(void)
{
  for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
  {
	  data.adc_pin_v[i] = ADC_RawToPinVoltage(adc_raw[i]);
  }

  /*
   * ADC rank mapping:
   * adc_raw[0] -> Voltage_1
   * adc_raw[1] -> Voltage_2
   * adc_raw[2] -> Voltage_3
   * adc_raw[3] -> Temperature_1
   * adc_raw[4] -> Temperature_2
   * adc_raw[5] -> Temperature_3
   * adc_raw[6] -> Current
   */
  data.voltage_1 = VoltageDivider_ToInputVoltage(data.adc_pin_v[0]);
  data.voltage_2 = VoltageDivider_ToInputVoltage(data.adc_pin_v[1]);
  data.voltage_3 = VoltageDivider_ToInputVoltage(data.adc_pin_v[2]);

  data.temperature_1 = NTC_ToTemperatureC(data.adc_pin_v[3]);
  data.temperature_2 = NTC_ToTemperatureC(data.adc_pin_v[4]);
  data.temperature_3 = NTC_ToTemperatureC(data.adc_pin_v[5]);

  data.current = HallSensor_ToCurrentA(data.adc_pin_v[6]);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_sequence_ready = 1;
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
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
