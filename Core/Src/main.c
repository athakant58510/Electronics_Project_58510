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

/* ADC defaults */
#define ADC_CHANNEL_COUNT     7U
#define ADC_MAX_VALUE         4095.0f
#define ADC_VREF              3.3f

/* Voltage divider values */
#define VOLTAGE_R_TOP         47000.0f
#define VOLTAGE_R_BOTTOM      10000.0f

/* NTC divider values: 3.3V -> 10k pull-up -> ADC node -> NTC -> GND */
#define NTC_PULLUP_OHM        10000.0f
#define NTC_R0_OHM            10000.0f
#define NTC_T0_K              298.15f
#define NTC_BETA              3950.0f

/* Hall current sensor: TMCS1101A4B */
#define CURRENT_ZERO_V_DEFAULT        1.65f
#define CURRENT_SENS_V_PER_A          0.400f

/* Digital filtering */
#define MOVING_AVG_SIZE               16U
#define CURRENT_OFFSET_CAL_SAMPLES    128U

/*
 * ADC rank mapping:
 * Rank 1: PA3 / ADC1_INP15 -> Voltage_1
 * Rank 2: PA2 / ADC1_INP14 -> Voltage_3
 * Rank 3: PC0 / ADC1_INP10 -> Voltage_2
 * Rank 4: PB1 / ADC1_INP5  -> Temperature_1
 * Rank 5: PA4 / ADC1_INP18 -> Temperature_2
 * Rank 6: PA5 / ADC1_INP19 -> Temperature_3
 * Rank 7: PA6 / ADC1_INP3  -> Current
 */
#define ADC_V1      0U
#define ADC_V3      1U
#define ADC_V2      2U
#define ADC_T1      3U
#define ADC_T2      4U
#define ADC_T3      5U
#define ADC_I1      6U

#define FILT_V1         0U
#define FILT_V2         1U
#define FILT_V3         2U
#define FILT_T1         3U
#define FILT_T2         4U
#define FILT_T3         5U
#define FILT_I1         6U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
uint16_t adc_raw[ADC_CHANNEL_COUNT];

volatile uint8_t adc_sequence_ready = 0;
volatile uint32_t adc_sequence_counter = 0;

/* This value is measured at startup with 0 A current */
static float current_zero_v = CURRENT_ZERO_V_DEFAULT;

/* Moving average buffers */
static float moving_avg_buffer[ADC_CHANNEL_COUNT][MOVING_AVG_SIZE] = {0};
static float moving_avg_sum[ADC_CHANNEL_COUNT] = {0};
static uint8_t moving_avg_index = 0;
static uint8_t moving_avg_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static float ADCRawToVolt(uint16_t raw);
static float VoltConv(float vadc);
static float NTCRawToTemp(float vadc);
static float HallToCurr(float vadc);

static float MovingAvg_Update(uint8_t channel, float new_sample);
static void MovingAvgHelper(void);
static void CurrOffsetCalibrate(void);

static void main_Func(void);
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
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		Error_Handler();

	}

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, ADC_CHANNEL_COUNT) != HAL_OK)
	{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		Error_Handler();
	}

	if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
	{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		Error_Handler();
	}

	CurrOffsetCalibrate();
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
			main_Func();
			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
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


/**
 * @brief  Converts raw ADC value to ADC input voltage.
 * @param  raw ADC raw value.
 * @retval ADC input voltage in volts.
 */
static float ADCRawToVolt(uint16_t raw)
{
	return ((float)raw * ADC_VREF) / ADC_MAX_VALUE;
}


/**
 * @brief  Converts ADC voltage to the real input voltage.
 * @param  vadc Voltage measured at the ADC pin.
 * @retval Input voltage before the voltage divider.
 */
static float VoltConv(float vadc)
{
	return vadc * (VOLTAGE_R_TOP + VOLTAGE_R_BOTTOM) / VOLTAGE_R_BOTTOM;
}


/**
 * @brief  Converts NTC ADC voltage to temperature.
 * @param  vadc Voltage measured at the NTC divider output.
 * @retval Temperature in degrees Celsius.
 */
static float NTCRawToTemp(float vadc)
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


/**
 * @brief  Converts Hall sensor output voltage to current.
 * @param  vadc Voltage measured at the Hall sensor output.
 * @retval Current in amperes.
 */
static float HallToCurr(float vadc)
{
	return (vadc - current_zero_v) / CURRENT_SENS_V_PER_A;
}


/**
 * @brief  Updates the moving average filter for one channel.
 * @param  channel Channel index.
 * @param  new_sample New sample value.
 * @retval Filtered value.
 */
static float MovingAvg_Update(uint8_t channel, float new_sample)
{
	float divisor;

	if (channel >= ADC_CHANNEL_COUNT)
	{
		return new_sample;
	}

	moving_avg_sum[channel] -= moving_avg_buffer[channel][moving_avg_index];
	moving_avg_buffer[channel][moving_avg_index] = new_sample;
	moving_avg_sum[channel] += new_sample;

	if (moving_avg_count < MOVING_AVG_SIZE)
	{
		divisor = (float)(moving_avg_count + 1U);
	}
	else
	{
		divisor = (float)MOVING_AVG_SIZE;
	}

	return moving_avg_sum[channel] / divisor;
}



/**
 * @brief  Updates the moving average index after one full ADC sequence.
 * @param  None
 * @retval None
 */
static void MovingAvgHelper(void)
{
	if (moving_avg_count < MOVING_AVG_SIZE)
	{
		moving_avg_count++;
	}

	moving_avg_index++;

	if (moving_avg_index >= MOVING_AVG_SIZE)
	{
		moving_avg_index = 0;
	}
}


/**
 * @brief  Converts ADC samples to voltage, temperature and current values.
 * @param  None
 * @retval None
 */
static void main_Func(void)
{
	float v1_raw;
	float v2_raw;
	float v3_raw;

	float t1_raw;
	float t2_raw;
	float t3_raw;

	float current_raw;

	for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
	{
		data.adc_pin_v[i] = ADCRawToVolt(adc_raw[i]);
	}

	v1_raw = VoltConv(data.adc_pin_v[ADC_V1]);
	v2_raw = VoltConv(data.adc_pin_v[ADC_V2]);
	v3_raw = VoltConv(data.adc_pin_v[ADC_V3]);

	t1_raw = NTCRawToTemp(data.adc_pin_v[ADC_T1]);
	t2_raw = NTCRawToTemp(data.adc_pin_v[ADC_T2]);
	t3_raw = NTCRawToTemp(data.adc_pin_v[ADC_T3]);

	current_raw = HallToCurr(data.adc_pin_v[ADC_I1]);

	data.voltage_1 = MovingAvg_Update(FILT_V1, v1_raw);
	data.voltage_2 = MovingAvg_Update(FILT_V2, v2_raw);
	data.voltage_3 = MovingAvg_Update(FILT_V3, v3_raw);

	data.temperature_1 = MovingAvg_Update(FILT_T1, t1_raw);
	data.temperature_2 = MovingAvg_Update(FILT_T2, t2_raw);
	data.temperature_3 = MovingAvg_Update(FILT_T3, t3_raw);

	data.current = MovingAvg_Update(FILT_I1, current_raw);

	MovingAvgHelper();
}



/**
 * @brief  Calculates Hall sensor offset using zero-current samples.
 * @param  None
 * @retval None
 */
static void CurrOffsetCalibrate(void)
{
	float offset_sum = 0.0f;
	uint16_t samples = 0;
	uint32_t start_tick = HAL_GetTick();

	/*
	 * Wait for CURRENT_OFFSET_CAL_SAMPLES complete ADC sequences.
	 * Current must be 0 A during this step.
	 */
	while (samples < CURRENT_OFFSET_CAL_SAMPLES)
	{
		if (adc_sequence_ready)
		{
			adc_sequence_ready = 0;

			offset_sum += ADCRawToVolt(adc_raw[ADC_I1]);
			samples++;
		}

		/*
		 * Safety timeout: if ADC/DMA/TIM trigger is wrong, do not block forever.
		 */
		if ((HAL_GetTick() - start_tick) > 5000U)
		{
			Error_Handler();
		}
	}

	current_zero_v = offset_sum / (float)CURRENT_OFFSET_CAL_SAMPLES;
}


/**
 * @brief  ADC conversion callback function.
 * @param  hadc Pointer to ADC handle.
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1)
	{
		adc_sequence_ready = 1;
		adc_sequence_counter++;//Debug variable
	}
}


/**
 * @brief  ADC error callback function.
 * @param  hadc Pointer to ADC handle.
 * @retval None
 */
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
