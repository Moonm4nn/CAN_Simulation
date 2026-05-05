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
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float throttle_pct;       // smoothed 0..100
  float torque_limit_pct;   // 0..100 (stub for later CAN)
  uint8_t fan_req;          // 0/1 (stub for later CAN)

  float rpm;
  float tempC;
  float voltage;

  uint8_t overtemp;
  uint8_t limited;
} PowertrainState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static PowertrainState pt = {
  .throttle_pct = 0.0f,
  .torque_limit_pct = 100.0f,
  .fan_req = 0,
  .rpm = 800.0f,
  .tempC = 35.0f,
  .voltage = 12.6f
};

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static uint8_t adc_to_pct(uint32_t adc_raw) {
  if (adc_raw > 4095U) adc_raw = 4095U;
  uint32_t pct = (adc_raw * 100U) / 4095U;
  if (pct < 3U) pct = 0U;      // deadband
  if (pct > 100U) pct = 100U;
  return (uint8_t)pct;
}

static void pt_update_model(PowertrainState* s, float dt_s) {
  // Constants
  const float RPM_IDLE = 800.0f;
  const float RPM_MAX  = 6500.0f;
  const float TEMP_AMB = 25.0f;

  const float RPM_TAU  = 0.15f;     // seconds (ramp feel)
  const float HEAT_K   = 0.00002f;  // tune if needed
  const float COOL_K   = 0.03f;
  const float COOL_FAN = 0.08f;

  // Apply torque limit
  float eff = (s->throttle_pct / 100.0f) * (s->torque_limit_pct / 100.0f);
  eff = clampf(eff, 0.0f, 1.0f);

  // RPM target and ramp
  float rpm_target = RPM_IDLE + eff * (RPM_MAX - RPM_IDLE);
  float alpha_rpm  = 1.0f - expf(-dt_s / RPM_TAU);
  s->rpm += alpha_rpm * (rpm_target - s->rpm);

  // Temperature (rise with load, cool to ambient; fan speeds cooling)
  float heat_in = HEAT_K * s->rpm * (0.3f + 0.7f * eff);
  float cool_k  = s->fan_req ? COOL_FAN : COOL_K;
  float cool    = cool_k * (s->tempC - TEMP_AMB);
  s->tempC += (heat_in - cool) * dt_s;

  // Voltage droop with load
  s->voltage = 12.6f - 0.8f * eff;

  // Status flags
  s->overtemp = (s->tempC >= 105.0f) ? 1 : 0;
  s->limited  = (s->torque_limit_pct < 99.5f) ? 1 : 0;
}

static void uart_debug_print(const PowertrainState* s, uint32_t raw_adc, uint32_t pct_inst) {
  // If you don't want UART spam, increase the print interval in the loop.
  char buf[160];
  int n = snprintf(buf, sizeof(buf),
                   "ADC=%lu thr=%lu%%(inst) thr=%.1f%%(sm) lim=%.0f%% rpm=%.0f temp=%.1fC V=%.2f OT=%d FAN=%d\r\n",
                   (unsigned long)raw_adc,
                   (unsigned long)pct_inst,
                   s->throttle_pct,
                   s->torque_limit_pct,
                   s->rpm,
                   s->tempC,
                   s->voltage,
                   s->overtemp,
                   s->fan_req);
  if (n > 0) {
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, 50);
  }
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  
  uint32_t accValue   = 0;
  uint32_t accPercent = 0;

  // scheduling
  uint32_t lastSenseMs = 0;
  uint32_t lastModelMs = 0;
  uint32_t lastLedMs   = 0;
  uint32_t lastUartMs  = 0;

  // tuning
  const uint32_t SENSE_PERIOD_MS = 10;   // read pot @ 100 Hz
  const uint32_t MODEL_PERIOD_MS = 10;   // update model @ 100 Hz
  const uint32_t LED_PERIOD_MS   = 500;  // heartbeat LED
  const uint32_t UART_PERIOD_MS  = 200;  // debug print rate

  // throttle smoothing
  const float THR_ALPHA = 0.15f;         // 0..1 higher = faster response
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    // 1) Read throttle potentiometer periodically
    if (now - lastSenseMs >= SENSE_PERIOD_MS) {
      lastSenseMs = now;

      if (HAL_ADC_Start(&hadc1) == HAL_OK) {
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
          accValue = HAL_ADC_GetValue(&hadc1);
          accPercent = (accValue * 100U) / 4095U;   // instantaneous
          uint8_t pct_db = adc_to_pct(accValue);    // with deadband

          // Smooth throttle (prevents jitter)
          pt.throttle_pct = (1.0f - THR_ALPHA) * pt.throttle_pct + THR_ALPHA * (float)pct_db;
        }
        HAL_ADC_Stop(&hadc1);
      }

      // Stub "commands" (later these come from CAN Node B):
      // Fan requested if temp rising high
      pt.fan_req = (pt.tempC > 90.0f) ? 1 : 0;

      // Torque limit if overtemp (demo limp mode)
      pt.torque_limit_pct = (pt.tempC > 100.0f) ? 60.0f : 100.0f;
    }

    // 2) Update powertrain model periodically
    if (now - lastModelMs >= MODEL_PERIOD_MS) {
      lastModelMs = now;
      pt_update_model(&pt, 0.010f); // 10ms timestep
    }

    // 3) Heartbeat LED (non-blocking, no HAL_Delay)
    if (now - lastLedMs >= LED_PERIOD_MS) {
      lastLedMs = now;
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }

    // 4) UART debug prints (optional but useful)
    if (now - lastUartMs >= UART_PERIOD_MS) {
      lastUartMs = now;
      uart_debug_print(&pt, accValue, accPercent);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
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
  sConfig.Channel = ADC_CHANNEL_12;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
