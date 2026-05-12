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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "can_protocol.h"
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

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t canRxCount = 0;
volatile uint32_t canRxDecodeOk = 0;
volatile uint32_t canRxDecodeFail = 0;
volatile uint32_t canRxFifoReadFail = 0;

volatile uint32_t canWindowRxCount = 0;
volatile uint32_t canWindowDecodeOk = 0;
volatile uint32_t canWindowDecodeFail = 0;
volatile uint32_t canWindowFifoReadFail = 0;

volatile uint32_t canWindowThrottleCount = 0;
volatile uint32_t canWindowEngineCount = 0;
volatile uint32_t canWindowBrakeCount = 0;
volatile uint32_t canWindowFaultCount = 0;
volatile uint32_t canWindowUnknownCount = 0;

volatile uint8_t canWindowHasThrottle = 0;
volatile uint8_t canWindowHasEngine = 0;
volatile uint8_t canWindowHasBrake = 0;
volatile uint8_t canWindowHasFault = 0;

volatile CAN_DecodedMessage_t canWindowLastThrottleMsg;
volatile CAN_DecodedMessage_t canWindowLastEngineMsg;
volatile CAN_DecodedMessage_t canWindowLastBrakeMsg;
volatile CAN_DecodedMessage_t canWindowLastFaultMsg;

volatile uint8_t newCanMessageFlag = 0;
volatile CAN_DecodedMessage_t lastDecodedMsg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_Filter_Config(void);
static void dashboard_print_status(uint32_t windowMs,
                                   uint32_t windowRxCount,
                                   uint32_t windowDecodeOk,
                                   uint32_t windowDecodeFail,
                                   uint32_t windowFifoFail,
                                   uint32_t throttleCount,
                                   uint32_t engineCount,
                                   uint32_t brakeCount,
                                   uint32_t faultCount,
                                   uint32_t unknownCount,
                                   const CAN_DecodedMessage_t *throttleMsg,
                                   const CAN_DecodedMessage_t *engineMsg,
                                   const CAN_DecodedMessage_t *brakeMsg,
                                   const CAN_DecodedMessage_t *faultMsg,
                                   uint8_t hasThrottle,
                                   uint8_t hasEngine,
                                   uint8_t hasBrake,
                                   uint8_t hasFault);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  CAN_DecodedMessage_t decoded;

  if (hcan->Instance != CAN1) {
    return;
  }

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
    canRxCount++;
    canWindowRxCount++;

    if (CAN_Protocol_Decode(&rxHeader, rxData, &decoded)) {
      canRxDecodeOk++;
      canWindowDecodeOk++;

      switch (decoded.type) {
        case CAN_MSG_THROTTLE_STATUS:
          canWindowThrottleCount++;
          canWindowHasThrottle = 1;
          canWindowLastThrottleMsg = decoded;
          break;

        case CAN_MSG_ENGINE_STATUS:
          canWindowEngineCount++;
          canWindowHasEngine = 1;
          canWindowLastEngineMsg = decoded;
          break;

        case CAN_MSG_BRAKE_STATUS:
          canWindowBrakeCount++;
          canWindowHasBrake = 1;
          canWindowLastBrakeMsg = decoded;
          break;

        case CAN_MSG_FAULT_STATUS:
          canWindowFaultCount++;
          canWindowHasFault = 1;
          canWindowLastFaultMsg = decoded;
          break;

        default:
          canWindowUnknownCount++;
          break;
      }
    } else {
      canRxDecodeFail++;
      canWindowDecodeFail++;
    }
  } else {
    canRxFifoReadFail++;
    canWindowFifoReadFail++;
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  CAN_Filter_Config();

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }

  uint32_t lastStatusPrintMs = 0;
  uint32_t lastHeartbeatMs = 0;
  const uint32_t statusPrintIntervalMs = 500;
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    if (now - lastStatusPrintMs >= statusPrintIntervalMs) {
      uint32_t windowRxCount;
      uint32_t windowDecodeOk;
      uint32_t windowDecodeFail;
      uint32_t windowFifoFail;
      uint32_t windowThrottle;
      uint32_t windowEngine;
      uint32_t windowBrake;
      uint32_t windowFault;
      uint32_t windowUnknown;
      uint8_t hasThrottle;
      uint8_t hasEngine;
      uint8_t hasBrake;
      uint8_t hasFault;
      CAN_DecodedMessage_t throttleMsg;
      CAN_DecodedMessage_t engineMsg;
      CAN_DecodedMessage_t brakeMsg;
      CAN_DecodedMessage_t faultMsg;

      __disable_irq();
      windowRxCount = canWindowRxCount;
      windowDecodeOk = canWindowDecodeOk;
      windowDecodeFail = canWindowDecodeFail;
      windowFifoFail = canWindowFifoReadFail;
      windowThrottle = canWindowThrottleCount;
      windowEngine = canWindowEngineCount;
      windowBrake = canWindowBrakeCount;
      windowFault = canWindowFaultCount;
      windowUnknown = canWindowUnknownCount;
      hasThrottle = canWindowHasThrottle;
      hasEngine = canWindowHasEngine;
      hasBrake = canWindowHasBrake;
      hasFault = canWindowHasFault;

      throttleMsg = canWindowLastThrottleMsg;
      engineMsg = canWindowLastEngineMsg;
      brakeMsg = canWindowLastBrakeMsg;
      faultMsg = canWindowLastFaultMsg;

      canWindowRxCount = 0;
      canWindowDecodeOk = 0;
      canWindowDecodeFail = 0;
      canWindowFifoReadFail = 0;
      canWindowThrottleCount = 0;
      canWindowEngineCount = 0;
      canWindowBrakeCount = 0;
      canWindowFaultCount = 0;
      canWindowUnknownCount = 0;
      canWindowHasThrottle = 0;
      canWindowHasEngine = 0;
      canWindowHasBrake = 0;
      canWindowHasFault = 0;
      __enable_irq();

      lastStatusPrintMs = now;
      dashboard_print_status(statusPrintIntervalMs,
                             windowRxCount,
                             windowDecodeOk,
                             windowDecodeFail,
                             windowFifoFail,
                             windowThrottle,
                             windowEngine,
                             windowBrake,
                             windowFault,
                             windowUnknown,
                             &throttleMsg,
                             &engineMsg,
                             &brakeMsg,
                             &faultMsg,
                             hasThrottle,
                             hasEngine,
                             hasBrake,
                             hasFault);
    }

    if (now - lastHeartbeatMs >= 500) {
      lastHeartbeatMs = now;
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  
  /* USER CODE END CAN1_Init 2 */

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
static void CAN_Filter_Config(void)
{
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK) {
    Error_Handler();
  }
}

static void dashboard_print_status(uint32_t windowMs,
                                   uint32_t windowRxCount,
                                   uint32_t windowDecodeOk,
                                   uint32_t windowDecodeFail,
                                   uint32_t windowFifoFail,
                                   uint32_t throttleCount,
                                   uint32_t engineCount,
                                   uint32_t brakeCount,
                                   uint32_t faultCount,
                                   uint32_t unknownCount,
                                   const CAN_DecodedMessage_t *throttleMsg,
                                   const CAN_DecodedMessage_t *engineMsg,
                                   const CAN_DecodedMessage_t *brakeMsg,
                                   const CAN_DecodedMessage_t *faultMsg,
                                   uint8_t hasThrottle,
                                   uint8_t hasEngine,
                                   uint8_t hasBrake,
                                   uint8_t hasFault)
{
  char buf[160];

  int n = snprintf(buf, sizeof(buf),
                   "NODE_B %lums: RX=%lu OK=%lu FAIL=%lu FIFO=%lu THR=%lu ENG=%lu BRK=%lu FLT=%lu UNK=%lu\r\n",
                   (unsigned long)windowMs,
                   (unsigned long)windowRxCount,
                   (unsigned long)windowDecodeOk,
                   (unsigned long)windowDecodeFail,
                   (unsigned long)windowFifoFail,
                   (unsigned long)throttleCount,
                   (unsigned long)engineCount,
                   (unsigned long)brakeCount,
                   (unsigned long)faultCount,
                   (unsigned long)unknownCount);

  if (n > 0) {
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 50);
  }

  if (hasThrottle && throttleMsg != NULL) {
    n = snprintf(buf, sizeof(buf),
                 "  THROTTLE: thr=%u%% rpm=%u speed=%u kph\r\n",
                 throttleMsg->data.throttle.throttle_percent,
                 throttleMsg->data.throttle.rpm,
                 throttleMsg->data.throttle.speed_kph);
    if (n > 0) {
      HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 50);
    }
  }

  if (hasEngine && engineMsg != NULL) {
    n = snprintf(buf, sizeof(buf),
                 "  ENGINE: rpm=%u speed=%u kph temp=%uC fault=%u\r\n",
                 engineMsg->data.engine.rpm,
                 engineMsg->data.engine.speed_kph,
                 engineMsg->data.engine.engine_temp_c,
                 engineMsg->data.engine.engine_fault);
    if (n > 0) {
      HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 50);
    }
  }

  if (hasBrake && brakeMsg != NULL) {
    n = snprintf(buf, sizeof(buf),
                 "  BRAKE: active=%u pressure=%u%% light=%u fault=%u\r\n",
                 brakeMsg->data.brake.brake_active,
                 brakeMsg->data.brake.brake_pressure,
                 brakeMsg->data.brake.brake_light,
                 brakeMsg->data.brake.brake_fault);
    if (n > 0) {
      HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 50);
    }
  }

  if (hasFault && faultMsg != NULL) {
    n = snprintf(buf, sizeof(buf),
                 "  FAULT: source=%u code=%u active=%u\r\n",
                 faultMsg->data.fault.source_node,
                 faultMsg->data.fault.fault_code,
                 faultMsg->data.fault.fault_active);
    if (n > 0) {
      HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 50);
    }
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
