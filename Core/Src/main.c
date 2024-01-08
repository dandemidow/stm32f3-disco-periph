/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) Danila Demidov
  */
#include <stdio.h>
#include <math.h>

#include "main.h"

#include "stm32f3_discovery.h"
#include "stm32f3_discovery_accelerometer.h"
#include "stm32f3_discovery_gyroscope.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void BSP_PWM_Set(uint32_t const duty, uint32_t const channel);
static void BSP_PWM_SetPulse(uint32_t const duty, uint32_t const channel);

static void SampleGyroData();

extern void initialise_monitor_handles(void);

struct AxisInfo {
  float position;
  float offset;
  uint32_t calibration;
  float *buffer;
  uint32_t index;
};
#define kBufSize 500U
#define kCalibrationSamples 250U

float test_buffer[kBufSize] = {0};
float test_buffer_y[kBufSize] = {0};
static struct AxisInfo x_axis = {0.F, 0.009924F, kCalibrationSamples, &test_buffer[0], 0U};
static struct AxisInfo y_axis = {0.F, -0.022529F, kCalibrationSamples, &test_buffer_y[0], 0U};
//LED_GREEN  = LED6,
//LED_ORANGE = LED5,
//LED_RED    = LED3,
//LED_BLUE   = LED4,
//LED_GREEN_2  = LED7,
//LED_ORANGE_2 = LED8,
//LED_RED_2    = LED10,
//LED_BLUE_2   = LED9
static Led_TypeDef leds[] = {LED_RED, LED_BLUE, LED_GREEN, LED_ORANGE_2, LED_RED_2, LED_BLUE_2, LED_GREEN_2, LED_ORANGE};

typedef enum {
  k0Pi,
  kPi_2,
  kPi,
  k3Pi_2,
  k2Pi
} period_t;

typedef struct {
  int duty;
  int dir;
  uint32_t channel;
  uint32_t count;
  uint32_t div;
  int enable;
  int old_dir;
} harm_t;

inline void EnableSignal(harm_t *signal) {
  signal->enable = 1;
}

inline void DisableSignal(harm_t *signal) {
  signal->enable = 0;
}

inline void StartSignal(harm_t *signal) {
  signal->dir = signal->old_dir;

}

inline void StopSignal(harm_t *signal) {
  signal->old_dir = signal->dir;
  signal->dir = 0;
}

period_t HarmDuty(harm_t *signal) {
    if (!signal->enable || !signal->dir) {
      return k0Pi;
    }
    signal->duty += signal->dir;
    period_t result = k0Pi;
    switch (signal->duty / signal->div) {
    case 0: {
        signal->dir = 1;
        result = k3Pi_2;
      };
      break;
    case 50: {
        if (signal->dir > 0) {
          result = k2Pi;
        } else {
          result = kPi;
        }
      };
      break;
    case 100: {
        signal->dir = -1;
        result = kPi_2;
      };
      break;
    }
    return result;
}

void SetDuty(harm_t *signal) {
  if (signal->enable == 1) {
    if (signal->dir != 0) {
      BSP_PWM_SetPulse(signal->duty / signal->div, signal->channel);
      HAL_Delay(1);
    }
  } else {
    BSP_PWM_SetPulse(50, signal->channel);
  }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  initialise_monitor_handles();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED7);
  BSP_LED_Init(LED9);
  BSP_LED_Init(LED10);
  BSP_LED_Init(LED8);
  BSP_LED_Init(LED6);
  MX_TIM1_Init();
  MX_TIM2_Init();
  HAL_TIM_RegisterCallback(&htim2, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, &SampleGyroData);

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
    /* PWM Generation Error */
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
    /* PWM Generation Error */
    Error_Handler();
  }

  HAL_Delay(100);

  if(BSP_GYRO_Init() != HAL_OK) {
    Error_Handler();
  }
  if(BSP_ACCELERO_Init() != HAL_OK) {
    Error_Handler();
  }

  if (HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }

  HAL_Delay(5000);

  /* Infinite loop */
  harm_t signal_1 = {50, 1, TIM_CHANNEL_1, 0, 1U, 1, 1};
  harm_t signal_2 = {50, 0, TIM_CHANNEL_2, 0, 1U, 1, 1};
  Led_TypeDef led_index = 0;
  period_t period;
  while (1) {
    if (x_axis.calibration != 0U) {
        HAL_Delay(2);
        continue;
    }
    BSP_LED_Toggle(led_index);
    led_index = (led_index + 1) % 8U;

    SetDuty(&signal_1);
    SetDuty(&signal_2);

    period = HarmDuty(&signal_1);
    if (signal_1.count == 0) {
      if (period == kPi_2 || period == kPi || period == k3Pi_2) {
        StopSignal(&signal_1);
//        printf("signal 01 %d\r\n", period);
        StartSignal(&signal_2);
      }
      if (period == k2Pi) {
        signal_1.count++;
//        printf("signal 1: %d;\r\n", signal_1.count);
        StopSignal(&signal_1);
        StartSignal(&signal_2);
      }
    } else {
      if (period == k2Pi) {
        signal_1.count++;
//        printf("signal 1: %d;\r\n", signal_1.count);
        StopSignal(&signal_1);
        StartSignal(&signal_2);
      }
    }

    period = HarmDuty(&signal_2);
    if (signal_2.count == 0) {
      if (period == kPi_2 || period == kPi || period == k3Pi_2) {
//        printf("signal 02 %d\r\n", period);
        StopSignal(&signal_2);
        StartSignal(&signal_1);
      }
      if (period == k2Pi) {
        signal_2.count++;
//        printf("signal 2: %d;\r\n", signal_2.count);
        StopSignal(&signal_2);
        StartSignal(&signal_1);
      }
    } else {
      if (period == k2Pi) {
        signal_2.count++;
//        printf("signal 2: %d;\r\n", signal_2.count);
        if (signal_2.count % 2 == 1) {
          StopSignal(&signal_2);
          StartSignal(&signal_1);
        }
      }
    }


    /* Gyroscope variable */
//    float Buffer[3];
//    float Xval, Yval, Zval = 0x00;
    int16_t buffer[3] = {0};
    int16_t xval, yval, zval = 0x00;

    /* Read Acceleration*/
    //BSP_ACCELERO_GetXYZ(buffer);
    /* Update autoreload and capture compare registers value*/
    xval = buffer[0];
    yval = buffer[1];
    zval = buffer[2];


    /* Read Gyro Angular data */
//    BSP_GYRO_GetXYZ(Buffer);
    /* Update autoreload and capture compare registers value*/
//    Xval = Buffer[0];
//    Yval = Buffer[1];
//    Zval = Buffer[2];

    if (x_axis.index >= kBufSize) {
//      printf("%d; %d; %d; ", (int)(Xval/20.F), (int)(Yval/20.F), (int)(Zval/20.F));
      printf("offset x: %f; y: %f\r\n", x_axis.offset, y_axis.offset);
      for (int i = 0; i < x_axis.index; i++) {
        printf("%f; %f\r\n", x_axis.buffer[i], y_axis.buffer[i]);
      }
      printf("==========\r\n");
      x_axis.index = 0U;
      x_axis.buffer = NULL;
      DisableSignal(&signal_1);
      DisableSignal(&signal_2);
      //printf("%d; %d; %d\r\n", xval, yval, zval);
    } else {
      HAL_Delay(2);
    }
  }
}

static inline void CalcPosition(struct AxisInfo *axis, float const gyro) {
  float const delta = 0.00002F * gyro - axis->offset;
  if (fabs(delta) > 0.005F) {
    axis->position += delta;
  }
  if (axis->index < kBufSize && axis->buffer) {
    axis->buffer[axis->index] = axis->position;
    axis->index++;
  }
}

/**
 * @brief SampleGyroData
 */
static void SampleGyroData() {
  float Buffer[3];
  BSP_GYRO_GetXYZ(Buffer);
  if (x_axis.calibration > 0U) {
    x_axis.calibration--;
//    x_axis.offset += 0.00002F * Buffer[1];
//    x_axis.offset /= 2.F;
//    y_axis.offset += 0.00002F * Buffer[0];
//    y_axis.offset /= 2.F;
  } else {
    CalcPosition(&x_axis, Buffer[1]);
    CalcPosition(&y_axis, Buffer[0]);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
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
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

uint32_t kPeriodValue = (uint32_t)(479 - 1);

static void BSP_PWM_SetPulse(uint32_t const duty, uint32_t const channel) {
  float const kMinPercent = 5.F;
  float const kMaxPercent = 10.F;
  float const kRange = kMaxPercent - kMinPercent;
  float const coef = kMinPercent + (duty * kRange) / 100.F;
  uint32_t const pulse = (uint32_t)((kPeriodValue * coef) / 100);

  __HAL_TIM_SET_COMPARE(&htim1, channel, pulse);
}

// duty: [1000, 2000] <- 100 percet range
static void BSP_PWM_Set(uint32_t const duty, uint32_t const channel) {
  TIM_OC_InitTypeDef sConfigOC = {0};

  float const kMinPercent = 5.F;
  float const kMaxPercent = 10.F;
  float const kRange = kMaxPercent - kMinPercent;
  float const coef = kMinPercent + (duty * kRange) / 100.F;

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (uint32_t)((kPeriodValue * coef) / 100);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, channel) != HAL_OK) {
    Error_Handler();
  }
}


/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

//  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  uint32_t uhPrescalerValue = (uint32_t)(SystemCoreClock / 24000) - 1;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = uhPrescalerValue;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = kPeriodValue;
  htim1.Init.ClockDivision = 0;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  BSP_PWM_Set(50U, TIM_CHANNEL_1);
  BSP_PWM_Set(50U, TIM_CHANNEL_2);

//  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//  sBreakDeadTimeConfig.DeadTime = 0;
//  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//  sBreakDeadTimeConfig.BreakFilter = 0;
//  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
//  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
//  sBreakDeadTimeConfig.Break2Filter = 0;
//  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }

  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};

  uint32_t uhPrescalerValue = (uint32_t)(SystemCoreClock / 24000) - 1;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = uhPrescalerValue;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = kPeriodValue;
  htim2.Init.ClockDivision = 0;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {}
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
//  sConfigOC.OCMode = TIM_OCMODE_TIMING;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
//    Error_Handler();
//  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
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
