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

#include "main.h"

#include "stm32f3_discovery.h"
#include "stm32f3_discovery_accelerometer.h"
#include "stm32f3_discovery_gyroscope.h"

TIM_HandleTypeDef htim1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void BSP_PWM_Set(uint32_t const duty, uint32_t const channel);
static void BSP_PWM_SetPulse(uint32_t const duty, uint32_t const channel);

extern void initialise_monitor_handles(void);

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

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
    /* PWM Generation Error */
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
    /* PWM Generation Error */
    Error_Handler();
  }

  HAL_Delay(100);

  BSP_LED_Toggle(LED6);
  if(BSP_GYRO_Init() != HAL_OK) {
    BSP_LED_Toggle(LED3);
    Error_Handler();
  }
  if(BSP_ACCELERO_Init() != HAL_OK) {
    BSP_LED_Toggle(LED3);
    Error_Handler();
  }

  HAL_Delay(5000);

  /* Infinite loop */
  uint32_t duty1 = 50;
  uint32_t duty2 = 50;
  int dir1 = 0;
  int dir2 = 1;
  int meas_data_counter = 0;
  while (1) {
    if (dir1 != 0) {
      BSP_PWM_SetPulse(duty1, TIM_CHANNEL_1);
    }

    if (dir2 != 0) {
      BSP_PWM_SetPulse(duty2, TIM_CHANNEL_2);
    }
    duty1 += dir1;
    if (duty1 == 0) {
      dir1 = 1;
    } else if (duty1 >= 100) {
      dir1 = -1;
    }
    duty2 += dir2;
    if (duty2 == 0) {
      dir2 = 1;
      BSP_LED_On(LED8);
    } else if (duty2 >= 100) {
      dir2 = -1;
      BSP_LED_Off(LED8);
    }
    BSP_LED_Toggle(LED4);
    /* Gyroscope variable */
    float Buffer[3];
    float Xval, Yval, Zval = 0x00;
    int16_t buffer[3] = {0};
    int16_t xval, yval, zval = 0x00;

    /* Read Acceleration*/
    BSP_ACCELERO_GetXYZ(buffer);
    /* Update autoreload and capture compare registers value*/
    xval = buffer[0];
    yval = buffer[1];
    zval = buffer[2];


    /* Read Gyro Angular data */
    BSP_GYRO_GetXYZ(Buffer);
    /* Update autoreload and capture compare registers value*/
    Xval = Buffer[0];
    Yval = Buffer[1];
    Zval = Buffer[2];

    if (meas_data_counter++ % 2 == 0) {
      printf("%d; %d; %d; ", (int)(Xval/20.F), (int)(Yval/20.F), (int)(Zval/20.F));
      printf("%d; %d; %d\r\n", xval, yval, zval);
    } else {
      HAL_Delay(2);
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
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

// duty: [800, 2200] <- 100 percet range
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
