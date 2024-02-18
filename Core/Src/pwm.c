// dandemidow(c)

#include "pwm.h"

extern TIM_HandleTypeDef htim1;

static void MX_TIM1_Init(void);


void BSP_PWM_Init() {
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
}

void BSP_PWM_SetPulse(uint32_t const duty, uint32_t const channel) {
  float const kMinPercent = 5.F;
  float const kMaxPercent = 10.F;
  float const kRange = kMaxPercent - kMinPercent;
  float const coef = kMinPercent + (duty * kRange) / 100.F;
  uint32_t const pulse = (uint32_t)((kPeriodValue * coef) / 100);

  __HAL_TIM_SET_COMPARE(&htim1, channel, pulse);
}

// duty: [1000, 2000] <- 100 percet range
void BSP_PWM_Set(uint32_t const duty, uint32_t const channel) {
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

  HAL_TIM_MspPostInit(&htim1);
}
