// dandemidow(c)

#include "position.h"

#include <math.h>
extern "C" {
#include "pwm.h"
}

extern struct AxisInfo x_axis;
extern struct AxisInfo y_axis;
//extern "C"{
extern TIM_HandleTypeDef htim2;
//}
static void MX_TIM2_Init(void);

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
void SampleGyroData(TIM_HandleTypeDef *htim) {
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

void BSP_Position_Init() {
  MX_TIM2_Init();
  HAL_TIM_RegisterCallback(&htim2, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, &SampleGyroData);

  if(BSP_GYRO_Init() != HAL_OK) {
    Error_Handler();
  }
  if(BSP_ACCELERO_Init() != HAL_OK) {
    Error_Handler();
  }

  if (HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
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
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
