// danila(c)
#ifndef __POSITION_H
#define __POSITION_H

#include <cstdint>

extern "C" {
#include "main.h"

#include "stm32f3_discovery.h"
#include "stm32f3_discovery_accelerometer.h"
#include "stm32f3_discovery_gyroscope.h"
}

#define kBufSize 500U

struct AxisInfo {
  float position;
  float offset;
  uint32_t calibration;
  float *buffer;
  uint32_t index;
};

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

period_t HarmDuty(harm_t *signal);

void SetDuty(harm_t *signal);

void SampleGyroData(TIM_HandleTypeDef *htim);

void BSP_Position_Init();

#endif // __POSITION_H
