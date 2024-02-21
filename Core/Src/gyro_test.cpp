// dandemidow(c)

#include "gyro_test.h"

#include <cstdio>

#define kCalibrationSamples 250U

float test_buffer[kBufSize] = {0};
float test_buffer_y[kBufSize] = {0};
struct AxisInfo x_axis = {0.F, 0.009924F, kCalibrationSamples, &test_buffer[0], 0U};
struct AxisInfo y_axis = {0.F, -0.022529F, kCalibrationSamples, &test_buffer_y[0], 0U};
harm_t signal_1 = {50, 1, TIM_CHANNEL_1, 0, 1U, 1, 1};
harm_t signal_2 = {50, 0, TIM_CHANNEL_2, 0, 1U, 1, 1};
period_t period {};


void Dance() {
  if (x_axis.calibration != 0U) {
     HAL_Delay(2);
     return;
  }
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

  if (x_axis.index >= kBufSize) {
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
