// dandemidow(c)
#ifndef __PWM_H
#define __PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "main.h"

#define kPeriodValue (uint32_t)(479 - 1)

void BSP_PWM_Init();

void BSP_PWM_Set(uint32_t const duty, uint32_t const channel);
void BSP_PWM_SetPulse(uint32_t const duty, uint32_t const channel);

#ifdef __cplusplus
}
#endif

#endif // __PWM_H
