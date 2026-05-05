#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>

typedef enum
{
    BSP_PWM_CHANNEL_A = 0,
    BSP_PWM_CHANNEL_B = 1,
    BSP_PWM_CHANNEL_C = 2
} BSP_PWM_Channel_t;

void BSP_PWM_Init(void);

int BSP_PWM_Start(void);
int BSP_PWM_Stop(void);

void BSP_PWM_EnableDriver(void);
void BSP_PWM_DisableDriver(void);

void BSP_PWM_SetDuty(BSP_PWM_Channel_t channel, float duty);
void BSP_PWM_SetDutyABC(float duty_a, float duty_b, float duty_c);

float BSP_PWM_GetDutyA(void);
float BSP_PWM_GetDutyB(void);
float BSP_PWM_GetDutyC(void);

#endif
