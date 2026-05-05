#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

/*
 * ADC 采样通道数量
 *
 * 如果 CubeMX 里 ADC1 配置了 2 个 Regular Rank，就改成 2。
 * 如果配置了 3 个 Regular Rank，就保持 3。
 */
#define BSP_ADC_CHANNEL_COUNT      2U

typedef enum
{
    BSP_ADC_PHASE_A = 0,
    BSP_ADC_PHASE_B = 1,
    BSP_ADC_PHASE_C = 2
} BSP_ADC_Channel_t;

void BSP_ADC_Init(void);
int BSP_ADC_Start(void);
int BSP_ADC_Stop(void);

uint16_t BSP_ADC_GetRaw(uint8_t channel);
uint16_t BSP_ADC_GetPhaseARaw(void);
uint16_t BSP_ADC_GetPhaseBRaw(void);
uint16_t BSP_ADC_GetPhaseCRaw(void);

#endif

