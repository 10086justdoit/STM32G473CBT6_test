#ifndef PLATFORM_DELAY_H
#define PLATFORM_DELAY_H

#include <stdint.h>

void Platform_DelayInit(void);
void Platform_DelayUs(uint16_t us);
void Platform_DelayMs(uint32_t ms);

#endif


/*
 * CubeMX 里面需要配置 TIM7：
 *
 * TIM7 计数频率 = 1 MHz
 * 也就是 1 个计数 = 1 us
 *
 * 例如系统定时器时钟是 170 MHz：
 * Prescaler = TIM7_CLK / 1000000 - 1 = 170 - 1
 * Counter Period = 65535
 */
