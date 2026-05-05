#include "platform_delay.h"
#include "main.h"
#include "tim.h"

void Platform_DelayInit(void)
{
    /*
     * TIM7 已经由 CubeMX 初始化。
     * 要求 TIM7 计数频率为 1 MHz。
     *
     * 例如 TIM7 时钟为 170 MHz：
     * Prescaler = 170 - 1
     * Period    = 65535
     */
    HAL_TIM_Base_Stop(&htim7);
    __HAL_TIM_SetCounter(&htim7, 0);
}

void Platform_DelayUs(uint16_t us)
{
    if (us == 0U)
    {
        return;
    }

    __HAL_TIM_SetCounter(&htim7, 0);

    HAL_TIM_Base_Start(&htim7);

    while (__HAL_TIM_GetCounter(&htim7) < us)
    {
    }

    HAL_TIM_Base_Stop(&htim7);
    __HAL_TIM_SetCounter(&htim7, 0);
}

void Platform_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}
