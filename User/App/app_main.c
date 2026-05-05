// #include "app_main.h"
#include "main.h"

#include "bsp_uart.h"
#include "vofa_service.h"
#include "bsp_adc.h"

void App_Init(void)
{
    BSP_UART_Init();
    VOFA_Service_Init();

    BSP_ADC_Init();

    if (BSP_ADC_Start() == 0)
    {
        VOFA_Service_SendTextLine("ADC DMA start ok");
    }
    else
    {
        VOFA_Service_SendTextLine("ADC DMA start failed");
    }
}

void App_Start(void)
{
}

void App_Loop(void)
{
    uint16_t rank1;
    uint16_t rank2;
    float v1;
    float v2;

    rank1 = BSP_ADC_GetPhaseARaw();
    rank2 = BSP_ADC_GetPhaseBRaw();

    v1 = ((float)rank1 / 4095.0f) * 3.3f;
    v2 = ((float)rank2 / 4095.0f) * 3.3f;

    VOFA_Service_FireWater("rank1=%u,rank2=%u,v1=%.3f,v2=%.3f",
                           rank1,
                           rank2,
                           v1,
                           v2);

    HAL_Delay(100);
}
