#include "app_main.h"
#include "main.h"

#include "bsp_uart.h"
#include "vofa_service.h"
#include "bsp_adc.h"
#include "ina240.h"
#include "low_pass_filter.h"

static LowPassFilter_t g_lpf_current_a;
static LowPassFilter_t g_lpf_current_b;
static LowPassFilter_t g_lpf_current_c;

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

    INA240_Init();

    /*
     * 校准时确保电机不通电，PWM 没有输出。
     */
    INA240_CalibrateOffset(100);

    /*
     * 电流滤波系数。
     *
     * 0.05：滤波较强，响应较慢
     * 0.10：中等
     * 0.20：响应更快，滤波较弱
     */
    LowPassFilter_Init(&g_lpf_current_a, 0.05f);
    LowPassFilter_Init(&g_lpf_current_b, 0.05f);
    LowPassFilter_Init(&g_lpf_current_c, 0.05f);

    VOFA_Service_SendTextLine("INA240 + LPF start");
}

void App_Start(void)
{
}

void App_Loop(void)
{
    float ia;
    float ib;
    float ic;

    float ia_lpf;
    float ib_lpf;
    float ic_lpf;

    INA240_Update();

    ia = INA240_GetCurrentA();
    ib = INA240_GetCurrentB();
    ic = INA240_GetCurrentC();

    ia_lpf = LowPassFilter_Update(&g_lpf_current_a, ia);
    ib_lpf = LowPassFilter_Update(&g_lpf_current_b, ib);

    /*
     * 推荐：C 相由滤波后的 A/B 相计算
     */
    ic_lpf = -(ia_lpf + ib_lpf);

    VOFA_Service_FireWater("%u,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                           INA240_GetRawA(),
                           INA240_GetRawB(),
                           ia,
                           ib,
                           ic,
                           ia_lpf,
                           ib_lpf,
                           ic_lpf);

    HAL_Delay(50);
}

