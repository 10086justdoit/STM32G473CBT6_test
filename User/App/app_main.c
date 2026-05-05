#include "app_main.h"
#include "main.h"

#include "bsp_uart.h"
#include "vofa_service.h"
#include "bsp_pwm.h"

void App_Init(void)
{
    BSP_UART_Init();
    VOFA_Service_Init();

    BSP_PWM_Init();

    if (BSP_PWM_Start() == 0)
    {
        VOFA_Service_SendTextLine("PWM start ok");
    }
    else
    {
        VOFA_Service_SendTextLine("PWM start failed");
    }

    /*
     * 先关闭驱动，只测试 MCU 输出 PWM 波形。
     */
    BSP_PWM_DisableDriver();

    /*
     * 固定占空比测试：
     * PA8  / TIM1_CH1 = 20%
     * PA9  / TIM1_CH2 = 50%
     * PA10 / TIM1_CH3 = 80%
     */
    BSP_PWM_SetDutyABC(0.20f, 0.50f, 0.80f);
}

void App_Start(void)
{
}

void App_Loop(void)
{
    VOFA_Service_FireWater("%.3f,%.3f,%.3f",
                           BSP_PWM_GetDutyA(),
                           BSP_PWM_GetDutyB(),
                           BSP_PWM_GetDutyC());

    HAL_Delay(500);
}
