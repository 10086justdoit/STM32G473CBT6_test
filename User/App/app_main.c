#include "app_main.h"
#include "main.h"

#include "bsp_uart.h"
#include "vofa_service.h"
#include "motor_service.h"

#define APP_MOTOR_DT_S              0.001f      /* 电机服务更新周期，单位 s */

#define APP_OPEN_LOOP_TARGET_UQ     1.2f        /* 开环目标 Uq 电压幅值 */
#define APP_OPEN_LOOP_FREQ_HZ       0.8f        /* 开环电角度旋转频率，单位 Hz */
#define APP_UQ_RAMP_STEP            0.0005f     /* 开环电压幅值每次更新增加的步长，单位 V */

#define APP_ENABLE_DELAY_COUNT      1000U       /* 启动后等待多少次循环（约 1s）再使能驱动，单位 count */
#define APP_CURRENT_LIMIT_A         1.0f        /* 电流限制，单位 A，超过这个值就停机 */

static uint16_t s_print_count = 0U;             /* 用于控制打印频率的计数器 */
static uint16_t s_enable_delay_count = 0U;      /* 用于控制启动后延迟使能驱动的计数器 */

static float s_current_uq = 0.0f;               /* 当前开环 Uq 电压幅值 */
static uint8_t s_driver_enabled = 0U;

void App_Init(void)
{
    Motor_Service_Status_t status;

    BSP_UART_Init();
    VOFA_Service_Init();

    status = Motor_Service_Init();

    if (status == MOTOR_SERVICE_OK)
    {
        VOFA_Service_SendTextLine("Motor_Service init ok");
    }
    else
    {
        VOFA_Service_FireWater("Motor_Service init failed,%d", status);
    }

    Motor_Service_StartOpenLoopElectrical(0.0f, APP_OPEN_LOOP_FREQ_HZ);
    Motor_Service_DisableDriver();

    s_current_uq = 0.0f;
    s_driver_enabled = 0U;
    s_enable_delay_count = 0U;

    VOFA_Service_SendTextLine("Motor open-loop prepare");
}

void App_Start(void)
{
}

void App_Loop(void)
{
    Motor_Service_Status_t status;
    const Motor_Service_Data_t *motor;

    if (s_driver_enabled == 0U)
    {
        s_enable_delay_count++;

        if (s_enable_delay_count >= APP_ENABLE_DELAY_COUNT)
        {
            Motor_Service_EnableDriver();
            s_driver_enabled = 1U;

            VOFA_Service_SendTextLine("Motor driver enabled");
        }
    }
    else
    {
        if (s_current_uq < APP_OPEN_LOOP_TARGET_UQ)
        {
            s_current_uq += APP_UQ_RAMP_STEP;

            if (s_current_uq > APP_OPEN_LOOP_TARGET_UQ)
            {
                s_current_uq = APP_OPEN_LOOP_TARGET_UQ;
            }

            Motor_Service_SetOpenLoopVoltage(s_current_uq);
        }
    }

    status = Motor_Service_Update(APP_MOTOR_DT_S);

    if (status != MOTOR_SERVICE_OK)
    {
        Motor_Service_Stop();
        VOFA_Service_FireWater("Motor_Service error,%d", status);
        HAL_Delay(100);
        return;
    }

    motor = Motor_Service_GetData();

    if ((motor->ia_lpf > APP_CURRENT_LIMIT_A) ||
        (motor->ia_lpf < -APP_CURRENT_LIMIT_A) ||
        (motor->ib_lpf > APP_CURRENT_LIMIT_A) ||
        (motor->ib_lpf < -APP_CURRENT_LIMIT_A))
    {
        Motor_Service_Stop();
        VOFA_Service_SendTextLine("over current stop");
        HAL_Delay(500);
        return;
    }

    s_print_count++;

    if (s_print_count >= 100U)
    {
        s_print_count = 0U;

        VOFA_Service_FireWater("%.3f,%u,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.3f,%u",
                               motor->open_loop_electrical_angle,
                               motor->svpwm.sector,
                               motor->duty.duty_a,
                               motor->duty.duty_b,
                               motor->duty.duty_c,
                               s_current_uq,
                               motor->ia_lpf,
                               motor->ib_lpf,
                               motor->electrical_angle,
                               motor->driver_enabled);
    }

    HAL_Delay(1);
}
