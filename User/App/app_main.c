#include "app_main.h"
#include "main.h"

#include "bsp_uart.h"
#include "vofa_service.h"
#include "motor_service.h"
#include "as5048a.h"

#define APP_MOTOR_DT_S              0.001f

/*
 * 对齐完成后的开环验证参数
 */
#define APP_OPEN_LOOP_TARGET_UQ     0.8f
#define APP_OPEN_LOOP_FREQ_HZ       0.5f
#define APP_UQ_RAMP_STEP            0.0005f

/*
 * 电流保护
 */
#define APP_CURRENT_LIMIT_A         1.0f

static uint16_t s_print_count = 0U;
static float s_current_uq = 0.0f;
static uint8_t s_align_ok = 0U;

void App_Init(void)
{
    Motor_Service_Status_t status;
    const Motor_Service_Data_t *motor;

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
        return;
    }

    /*
     * 初始化完成后先保持关闭驱动。
     */
    Motor_Service_DisableDriver();

    VOFA_Service_SendTextLine("Align prepare");
    HAL_Delay(1000);

    /*
     * 开始零电角度对齐。
     * 注意：这个函数内部会使能驱动，并让电机产生力矩。
     */
    VOFA_Service_SendTextLine("Align start");

    status = Motor_Service_AlignSensor();

    if (status == MOTOR_SERVICE_OK)
    {
        motor = Motor_Service_GetData();

        s_align_ok = 1U;

        VOFA_Service_SendTextLine("Align ok");

        VOFA_Service_FireWater("zero=%.6f,dir=%d",
                               motor->zero_electric_angle,
                               motor->direction);

        /*
         * 对齐结束后，做一个低压开环验证。
         */
        s_current_uq = 0.0f;

        Motor_Service_StartOpenLoopElectrical(0.0f,
                                              APP_OPEN_LOOP_FREQ_HZ);

        Motor_Service_EnableDriver();

        VOFA_Service_SendTextLine("Open loop verify start");
    }
    else
    {
        s_align_ok = 0U;

        Motor_Service_Stop();

        VOFA_Service_FireWater("Align failed,%d", status);
    }
}

void App_Start(void)
{
}

void App_Loop(void)
{
    Motor_Service_Status_t status;
    const Motor_Service_Data_t *motor;

    if (s_align_ok == 0U)
    {
        HAL_Delay(100);
        return;
    }

    /*
     * 对齐成功后，Uq 慢慢爬升，验证电机能否平稳开环转动。
     */
    if (s_current_uq < APP_OPEN_LOOP_TARGET_UQ)
    {
        s_current_uq += APP_UQ_RAMP_STEP;

        if (s_current_uq > APP_OPEN_LOOP_TARGET_UQ)
        {
            s_current_uq = APP_OPEN_LOOP_TARGET_UQ;
        }

        Motor_Service_SetOpenLoopVoltage(s_current_uq);
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

    /*
     * 简单过流保护。
     */
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

    if (s_print_count >= 200U)
    {
        s_print_count = 0U;

        /*
         * 输出格式：
         *
         * I0  = open_loop_electrical_angle
         * I1  = sector
         * I2  = duty_a
         * I3  = duty_b
         * I4  = duty_c
         * I5  = uq
         * I6  = ia_lpf
         * I7  = ib_lpf
         * I8  = raw_angle
         * I9  = mechanical_angle
         * I10 = electrical_angle
         * I11 = velocity_rpm
         * I12 = zero_electric_angle
         * I13 = direction
         * I14 = driver_enabled
         */
        VOFA_Service_FireWater("%.3f,%u,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%u,%.3f,%.3f,%.3f,%.3f,%d,%u",
                               motor->open_loop_electrical_angle,
                               motor->svpwm.sector,
                               motor->duty.duty_a,
                               motor->duty.duty_b,
                               motor->duty.duty_c,
                               s_current_uq,
                               motor->ia_lpf,
                               motor->ib_lpf,
                               motor->raw_angle,
                               motor->mechanical_angle,
                               motor->electrical_angle,
                               AS5048A_GetVelocityRpm(),
                               motor->zero_electric_angle,
                               motor->direction,
                               motor->driver_enabled);
    }

    HAL_Delay(1);
}
