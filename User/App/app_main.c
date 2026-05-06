// #include "app_main.h"
// #include "main.h"

// #include "bsp_uart.h"
// #include "vofa_service.h"

// #include "motor_service.h"
// #include "ina240.h"

// /* ============================================================
//  * 电流环复测参数
//  * ============================================================ */

// /*
//  * Motor_Service_Update() 调用周期。
//  *
//  * 当前 App_Loop() 末尾使用 HAL_Delay(1)，所以近似 1ms。
//  */
// #define APP_MOTOR_DT_S                  0.001f

// /*
//  * 电流环测试目标。
//  *
//  * 当前只测试 Q 轴电流环：
//  * id_ref = 0
//  * iq_ref = 0.20A
//  */
// #define APP_ID_TARGET_REF               0.0f
// #define APP_IQ_TARGET_REF               -0.20f

// /*
//  * 启动后等待 1 秒再使能驱动。
//  */
// #define APP_ENABLE_DELAY_COUNT          1000U

// /*
//  * 软件过流保护。
//  *
//  * 当前只做低电流调试，先限制到 1A。
//  */
// #define APP_CURRENT_LIMIT_A             1.0f

// /*
//  * 打印周期。
//  *
//  * 100 表示约 100ms 打印一次。
//  * VOFA 的 Δt 建议设置为 100ms。
//  */
// #define APP_PRINT_DIVIDER               100U

// /* ============================================================
//  * 电流环测试状态机
//  * ============================================================ */

// typedef enum
// {
//     APP_CURRENT_TEST_WAIT_ENABLE = 0,       /* 等待驱动使能 */
//     APP_CURRENT_TEST_RUN,                   /* 电流环运行 */
//     APP_CURRENT_TEST_STOP                   /* 停止状态 */
// } App_Current_Test_State_t;

// /* ============================================================
//  * 模块内部变量
//  * ============================================================ */

// static App_Current_Test_State_t s_test_state = APP_CURRENT_TEST_WAIT_ENABLE;

// static uint16_t s_print_count = 0U;
// static uint16_t s_enable_delay_count = 0U;

// /* ============================================================
//  * App 初始化
//  * ============================================================ */

// void App_Init(void)
// {
//     Motor_Service_Status_t status;

//     BSP_UART_Init();
//     VOFA_Service_Init();

//     status = Motor_Service_Init();

//     if (status == MOTOR_SERVICE_OK)
//     {
//         VOFA_Service_SendTextLine("Motor_Service init ok");
//     }
//     else
//     {
//         VOFA_Service_FireWater("Motor_Service init failed,%d", status);
//         return;
//     }

//     /*
//      * 先进入电流环模式，但 iq_ref = 0。
//      * 驱动暂时关闭，防止初始化阶段误动作。
//      */
//     Motor_Service_StartCurrentLoop(APP_ID_TARGET_REF, 0.0f);
//     Motor_Service_DisableDriver();

//     s_print_count = 0U;
//     s_enable_delay_count = 0U;

//     s_test_state = APP_CURRENT_TEST_WAIT_ENABLE;

//     VOFA_Service_SendTextLine("Current loop retest prepare");
// }

// void App_Start(void)
// {
// }

// /* ============================================================
//  * 电流环测试状态机
//  * ============================================================ */

// static void App_CurrentTest_UpdateReference(void)
// {
//     switch (s_test_state)
//     {
//         case APP_CURRENT_TEST_WAIT_ENABLE:
//         {
//             s_enable_delay_count++;

//             if (s_enable_delay_count >= APP_ENABLE_DELAY_COUNT)
//             {
//                 /*
//                  * 使能驱动后重新启动电流环，
//                  * 目的是清除驱动关闭期间可能残留的 PID 状态。
//                  */
//                 Motor_Service_EnableDriver();
//                 Motor_Service_StartCurrentLoop(APP_ID_TARGET_REF, 0.0f);

//                 /*
//                  * 再给电流目标。
//                  */
//                 Motor_Service_SetCurrentReference(APP_ID_TARGET_REF,
//                                                   APP_IQ_TARGET_REF);

//                 s_test_state = APP_CURRENT_TEST_RUN;

//                 VOFA_Service_SendTextLine("Current loop driver enabled");
//                 VOFA_Service_SendTextLine("Set iq_ref 0.20A");
//             }

//             break;
//         }

//         case APP_CURRENT_TEST_RUN:
//         {
//             /*
//              * 持续保持电流目标。
//              *
//              * 测试方法：
//              * 手动慢慢转动电机一圈，
//              * 观察不同机械位置下 iq_meas 是否还能跟住 iq_ref。
//              */
//             Motor_Service_SetCurrentReference(APP_ID_TARGET_REF,
//                                               APP_IQ_TARGET_REF);
//             break;
//         }

//         case APP_CURRENT_TEST_STOP:
//         default:
//         {
//             Motor_Service_SetCurrentReference(0.0f, 0.0f);
//             break;
//         }
//     }
// }

// /* ============================================================
//  * App 主循环
//  * ============================================================ */

// void App_Loop(void)
// {
//     Motor_Service_Status_t status;
//     const Motor_Service_Data_t *motor;

//     /*
//      * 1. 更新测试目标。
//      */
//     App_CurrentTest_UpdateReference();

//     /*
//      * 2. 执行电机服务。
//      *
//      * 当前模式为 CURRENT_LOOP：
//      * 读取编码器角度和电流；
//      * 计算 id / iq；
//      * 根据 id_ref / iq_ref 计算 vd / vq；
//      * 通过 FOC/SVPWM 输出 PWM。
//      */
//     status = Motor_Service_Update(APP_MOTOR_DT_S);

//     if (status != MOTOR_SERVICE_OK)
//     {
//         Motor_Service_Stop();

//         s_test_state = APP_CURRENT_TEST_STOP;

//         VOFA_Service_FireWater("Motor_Service error,%d", status);

//         HAL_Delay(100);
//         return;
//     }

//     motor = Motor_Service_GetData();

//     /*
//      * 3. 软件过流保护。
//      */
//     if ((motor->ia_lpf > APP_CURRENT_LIMIT_A) ||
//         (motor->ia_lpf < -APP_CURRENT_LIMIT_A) ||
//         (motor->ib_lpf > APP_CURRENT_LIMIT_A) ||
//         (motor->ib_lpf < -APP_CURRENT_LIMIT_A))
//     {
//         Motor_Service_Stop();

//         s_test_state = APP_CURRENT_TEST_STOP;

//         VOFA_Service_SendTextLine("over current stop");

//         HAL_Delay(500);
//         return;
//     }

//     /*
//      * 4. VOFA 输出。
//      *
//      * I0  = iq_ref
//      * I1  = mechanical_angle
//      * I2  = electrical_angle
//      * I3  = raw_current_a
//      * I4  = raw_current_b
//      * I5  = ia_lpf
//      * I6  = ib_lpf
//      * I7  = id_meas
//      * I8  = iq_meas
//      * I9  = vq
//      * I10 = duty_a
//      * I11 = duty_b
//      * I12 = duty_c
//      * I13 = driver_enabled
//      * I14 = state
//      *
//      * 重点观察：
//      * iq_ref = 0.2000A
//      * iq_meas 是否接近 0.20A
//      * vq 是否长期顶到 2.5V
//      * rawA/rawB 是否有真实电流变化
//      */
//     s_print_count++;

//     if (s_print_count >= APP_PRINT_DIVIDER)
//     {
//         s_print_count = 0U;

//         VOFA_Service_FireWater("%.4f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%u,%d",
//                                motor->iq_ref,
//                                motor->mechanical_angle,
//                                motor->electrical_angle,
//                                INA240_GetCurrentA(),
//                                INA240_GetCurrentB(),
//                                motor->ia_lpf,
//                                motor->ib_lpf,
//                                motor->id_meas,
//                                motor->iq_meas,
//                                motor->vq,
//                                motor->duty.duty_a,
//                                motor->duty.duty_b,
//                                motor->duty.duty_c,
//                                motor->driver_enabled,
//                                (int)s_test_state);
//     }

//     HAL_Delay(1);
// }



// 




#include "app_main.h"
#include "main.h"

#include "bsp_uart.h"
#include "vofa_service.h"

#include "motor_service.h"

/* ============================================================
 * 电流环 PI 阶跃测试参数
 * ============================================================ */

#define APP_MOTOR_DT_S                  0.001f

/*
 * 初始软启动目标：0.2A
 * 阶跃目标：0.4A
 */
#define APP_ID_REF                      0.0f
#define APP_IQ_SOFT_TARGET_REF          -0.15f
#define APP_IQ_STEP_TARGET_REF          -0.20f

/*
 * iq_ref 软启动步进。
 *
 * 每 1ms 增加 0.00001A。
 * 从 0A 到 0.2A 约 4 秒。
 */
#define APP_IQ_RAMP_STEP                0.00001f

/*
 * 启动后等待 1 秒再使能驱动。
 */
#define APP_ENABLE_DELAY_COUNT          1000U

/*
 * 达到 0.2A 后，保持 3 秒，再阶跃到 0.4A。
 */
#define APP_LOW_CURRENT_HOLD_COUNT      10000U

/*
 * 阶跃到 0.4A 后，保持运行观察。
 */
#define APP_HIGH_CURRENT_HOLD_COUNT     10000U

/*
 * 软件过流保护。
 */
#define APP_CURRENT_LIMIT_A             1.0f

/*
 * 打印周期。
 *
 * 50 表示约 50ms 打印一次。
 * VOFA 的 Δt 建议设置为 50ms。
 */
#define APP_PRINT_DIVIDER               50U

/* ============================================================
 * 测试状态机
 * ============================================================ */

typedef enum
{
    APP_CURRENT_TEST_WAIT_ENABLE = 0,
    APP_CURRENT_TEST_RAMP_TO_0P2,
    APP_CURRENT_TEST_HOLD_0P2,
    APP_CURRENT_TEST_STEP_TO_0P4,
    APP_CURRENT_TEST_HOLD_0P4,
    APP_CURRENT_TEST_STOP
} App_Current_Test_State_t;

/* ============================================================
 * 模块内部变量
 * ============================================================ */

static App_Current_Test_State_t s_test_state = APP_CURRENT_TEST_WAIT_ENABLE;

static uint16_t s_print_count = 0U;
static uint16_t s_enable_delay_count = 0U;
static uint16_t s_hold_count = 0U;

static float s_iq_ref = 0.0f;
  

/* ============================================================
 * App 初始化
 * ============================================================ */

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
        return;
    }

    /*
     * 启动电流环模式，但初始 iq_ref = 0。
     */
    s_iq_ref = 0.0f;

    Motor_Service_StartCurrentLoop(APP_ID_REF, s_iq_ref);
    Motor_Service_DisableDriver();

 
    s_enable_delay_count = 0U;
    s_hold_count = 0U;
    s_print_count = 0U;

    s_test_state = APP_CURRENT_TEST_WAIT_ENABLE;

    VOFA_Service_SendTextLine("Current loop step test prepare");
}

void App_Start(void)
{
}

/* ============================================================
 * 电流目标状态机
 * ============================================================ */

static void App_CurrentTest_UpdateReference(void)
{
    switch (s_test_state)
    {
        case APP_CURRENT_TEST_WAIT_ENABLE:
        {
            s_enable_delay_count++;

            if (s_enable_delay_count >= APP_ENABLE_DELAY_COUNT)
            {
                Motor_Service_EnableDriver();

               
                s_iq_ref = 0.0f;

                Motor_Service_SetCurrentReference(APP_ID_REF, s_iq_ref);

                s_test_state = APP_CURRENT_TEST_RAMP_TO_0P2;

                VOFA_Service_SendTextLine("Current loop driver enabled");
                VOFA_Service_SendTextLine("Ramp to 0.2A");
            }
            break;
        }

        case APP_CURRENT_TEST_RAMP_TO_0P2:
        {
            /*
             * 软启动到 0.2A。
             */
            if (s_iq_ref < APP_IQ_SOFT_TARGET_REF)
            {
                s_iq_ref += APP_IQ_RAMP_STEP;

                if (s_iq_ref > APP_IQ_SOFT_TARGET_REF)
                {
                    s_iq_ref = APP_IQ_SOFT_TARGET_REF;
                }

                Motor_Service_SetCurrentReference(APP_ID_REF, s_iq_ref);
            }
            else
            {
                s_iq_ref = APP_IQ_SOFT_TARGET_REF;
                Motor_Service_SetCurrentReference(APP_ID_REF, s_iq_ref);

                s_hold_count = 0U;
                s_test_state = APP_CURRENT_TEST_HOLD_0P2;

                VOFA_Service_SendTextLine("Hold 0.2A");
            }
            break;
        }

        case APP_CURRENT_TEST_HOLD_0P2:
        {
            /*
             * 保持 0.2A，等待电流稳定。
             */
            s_iq_ref = APP_IQ_SOFT_TARGET_REF;
            Motor_Service_SetCurrentReference(APP_ID_REF, s_iq_ref);

            s_hold_count++;

            if (s_hold_count >= APP_LOW_CURRENT_HOLD_COUNT)
            {
                /*
                 * 突然阶跃到 0.4A。
                 */
                s_iq_ref = APP_IQ_STEP_TARGET_REF;
                Motor_Service_SetCurrentReference(APP_ID_REF, s_iq_ref);

                s_hold_count = 0U;
                s_test_state = APP_CURRENT_TEST_STEP_TO_0P4;

                VOFA_Service_SendTextLine("Step to 0.4A");
            }
            break;
        }

        case APP_CURRENT_TEST_STEP_TO_0P4:
        {
            /*
             * 进入 0.4A 保持阶段。
             */
            s_iq_ref = APP_IQ_STEP_TARGET_REF;
            Motor_Service_SetCurrentReference(APP_ID_REF, s_iq_ref);

            s_test_state = APP_CURRENT_TEST_HOLD_0P4;
            s_hold_count = 0U;
            break;
        }

        case APP_CURRENT_TEST_HOLD_0P4:
        {
            /*
             * 保持 0.4A，观察阶跃响应。
             */
            s_iq_ref = APP_IQ_STEP_TARGET_REF;
            Motor_Service_SetCurrentReference(APP_ID_REF, s_iq_ref);

            s_hold_count++;

            if (s_hold_count >= APP_HIGH_CURRENT_HOLD_COUNT)
            {
                /*
                 * 测试结束后继续保持 0.4A。
                 * 如果你想测试结束后自动停机，可以打开下面两行。
                 */
                /*
                Motor_Service_Stop();
                s_test_state = APP_CURRENT_TEST_STOP;
                */
            }
            break;
        }

        case APP_CURRENT_TEST_STOP:
        default:
        {
            s_iq_ref = 0.0f;
            Motor_Service_SetCurrentReference(APP_ID_REF, s_iq_ref);
            break;
        }
    }
}

/* ============================================================
 * App 主循环
 * ============================================================ */

void App_Loop(void)
{
    Motor_Service_Status_t status;
    const Motor_Service_Data_t *motor;

    /*
     * 1. 更新 iq_ref：
     *    0 -> 0.2A 软启动
     *    0.2A 保持
     *    0.2A -> 0.4A 阶跃
     */
    App_CurrentTest_UpdateReference();

    /*
     * 2. 执行电机服务。
     */
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
     * 3. 软件过流保护。
     */
    if ((motor->ia_lpf > APP_CURRENT_LIMIT_A) ||
        (motor->ia_lpf < -APP_CURRENT_LIMIT_A) ||
        (motor->ib_lpf > APP_CURRENT_LIMIT_A) ||
        (motor->ib_lpf < -APP_CURRENT_LIMIT_A))
    {
        Motor_Service_Stop();

        s_test_state = APP_CURRENT_TEST_STOP;

        VOFA_Service_SendTextLine("over current stop");

        HAL_Delay(500);
        return;
    }

    /*
     * 4. VOFA 输出。
     *
     * I0 = iq_ref
     * I1 = iq_meas
     * I2 = id_meas
     * I3 = vq
     * I4 = state
     */
    s_print_count++;

    if (s_print_count >= APP_PRINT_DIVIDER)
    {
        s_print_count = 0U;

        VOFA_Service_FireWater("%.4f,%.4f,%.4f,%.4f",
                               motor->iq_ref,
                               motor->iq_meas,
                               motor->id_meas,
                                motor->vq);
    }

    HAL_Delay(1);
}
