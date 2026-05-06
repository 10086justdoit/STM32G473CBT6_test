#include "app_main.h"
#include "main.h"

#include "bsp_uart.h"
#include "vofa_service.h"

#include "motor_service.h"

/* ============================================================
 * 速度环低速测试参数
 * ============================================================ */

/*
 * Motor_Service_Update() 调用周期。
 *
 * 当前 App_Loop() 末尾使用 HAL_Delay(1)，所以近似 1ms。
 * 后续如果放到定时器中断，需要和实际中断周期保持一致。
 */
#define APP_MOTOR_DT_S                  0.001f

/*
 * 启动后等待 1 秒再使能驱动。
 */
#define APP_ENABLE_DELAY_COUNT          1000U

/*
 * 速度环低速测试目标。
 *
 * 第一次速度环测试先使用低速：
 * 0rpm -> 10rpm -> 20rpm
 */
#define APP_SPEED_SOFT_TARGET_RPM       20.0f
#define APP_SPEED_STEP_TARGET_RPM       30.0f

/*
 * 速度给定爬坡步进。
 *
 * 当前每 1ms 增加 0.002rpm。
 * 从 0rpm 到 10rpm 约 5 秒。
 */
#define APP_SPEED_RAMP_STEP_RPM         0.002f

/*
 * 进入软启动目标后，等待一段时间再给阶跃目标。
 *
 * 当前 3000 表示约 3 秒。
 */
#define APP_STEP_DELAY_COUNT            3000U

/*
 * 软件保护参数。
 *
 * 速度环输出 iq_ref 在 motor_service.c 中已经有限幅。
 * 这里再做一层保护。
 */
#define APP_CURRENT_LIMIT_A             0.40f
#define APP_SPEED_LIMIT_RPM             180.0f

/*
 * 打印周期。
 *
 * 100 表示约 100ms 打印一次。
 * VOFA 的 Δt 建议设置为 100ms。
 */
#define APP_PRINT_DIVIDER               100U

/* ============================================================
 * 速度环测试状态机
 * ============================================================ */

typedef enum
{
    APP_SPEED_TEST_WAIT_ENABLE = 0,     /* 等待驱动使能 */
    APP_SPEED_TEST_SOFT_START,          /* 速度软启动 */
    APP_SPEED_TEST_STEP_RUN,            /* 速度阶跃运行 */
    APP_SPEED_TEST_STOP                 /* 停止状态 */
} App_Speed_Test_State_t;

/* ============================================================
 * 模块内部变量
 * ============================================================ */

static App_Speed_Test_State_t s_test_state = APP_SPEED_TEST_WAIT_ENABLE;

static uint16_t s_print_count = 0U;
static uint16_t s_enable_delay_count = 0U;
static uint16_t s_step_delay_count = 0U;
static uint16_t s_over_speed_count = 0U;
static float s_speed_ref_cmd = 0.0f;

/* ============================================================
 * 内部函数声明
 * ============================================================ */

static void App_SpeedTest_UpdateReference(void);
static void App_SpeedTest_CheckProtection(void);
static void App_SpeedTest_PrintData(void);

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
     * 初始状态关闭驱动。
     */
    Motor_Service_DisableDriver();

    /*
     * 先启动速度环模式，但目标速度为 0rpm。
     * 驱动还没有使能，所以不会转动。
     */
    Motor_Service_StartSpeedLoop(0.0f);

    s_print_count = 0U;
    s_enable_delay_count = 0U;
    s_step_delay_count = 0U;
    s_over_speed_count = 0U;
    s_speed_ref_cmd = 0.0f;

    s_test_state = APP_SPEED_TEST_WAIT_ENABLE;

    VOFA_Service_SendTextLine("Speed loop low speed test prepare");
}

void App_Start(void)
{
}

/* ============================================================
 * 速度环目标更新
 * ============================================================ */

static void App_SpeedTest_UpdateReference(void)
{
    switch (s_test_state)
    {
        case APP_SPEED_TEST_WAIT_ENABLE:
        {
            s_enable_delay_count++;

            if (s_enable_delay_count >= APP_ENABLE_DELAY_COUNT)
            {
                /*
                 * 使能驱动后重新启动速度环。
                 * 这样可以清除之前 PID 内部状态。
                 */
                Motor_Service_EnableDriver();
                Motor_Service_StartSpeedLoop(0.0f);

                s_speed_ref_cmd = 0.0f;
                s_step_delay_count = 0U;

                s_test_state = APP_SPEED_TEST_SOFT_START;

                VOFA_Service_SendTextLine("Speed loop driver enabled");
                VOFA_Service_SendTextLine("Speed soft start");
            }

            break;
        }

        case APP_SPEED_TEST_SOFT_START:
        {
            /*
             * 速度软启动：
             * 从 0rpm 缓慢爬升到 APP_SPEED_SOFT_TARGET_RPM。
             */
            if (s_speed_ref_cmd < APP_SPEED_SOFT_TARGET_RPM)
            {
                s_speed_ref_cmd += APP_SPEED_RAMP_STEP_RPM;

                if (s_speed_ref_cmd > APP_SPEED_SOFT_TARGET_RPM)
                {
                    s_speed_ref_cmd = APP_SPEED_SOFT_TARGET_RPM;
                }
            }
            else
            {
                s_step_delay_count++;

                if (s_step_delay_count >= APP_STEP_DELAY_COUNT)
                {
                    s_test_state = APP_SPEED_TEST_STEP_RUN;

                    VOFA_Service_SendTextLine("Speed step start");
                }
            }

            Motor_Service_SetSpeedReference(s_speed_ref_cmd);

            break;
        }

        case APP_SPEED_TEST_STEP_RUN:
        {
            /*
             * 阶跃目标：
             * 从软启动目标切换到阶跃目标。
             */
            s_speed_ref_cmd = APP_SPEED_STEP_TARGET_RPM;

            Motor_Service_SetSpeedReference(s_speed_ref_cmd);

            break;
        }

        case APP_SPEED_TEST_STOP:
        default:
        {
            s_speed_ref_cmd = 0.0f;

            Motor_Service_SetSpeedReference(0.0f);

            break;
        }
    }
}

/* ============================================================
 * 软件保护
 * ============================================================ */

static void App_SpeedTest_CheckProtection(void)
{
    const Motor_Service_Data_t *motor;

    motor = Motor_Service_GetData();

    /*
     * 电流保护。
     */
    if ((motor->ia_lpf > APP_CURRENT_LIMIT_A) ||
        (motor->ia_lpf < -APP_CURRENT_LIMIT_A) ||
        (motor->ib_lpf > APP_CURRENT_LIMIT_A) ||
        (motor->ib_lpf < -APP_CURRENT_LIMIT_A))
    {
        Motor_Service_Stop();

        s_test_state = APP_SPEED_TEST_STOP;

        VOFA_Service_SendTextLine("over current stop");

        HAL_Delay(500);
        return;
    }

     /*
     * 速度保护。
     *
     * 低速调试阶段，速度反馈偶尔会有尖峰。
     * 所以不要单次超过限幅就立刻停止，
     * 需要连续多次超过限幅才停机。
     */
    if ((motor->speed_rpm > APP_SPEED_LIMIT_RPM) ||
        (motor->speed_rpm < -APP_SPEED_LIMIT_RPM))
    {
        s_over_speed_count++;

        if (s_over_speed_count >= 100U)
        {
            Motor_Service_Stop();

            s_test_state = APP_SPEED_TEST_STOP;

            VOFA_Service_SendTextLine("over speed stop");

            HAL_Delay(500);
            return;
        }
    }
    else
    {
        s_over_speed_count = 0U;
    }
}

/* ============================================================
 * VOFA 数据输出
 * ============================================================ */

static void App_SpeedTest_PrintData(void)
{
    const Motor_Service_Data_t *motor;

    motor = Motor_Service_GetData();

    s_print_count++;

    if (s_print_count >= APP_PRINT_DIVIDER)
    {
        s_print_count = 0U;

        /*
         * I0  = speed_ref_rpm      目标转速，单位 rpm
         * I1  = speed_rpm          实际转速，单位 rpm
         * I2  = speed_error        速度误差，单位 rpm
         * I3  = speed_output       速度环输出，单位 A，也就是 iq_ref
         * I4  = iq_ref             Q 轴电流指令，单位 A
         * I5  = iq_meas            Q 轴电流反馈，单位 A
         * I6  = id_meas            D 轴电流反馈，单位 A
         * I7  = vq                 Q 轴电压输出，单位 V
         * I8  = mechanical_angle   机械角度，单位 rad
         * I9  = electrical_angle   电角度，单位 rad
         * I10 = duty_a             A 相占空比
         * I11 = duty_b             B 相占空比
         * I12 = duty_c             C 相占空比
         * I13 = driver_enabled     驱动使能状态
         * I14 = state              测试状态
         */
        VOFA_Service_FireWater("%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%d",
                               motor->speed_ref_rpm,
                               motor->speed_rpm,
                               motor->pid_speed_error,
                               motor->pid_speed_output,
                               motor->iq_ref,
                               motor->iq_meas,
                               motor->id_meas,
                               motor->vq,
                               motor->mechanical_angle,
                               motor->electrical_angle,
                               motor->duty.duty_a,
                               motor->duty.duty_b,
                               motor->duty.duty_c,
                               motor->driver_enabled,
                               (int)s_test_state);
    }
}

/* ============================================================
 * App 主循环
 * ============================================================ */

void App_Loop(void)
{
    Motor_Service_Status_t status;

    /*
     * 1. 更新速度目标。
     */
    App_SpeedTest_UpdateReference();

    /*
     * 2. 执行电机服务。
     *
     * 当前模式为 SPEED_LOOP：
     * 速度环每 10ms 输出 iq_ref；
     * 电流环每 1ms 执行一次；
     * 最终通过 FOC / SVPWM 输出 PWM。
     */
    status = Motor_Service_Update(APP_MOTOR_DT_S);

    if (status != MOTOR_SERVICE_OK)
    {
        Motor_Service_Stop();

        s_test_state = APP_SPEED_TEST_STOP;

        VOFA_Service_FireWater("Motor_Service error,%d", status);

        HAL_Delay(100);
        return;
    }

    /*
     * 3. 软件保护。
     */
    App_SpeedTest_CheckProtection();

    /*
     * 4. VOFA 输出。
     */
    App_SpeedTest_PrintData();

    HAL_Delay(1);
}
