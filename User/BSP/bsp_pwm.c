#include "bsp_pwm.h"
#include "main.h"
#include "tim.h"

/* ============================================================
 * PWM 硬件配置区
 *
 * 当前 CubeMX 配置：
 * TIM1_CH1 -> PA8
 * TIM1_CH2 -> PA9
 * TIM1_CH3 -> PA10
 * DRV_EN   -> PB15
 *
 * 后续如果换定时器、通道或者驱动使能引脚，只改这里。
 * ============================================================ */

#define BSP_PWM_TIMER_HANDLE            htim1

#define BSP_PWM_TIM_CHANNEL_A           TIM_CHANNEL_1
#define BSP_PWM_TIM_CHANNEL_B           TIM_CHANNEL_2
#define BSP_PWM_TIM_CHANNEL_C           TIM_CHANNEL_3

#define BSP_PWM_DRV_EN_GPIO_PORT        GPIOB
#define BSP_PWM_DRV_EN_GPIO_PIN         GPIO_PIN_15

/* ============================================================
 * 模块内部变量
 * ============================================================ */

static float s_duty_a = 0.0f;
static float s_duty_b = 0.0f;
static float s_duty_c = 0.0f;

/* ============================================================
 * 内部工具函数
 * ============================================================ */

/**
 * @brief 限制占空比范围
 *
 * @param duty 输入占空比
 *
 * @return 限制后的占空比，范围 0.0f ~ 1.0f
 */
static float BSP_PWM_LimitDuty(float duty)
{
    if (duty < 0.0f)
    {
        return 0.0f;
    }

    if (duty > 1.0f)
    {
        return 1.0f;
    }

    return duty;
}

/**
 * @brief 将占空比转换为 CCR 比较值
 *
 * @param duty 占空比，范围 0.0f ~ 1.0f
 *
 * @return CCR 比较值
 */
static uint32_t BSP_PWM_DutyToCompare(float duty)
{
    uint32_t arr;
    uint32_t compare;

    duty = BSP_PWM_LimitDuty(duty);

    /*
     * TIMx->ARR 是自动重装载值。
     *
     * 你现在是 Center Aligned mode1：
     * PWM 频率 = TIM_CLK / [2 * (ARR + 1)]
     *
     * 但占空比换算仍然可以按照：
     * CCR = duty * (ARR + 1)
     */
    arr = __HAL_TIM_GET_AUTORELOAD(&BSP_PWM_TIMER_HANDLE);

    compare = (uint32_t)((float)(arr + 1U) * duty);

    if (compare > arr)
    {
        compare = arr;
    }

    return compare;
}

/**
 * @brief 设置指定 TIM 通道比较值
 *
 * @param tim_channel TIM_CHANNEL_1 / TIM_CHANNEL_2 / TIM_CHANNEL_3
 * @param duty 占空比
 */
static void BSP_PWM_SetCompare(uint32_t tim_channel, float duty)
{
    uint32_t compare;

    compare = BSP_PWM_DutyToCompare(duty);

    __HAL_TIM_SET_COMPARE(&BSP_PWM_TIMER_HANDLE, tim_channel, compare);
}

/* ============================================================
 * 对外接口
 * ============================================================ */

/**
 * @brief 初始化 PWM BSP 模块
 *
 * 功能：
 * 1. 关闭驱动使能
 * 2. 三相占空比清零
 *
 * 注意：
 * TIM1 的底层初始化由 CubeMX 的 MX_TIM1_Init() 完成。
 */
void BSP_PWM_Init(void)
{
    BSP_PWM_DisableDriver();

    BSP_PWM_SetDutyABC(0.0f, 0.0f, 0.0f);
}

/**
 * @brief 启动三路 PWM 输出
 *
 * @return 0 启动成功
 * @return -1 CH1 启动失败
 * @return -2 CH2 启动失败
 * @return -3 CH3 启动失败
 */
int BSP_PWM_Start(void)
{
    if (HAL_TIM_PWM_Start(&BSP_PWM_TIMER_HANDLE, BSP_PWM_TIM_CHANNEL_A) != HAL_OK)
    {
        return -1;
    }

    if (HAL_TIM_PWM_Start(&BSP_PWM_TIMER_HANDLE, BSP_PWM_TIM_CHANNEL_B) != HAL_OK)
    {
        return -2;
    }

    if (HAL_TIM_PWM_Start(&BSP_PWM_TIMER_HANDLE, BSP_PWM_TIM_CHANNEL_C) != HAL_OK)
    {
        return -3;
    }

    return 0;
}

/**
 * @brief 停止三路 PWM 输出
 *
 * @return 0 固定返回成功
 */
int BSP_PWM_Stop(void)
{
    BSP_PWM_SetDutyABC(0.0f, 0.0f, 0.0f);

    (void)HAL_TIM_PWM_Stop(&BSP_PWM_TIMER_HANDLE, BSP_PWM_TIM_CHANNEL_A);
    (void)HAL_TIM_PWM_Stop(&BSP_PWM_TIMER_HANDLE, BSP_PWM_TIM_CHANNEL_B);
    (void)HAL_TIM_PWM_Stop(&BSP_PWM_TIMER_HANDLE, BSP_PWM_TIM_CHANNEL_C);

    BSP_PWM_DisableDriver();

    return 0;
}

/**
 * @brief 使能驱动芯片
 *
 * 当前默认：
 * PB15 = 1，使能驱动
 */
void BSP_PWM_EnableDriver(void)
{
    HAL_GPIO_WritePin(BSP_PWM_DRV_EN_GPIO_PORT,
                      BSP_PWM_DRV_EN_GPIO_PIN,
                      GPIO_PIN_SET);
}

/**
 * @brief 关闭驱动芯片
 *
 * 当前默认：
 * PB15 = 0，关闭驱动
 */
void BSP_PWM_DisableDriver(void)
{
    HAL_GPIO_WritePin(BSP_PWM_DRV_EN_GPIO_PORT,
                      BSP_PWM_DRV_EN_GPIO_PIN,
                      GPIO_PIN_RESET);
}

/**
 * @brief 设置单路 PWM 占空比
 *
 * @param channel PWM 通道
 * @param duty 占空比，范围 0.0f ~ 1.0f
 */
void BSP_PWM_SetDuty(BSP_PWM_Channel_t channel, float duty)
{
    duty = BSP_PWM_LimitDuty(duty);

    if (channel == BSP_PWM_CHANNEL_A)
    {
        s_duty_a = duty;
        BSP_PWM_SetCompare(BSP_PWM_TIM_CHANNEL_A, duty);
    }
    else if (channel == BSP_PWM_CHANNEL_B)
    {
        s_duty_b = duty;
        BSP_PWM_SetCompare(BSP_PWM_TIM_CHANNEL_B, duty);
    }
    else if (channel == BSP_PWM_CHANNEL_C)
    {
        s_duty_c = duty;
        BSP_PWM_SetCompare(BSP_PWM_TIM_CHANNEL_C, duty);
    }
    else
    {
        /*
         * 非法通道，不处理。
         */
    }
}

/**
 * @brief 同时设置 A/B/C 三路 PWM 占空比
 *
 * @param duty_a A 相占空比
 * @param duty_b B 相占空比
 * @param duty_c C 相占空比
 */
void BSP_PWM_SetDutyABC(float duty_a, float duty_b, float duty_c)
{
    BSP_PWM_SetDuty(BSP_PWM_CHANNEL_A, duty_a);
    BSP_PWM_SetDuty(BSP_PWM_CHANNEL_B, duty_b);
    BSP_PWM_SetDuty(BSP_PWM_CHANNEL_C, duty_c);
}

/**
 * @brief 获取 A 相占空比
 */
float BSP_PWM_GetDutyA(void)
{
    return s_duty_a;
}

/**
 * @brief 获取 B 相占空比
 */
float BSP_PWM_GetDutyB(void)
{
    return s_duty_b;
}

/**
 * @brief 获取 C 相占空比
 */
float BSP_PWM_GetDutyC(void)
{
    return s_duty_c;
}
