#include "low_pass_filter.h"


/**
 * @brief 限制 alpha 在 0.0 ~ 1.0 范围内
 */
static float LowPassFilter_LimitAlpha(float alpha)
{
    if (alpha < 0.0f)
    {
        return 0.0f;
    }

    if (alpha > 1.0f)
    {
        return 1.0f;
    }

    return alpha;
}

/* ============================================================
 * 低通滤波器函数实现
 * ============================================================ */

/**
 * @brief 低通滤波器函数实现
 *
 * 功能：
 * 1. 初始化滤波器，设置 alpha，重置输出值和初始化标志
 * 2. 重置滤波器输出值，并设置初始化标志
 * 3. 更新滤波器输出值，使用一阶低通滤波公式
 * 4. 设置滤波系数 alpha，范围限制在 0.0 ~ 1.0
 * 5. 获取当前滤波器输出值
 *
 * 调用要求：
 * 1. 传入的 filter 指针必须有效
 * 2. alpha 的合理范围是 0.0 ~ 1.0，超出范围会被限制
 */
void LowPassFilter_Init(LowPassFilter_t *filter, float alpha)
{
    if (filter == 0)
    {
        return;
    }

    filter->alpha = LowPassFilter_LimitAlpha(alpha);
    filter->output = 0.0f;
    filter->initialized = 0U;
}

/**
 * @brief 重置滤波器输出值
 *
 * @param filter 低通滤波器对象指针
 * @param value 重置后的输出值
 */
void LowPassFilter_Reset(LowPassFilter_t *filter, float value)
{
    if (filter == 0)
    {
        return;
    }

    filter->output = value;
    filter->initialized = 1U;
}

/**
 * @brief 更新滤波器输出值
 *
 * @param filter 低通滤波器对象指针
 * @param input 当前输入值
 * @return 更新后的滤波器输出值
 */
float LowPassFilter_Update(LowPassFilter_t *filter, float input)
{
    if (filter == 0)
    {
        return input;
    }

    if (filter->initialized == 0U)
    {
        filter->output = input;
        filter->initialized = 1U;
        return filter->output;
    }

    filter->output = filter->output + filter->alpha * (input - filter->output);

    return filter->output;
}

/**
 * @brief 设置滤波系数 alpha
 *
 * @param filter 低通滤波器对象指针
 * @param alpha 滤波系数，范围 0.0 ~ 1.0
 */
void LowPassFilter_SetAlpha(LowPassFilter_t *filter, float alpha)
{
    if (filter == 0)
    {
        return;
    }

    filter->alpha = LowPassFilter_LimitAlpha(alpha);
}

/**
 * @brief 获取当前滤波器输出值
 *
 * @param filter 低通滤波器对象指针
 * @return 当前滤波器输出值
 */
float LowPassFilter_GetOutput(const LowPassFilter_t *filter)
{
    if (filter == 0)
    {
        return 0.0f;
    }

    return filter->output;
}
