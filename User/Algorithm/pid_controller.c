#include "pid_controller.h"

/* ============================================================
 * 内部工具函数
 * ============================================================ */

/**
 * @brief 浮点限幅
 */
static float PID_Controller_LimitFloat(float value,
                                       float min_value,
                                       float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/**
 * @brief 判断限幅参数是否有效
 */
static uint8_t PID_Controller_IsLimitValid(float min_value,
                                           float max_value)
{
    if (min_value >= max_value)
    {
        return 0U;
    }

    return 1U;
}

/* ============================================================
 * 对外接口
 * ============================================================ */

 /*
 * PID 控制器结构体定义
 */
void PID_Controller_Init(PID_Controller_t *pid,
                         float kp,
                         float ki,
                         float kd,
                         float output_min,
                         float output_max,
                         float integral_min,
                         float integral_max)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    if (PID_Controller_IsLimitValid(output_min, output_max) != 0U)
    {
        pid->output_min = output_min;
        pid->output_max = output_max;
    }
    else
    {
        pid->output_min = -1.0f;
        pid->output_max = 1.0f;
    }

    if (PID_Controller_IsLimitValid(integral_min, integral_max) != 0U)
    {
        pid->integral_min = integral_min;
        pid->integral_max = integral_max;
    }
    else
    {
        pid->integral_min = -1.0f;
        pid->integral_max = 1.0f;
    }

    PID_Controller_Reset(pid);
}

/* 复位 PID 内部状态 */
void PID_Controller_Reset(PID_Controller_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->setpoint = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->previous_error = 0.0f;

    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;

    pid->derivative = 0.0f;
    pid->output = 0.0f;

    pid->initialized = 0U;
}

/* 复位 PID，并指定初始积分项 */
void PID_Controller_ResetIntegral(PID_Controller_t *pid,
                                  float integral_value)
{
    if (pid == 0)
    {
        return;
    }

    pid->i_term = PID_Controller_LimitFloat(integral_value,
                                            pid->integral_min,
                                            pid->integral_max);

    pid->p_term = 0.0f;
    pid->d_term = 0.0f;
    pid->derivative = 0.0f;
    pid->output = PID_Controller_LimitFloat(pid->i_term,
                                            pid->output_min,
                                            pid->output_max);

    pid->error = 0.0f;
    pid->previous_error = 0.0f;
    pid->initialized = 0U;
}

/* 设置 PID 参数 */
void PID_Controller_SetGains(PID_Controller_t *pid,
                             float kp,
                             float ki,
                             float kd)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/* 设置 PID 输出限幅 */
void PID_Controller_SetOutputLimit(PID_Controller_t *pid,
                                   float output_min,
                                   float output_max)
{
    if (pid == 0)
    {
        return;
    }

    if (PID_Controller_IsLimitValid(output_min, output_max) == 0U)
    {
        return;
    }

    pid->output_min = output_min;
    pid->output_max = output_max;

    pid->output = PID_Controller_LimitFloat(pid->output,
                                            pid->output_min,
                                            pid->output_max);
}

/* 设置 PID 积分项限幅 */
void PID_Controller_SetIntegralLimit(PID_Controller_t *pid,
                                     float integral_min,
                                     float integral_max)
{
    if (pid == 0)
    {
        return;
    }

    if (PID_Controller_IsLimitValid(integral_min, integral_max) == 0U)
    {
        return;
    }

    pid->integral_min = integral_min;
    pid->integral_max = integral_max;

    pid->i_term = PID_Controller_LimitFloat(pid->i_term,
                                            pid->integral_min,
                                            pid->integral_max);
}

/* 使用 setpoint 和 feedback 更新 PID */
float PID_Controller_Update(PID_Controller_t *pid,
                            float setpoint,
                            float feedback,
                            float dt_s)
{
    float error;

    if (pid == 0)
    {
        return 0.0f;
    }

    pid->setpoint = setpoint;
    pid->feedback = feedback;

    error = setpoint - feedback;

    return PID_Controller_UpdateError(pid, error, dt_s);
}

/* 直接使用 error 更新 PID */
float PID_Controller_UpdateError(PID_Controller_t *pid,
                                 float error,
                                 float dt_s)
{
    float output;

    if (pid == 0)
    {
        return 0.0f;
    }

    if (dt_s <= 0.0f)
    {
        return pid->output;
    }

    pid->error = error;

    /*
     * 第一次进入时不计算微分，避免上电瞬间 D 项冲击。
     */
    if (pid->initialized == 0U)
    {
        pid->previous_error = error;
        pid->derivative = 0.0f;
        pid->initialized = 1U;
    }
    else
    {
        pid->derivative = (error - pid->previous_error) / dt_s;
    }

    /*
     * P 项
     */
    pid->p_term = pid->kp * error;

    /*
     * I 项
     *
     * 使用梯形积分：
     * I(k) = I(k-1) + Ki * Ts * 0.5 * [e(k) + e(k-1)]
     *
     * 这个形式和你旧代码中的积分方式一致，但周期由外部 dt_s 传入，
     * 不再写死 Ts / Ts1 / Ts2。
     */
    pid->i_term =
        pid->i_term
        + pid->ki * dt_s * 0.5f * (error + pid->previous_error);

    pid->i_term = PID_Controller_LimitFloat(pid->i_term,
                                            pid->integral_min,
                                            pid->integral_max);

    /*
     * D 项
     *
     * 电流环里通常先把 Kd 设置为 0。
     */
    pid->d_term = pid->kd * pid->derivative;

    output = pid->p_term + pid->i_term + pid->d_term;

    output = PID_Controller_LimitFloat(output,
                                       pid->output_min,
                                       pid->output_max);

    pid->output = output;
    pid->previous_error = error;

    return pid->output;
}

/* 获取 PID 当前输出 */
float PID_Controller_GetOutput(const PID_Controller_t *pid)
{
    if (pid == 0)
    {
        return 0.0f;
    }

    return pid->output;
}

/* 获取 PID 当前误差 */
float PID_Controller_GetError(const PID_Controller_t *pid)
{
    if (pid == 0)
    {
        return 0.0f;
    }

    return pid->error;
}

/* 获取 PID 当前积分项 */
float PID_Controller_GetIntegral(const PID_Controller_t *pid)
{
    if (pid == 0)
    {
        return 0.0f;
    }

    return pid->i_term;
}

/* 获取 PID 当前 P 项 */
float PID_Controller_GetPTerm(const PID_Controller_t *pid)
{
    if (pid == 0)
    {
        return 0.0f;
    }

    return pid->p_term;
}

/* 获取 PID 当前 I 项 */
float PID_Controller_GetITerm(const PID_Controller_t *pid)
{
    if (pid == 0)
    {
        return 0.0f;
    }

    return pid->i_term;
}

/* 获取 PID 当前 D 项 */
float PID_Controller_GetDTerm(const PID_Controller_t *pid)
{
    if (pid == 0)
    {
        return 0.0f;
    }

    return pid->d_term;
}
