#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

/**
 * @brief PID 控制器状态码
 */
typedef enum
{
    PID_CONTROLLER_OK = 0,
    PID_CONTROLLER_ERROR_PARAM = -1
} PID_Controller_Status_t;

/**
 * @brief 通用 PID 控制器对象
 *
 * 说明：
 * 1. 一个 PID_Controller_t 对象对应一个 PID 控制器。
 * 2. Id 环、Iq 环、速度环、位置环都可以各自定义一个对象。
 * 3. PID 算法模块不依赖 FOC、INA240、PWM、AS5048A。
 */
typedef struct
{
    float kp;               /* 比例系数 */
    float ki;               /* 积分系数 */ 
    float kd;               /* 微分系数 */

    float output_min;       /* 输出下限 */
    float output_max;       /* 输出上限 */

    float integral_min;     /* 积分项下限 */
    float integral_max;     /* 积分项上限 */

    float setpoint;         /* 目标值 */
    float feedback;         /* 反馈值 */
    float error;            /* 当前误差 */
    float previous_error;   /* 上一次误差 */

    float p_term;           /* P 项 */
    float i_term;           /* I 项 */
    float d_term;           /* D 项 */

    float derivative;       /* 微分项 */
    float output;           /* 输出项 */

    uint8_t initialized;    /* 初始化标志 */
} PID_Controller_t;

/**
 * @brief 初始化 PID 控制器
 */
void PID_Controller_Init(PID_Controller_t *pid,
                         float kp,
                         float ki,
                         float kd,
                         float output_min,
                         float output_max,
                         float integral_min,
                         float integral_max);

/**
 * @brief 复位 PID 内部状态
 */
void PID_Controller_Reset(PID_Controller_t *pid);

/**
 * @brief 复位 PID，并指定初始积分项
 */
void PID_Controller_ResetIntegral(PID_Controller_t *pid,
                                  float integral_value);

/**
 * @brief 设置 PID 参数
 */
void PID_Controller_SetGains(PID_Controller_t *pid,
                             float kp,
                             float ki,
                             float kd);

/**
 * @brief 设置 PID 输出限幅
 */
void PID_Controller_SetOutputLimit(PID_Controller_t *pid,
                                   float output_min,
                                   float output_max);

/**
 * @brief 设置 PID 积分项限幅
 */
void PID_Controller_SetIntegralLimit(PID_Controller_t *pid,
                                     float integral_min,
                                     float integral_max);

/**
 * @brief 使用 setpoint 和 feedback 更新 PID
 *
 * error = setpoint - feedback
 *
 */
float PID_Controller_Update(PID_Controller_t *pid,
                            float setpoint,
                            float feedback,
                            float dt_s);

/**
 * @brief 直接使用 error 更新 PID
 *
 * @param pid PID 控制器对象
 * @param error 当前误差
 * @param dt_s 控制周期，单位 s
 *
 * @return PID 输出
 */
float PID_Controller_UpdateError(PID_Controller_t *pid,
                                 float error,
                                 float dt_s);

/**
 * @brief 获取 PID 当前输出
 */
float PID_Controller_GetOutput(const PID_Controller_t *pid);

/**
 * @brief 获取 PID 当前误差
 */
float PID_Controller_GetError(const PID_Controller_t *pid);

/**
 * @brief 获取 PID 当前积分项
 */
float PID_Controller_GetIntegral(const PID_Controller_t *pid);

/**
 * @brief 获取 PID 当前 P 项
 */
float PID_Controller_GetPTerm(const PID_Controller_t *pid);

/**
 * @brief 获取 PID 当前 I 项
 */
float PID_Controller_GetITerm(const PID_Controller_t *pid);

/**
 * @brief 获取 PID 当前 D 项
 */
float PID_Controller_GetDTerm(const PID_Controller_t *pid);

#endif
