#include "foc.h"

#include <math.h>
#include <string.h>

/* ============================================================
 * FOC 参数配置区
 *
 * 后续换电机、换母线电压、换极对数，优先修改这里。
 * ============================================================ */

#define FOC_DEFAULT_BUS_VOLTAGE         12.0f           /**< 默认母线电压，单位 V */
#define FOC_DEFAULT_POLE_PAIRS          11U             /**< 默认极对数 */
#define FOC_DEFAULT_DIRECTION           1               /**< 默认旋转方向 */
#define FOC_MAX_MODULATION              0.57735026919f  /**< 最大调制指数，1/sqrt(3) */

/* ============================================================
 * 模块内部变量
 * ============================================================ */

static FOC_State_t s_foc_state;                /**< FOC 状态结构体实例 */

/* ============================================================
 * 内部工具函数
 * ============================================================ */

/**
 * @brief 计算 SVPWM 所在扇区
 *
 * @param angle_rad 电压矢量角度，单位 rad，范围自动归一化到 0 ~ 2pi
 *
 * @return 扇区编号，范围 1 ~ 6
 */
static uint8_t FOC_CalcSector(float angle_rad)
{
    uint8_t sector;

    angle_rad = FOC_NormalizeAngle(angle_rad);

    sector = (uint8_t)(angle_rad / FOC_PI_3) + 1U;

    if (sector < 1U)
    {
        sector = 1U;
    }
    else if (sector > 6U)
    {
        sector = 6U;
    }

    return sector;
}

/**
 * @brief 限制三相占空比在 0 ~ 1
 *
 * @param duty 输入占空比
 *
 * @return 限幅后的占空比
 */
static FOC_DutyABC_t FOC_LimitDutyABC(FOC_DutyABC_t duty)
{
    duty.duty_a = FOC_LimitFloat(duty.duty_a, 0.0f, 1.0f);
    duty.duty_b = FOC_LimitFloat(duty.duty_b, 0.0f, 1.0f);
    duty.duty_c = FOC_LimitFloat(duty.duty_c, 0.0f, 1.0f);

    return duty;
}

/* ============================================================
 * 初始化与参数配置
 * ============================================================ */

/**
 * @brief 初始化 FOC 算法模块
 * 功能：
 * 1. 初始化 FOC 状态结构体，设置默认参数 如母线电压、极对数、旋转方向等
 * 2. 初始化占空比为 0.5（无输出状态）
 */

void FOC_Init(void)
{
    memset(&s_foc_state, 0, sizeof(s_foc_state));

    s_foc_state.config.bus_voltage = FOC_DEFAULT_BUS_VOLTAGE;
    s_foc_state.config.pole_pairs = FOC_DEFAULT_POLE_PAIRS;
    s_foc_state.config.direction = FOC_DEFAULT_DIRECTION;
    s_foc_state.config.zero_electric_angle = 0.0f;

    s_foc_state.duty.duty_a = 0.5f;
    s_foc_state.duty.duty_b = 0.5f;
    s_foc_state.duty.duty_c = 0.5f;
}

/**
 * @brief 设置母线电压
 */
void FOC_SetBusVoltage(float bus_voltage)
{
    if (bus_voltage > 0.0f)
    {
        s_foc_state.config.bus_voltage = bus_voltage;
    }
}

/**
 * @brief 设置电机极对数
 *
 * @param pole_pairs 极对数
 */
void FOC_SetPolePairs(uint8_t pole_pairs)
{
    if (pole_pairs > 0U)
    {
        s_foc_state.config.pole_pairs = pole_pairs;
    }
}

/**
 * @brief 设置旋转方向
 *
 * @param direction 方向，>=0 表示正向，<0 表示反向
 */
void FOC_SetDirection(int8_t direction)
{
    if (direction >= 0)
    {
        s_foc_state.config.direction = 1;
    }
    else
    {
        s_foc_state.config.direction = -1;
    }
}

/**
 * @brief 设置电角度零偏
 *
 * @param zero_electric_angle 电角度零偏，单位 rad
 */
void FOC_SetZeroElectricAngle(float zero_electric_angle)
{
    s_foc_state.config.zero_electric_angle =
        FOC_NormalizeAngle(zero_electric_angle);
}

/* ============================================================
 * 参数获取函数
 * ============================================================ */

/* 获取母线电压 */    
float FOC_GetBusVoltage(void)
{
    return s_foc_state.config.bus_voltage;
}

/* 获取极对数 */
uint8_t FOC_GetPolePairs(void)
{
    return s_foc_state.config.pole_pairs;
}

/* 获取旋转方向 */
int8_t FOC_GetDirection(void)
{
    return s_foc_state.config.direction;
}

/* 获取电角度零偏 */
float FOC_GetZeroElectricAngle(void)
{
    return s_foc_state.config.zero_electric_angle;
}

/* 获取 FOC 状态结构体指针 */
const FOC_State_t *FOC_GetState(void)
{
    return &s_foc_state;
}

/* ============================================================
 * 基础数学工具
 * ============================================================ */

/**
 * @brief 浮点限幅
 */
float FOC_LimitFloat(float value, float min_value, float max_value)
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
 * @brief 角度归一化到 0 ~ 2pi
 *
 * @param angle_rad 输入角度，单位 rad
 *
 * @return 归一化后的角度，单位 rad
 */
float FOC_NormalizeAngle(float angle_rad)
{
    float angle;

    angle = fmodf(angle_rad, FOC_2PI);

    if (angle < 0.0f)
    {
        angle += FOC_2PI;
    }

    return angle;
}


/* ============================================================
 * Clarke / Park 坐标变换
 * ============================================================ */

/**
 * @brief Clarke 变换
 *
 * 输入：
 * ia, ib
 *
 * 假设：
 * ia + ib + ic = 0
 *
 * 输出：
 * i_alpha = ia
 * i_beta  = (ia + 2 * ib) / sqrt(3)
 */
FOC_AlphaBeta_t FOC_Clarke(float ia, float ib)
{
    FOC_AlphaBeta_t output;

    output.alpha = ia;
    output.beta = (ia + 2.0f * ib) * FOC_INV_SQRT3;

    return output;
}

/**
 * @brief Park 变换
 *
 * alpha-beta 静止坐标系 -> d-q 旋转坐标系
 */
FOC_DQ_t FOC_Park(FOC_AlphaBeta_t input, float theta_rad)
{
    FOC_DQ_t output;
    float sin_t;
    float cos_t;

    sin_t = sinf(theta_rad);
    cos_t = cosf(theta_rad);

    output.d = input.alpha * cos_t + input.beta * sin_t;
    output.q = -input.alpha * sin_t + input.beta * cos_t;

    return output;
}

/**
 * @brief 反 Park 变换
 *
 * d-q 旋转坐标系 -> alpha-beta 静止坐标系
 */
FOC_AlphaBeta_t FOC_InvPark(FOC_DQ_t input, float theta_rad)
{
    FOC_AlphaBeta_t output;
    float sin_t;
    float cos_t;

    sin_t = sinf(theta_rad);
    cos_t = cosf(theta_rad);

    output.alpha = input.d * cos_t - input.q * sin_t;
    output.beta  = input.d * sin_t + input.q * cos_t;

    return output;
}

/**
 * @brief 反 Clarke 变换
 *
 * alpha-beta -> abc
 */
FOC_ABC_t FOC_InvClarke(FOC_AlphaBeta_t input)
{
    FOC_ABC_t output;

    output.a = input.alpha;
    output.b = -0.5f * input.alpha + 0.5f * FOC_SQRT3 * input.beta;
    output.c = -0.5f * input.alpha - 0.5f * FOC_SQRT3 * input.beta;

    return output;
}

/* ============================================================
 * 电角度与 SVPWM
 * ============================================================ */

/**
 * @brief 机械角度转换为电角度
 *
 * 公式：
 * electrical_angle = direction * pole_pairs * mechanical_angle - zero_offset
 *
 * @param mechanical_angle_rad 机械角度，单位 rad
 *
 * @return 电角度，单位 rad，范围 0 ~ 2pi
 */
float FOC_GetElectricalAngle(float mechanical_angle_rad)
{
    float electrical_angle;

    electrical_angle =
        (float)s_foc_state.config.direction
        * ((float)s_foc_state.config.pole_pairs * mechanical_angle_rad)
        - s_foc_state.config.zero_electric_angle;

    return FOC_NormalizeAngle(electrical_angle);
}

/**
 * @brief 计算 SVPWM 占空比
 *
 * 输入：
 * u_d, u_q              d/q 轴电压
 * electrical_angle_rad 电角度
 *
 * 输出：
 * svpwm                SVPWM 中间变量
 * duty                 三相占空比，范围 0 ~ 1
 *
 * 注意：
 * 本函数只计算占空比，不直接写 PWM。
 */
FOC_Status_t FOC_CalcSVPWM(float u_d,
                           float u_q,
                           float electrical_angle_rad,
                           FOC_SVPWM_Data_t *svpwm,
                           FOC_DutyABC_t *duty)
{
    float bus_voltage;
    float voltage_limit;
    float voltage_mag;
    float scale;

    float vector_angle;
    float u_ref;

    uint8_t sector;

    float t0;
    float t1;
    float t2;

    FOC_DutyABC_t duty_temp;

    if ((svpwm == 0) || (duty == 0))
    {
        return FOC_ERROR_PARAM;
    }

    bus_voltage = s_foc_state.config.bus_voltage;

    if (bus_voltage <= 0.0f)
    {
        return FOC_ERROR_PARAM;
    }

    /*
     * 限制电压矢量幅值，避免 SVPWM 过调制。
     */
    voltage_limit = FOC_MAX_MODULATION * bus_voltage;
    voltage_mag = sqrtf(u_d * u_d + u_q * u_q);

    if (voltage_mag > voltage_limit)
    {
        scale = voltage_limit / voltage_mag;

        u_d *= scale;
        u_q *= scale;

        voltage_mag = voltage_limit;
    }

    /*
     * 电压矢量角度。
     *
     * 当 u_d = 0, u_q > 0 时：
     * atan2f(u_q, u_d) = pi / 2
     *
     */
    vector_angle = electrical_angle_rad + atan2f(u_q, u_d);
    vector_angle = FOC_NormalizeAngle(vector_angle);

    u_ref = voltage_mag / bus_voltage;

    sector = FOC_CalcSector(vector_angle);

    /*
     * 计算两个有效矢量作用时间。
     *
     * 总 PWM 周期归一化为 1。
     */
    t1 = FOC_SQRT3
         * sinf((float)sector * FOC_PI_3 - vector_angle)
         * u_ref;

    t2 = FOC_SQRT3
         * sinf(vector_angle - ((float)sector - 1.0f) * FOC_PI_3)
         * u_ref;

    t0 = 1.0f - t1 - t2;

    if (t0 < 0.0f)
    {
        t0 = 0.0f;
    }

    switch (sector)
    {
        case 1U:
            duty_temp.duty_a = t1 + t2 + 0.5f * t0;
            duty_temp.duty_b = t2 + 0.5f * t0;
            duty_temp.duty_c = 0.5f * t0;
            break;

        case 2U:
            duty_temp.duty_a = t1 + 0.5f * t0;
            duty_temp.duty_b = t1 + t2 + 0.5f * t0;
            duty_temp.duty_c = 0.5f * t0;
            break;

        case 3U:
            duty_temp.duty_a = 0.5f * t0;
            duty_temp.duty_b = t1 + t2 + 0.5f * t0;
            duty_temp.duty_c = t2 + 0.5f * t0;
            break;

        case 4U:
            duty_temp.duty_a = 0.5f * t0;
            duty_temp.duty_b = t1 + 0.5f * t0;
            duty_temp.duty_c = t1 + t2 + 0.5f * t0;
            break;

        case 5U:
            duty_temp.duty_a = t2 + 0.5f * t0;
            duty_temp.duty_b = 0.5f * t0;
            duty_temp.duty_c = t1 + t2 + 0.5f * t0;
            break;

        case 6U:
            duty_temp.duty_a = t1 + t2 + 0.5f * t0;
            duty_temp.duty_b = 0.5f * t0;
            duty_temp.duty_c = t1 + 0.5f * t0;
            break;

        default:
            duty_temp.duty_a = 0.5f;
            duty_temp.duty_b = 0.5f;
            duty_temp.duty_c = 0.5f;
            break;
    }

    duty_temp = FOC_LimitDutyABC(duty_temp);

    svpwm->sector = sector;

    svpwm->t0 = t0;
    svpwm->t1 = t1;
    svpwm->t2 = t2;

    svpwm->ta = duty_temp.duty_a;
    svpwm->tb = duty_temp.duty_b;
    svpwm->tc = duty_temp.duty_c;

    svpwm->u_ref = u_ref;
    svpwm->vector_angle = vector_angle;

    *duty = duty_temp;

    return FOC_OK;
}

/**
 * @brief 使用机械角度计算 DQ 电压对应的 SVPWM 占空比
 *
 * 后面闭环 FOC 使用这个接口：
 * mechanical_angle_rad 由 AS5048A 提供。
 */
FOC_Status_t FOC_SetVoltageDQ(float u_d,
                              float u_q,
                              float mechanical_angle_rad)
{
    float electrical_angle;

    electrical_angle = FOC_GetElectricalAngle(mechanical_angle_rad);

    s_foc_state.mechanical_angle = mechanical_angle_rad;
    s_foc_state.electrical_angle = electrical_angle;

    return FOC_SetVoltageDQElectrical(u_d, u_q, electrical_angle);
}

/**
 * @brief 使用电角度计算 DQ 电压对应的 SVPWM 占空比
 *
 * 开环测试建议使用这个接口：
 * 直接给定 electrical_angle_rad。
 */
FOC_Status_t FOC_SetVoltageDQElectrical(float u_d,
                                        float u_q,
                                        float electrical_angle_rad)
{
    FOC_Status_t status;
    FOC_DutyABC_t duty;
    FOC_SVPWM_Data_t svpwm;

    electrical_angle_rad = FOC_NormalizeAngle(electrical_angle_rad);

    s_foc_state.electrical_angle = electrical_angle_rad;

    s_foc_state.voltage_dq.d = u_d;
    s_foc_state.voltage_dq.q = u_q;

    s_foc_state.voltage_alpha_beta =
        FOC_InvPark(s_foc_state.voltage_dq,
                    electrical_angle_rad);

    s_foc_state.voltage_abc =
        FOC_InvClarke(s_foc_state.voltage_alpha_beta);

    status = FOC_CalcSVPWM(u_d,
                           u_q,
                           electrical_angle_rad,
                           &svpwm,
                           &duty);

    if (status != FOC_OK)
    {
        return status;
    }

    s_foc_state.svpwm = svpwm;
    s_foc_state.duty = duty;

    return FOC_OK;
}

/**
 * @brief 获取当前三相占空比
 */
FOC_DutyABC_t FOC_GetDutyABC(void)
{
    return s_foc_state.duty;
}

/**
 * @brief 获取当前 SVPWM 中间变量
 */
FOC_SVPWM_Data_t FOC_GetSVPWMData(void)
{
    return s_foc_state.svpwm;
}

