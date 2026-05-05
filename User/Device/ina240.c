#include "ina240.h"

#include "bsp_adc.h"
#include "platform_delay.h"

#include <string.h>

/* ============================================================
 * INA240 参数配置区
 *
 * 后续换采样电阻、INA240 型号、ADC 参考电压，只改这里。
 * ============================================================ */

#define INA240_ADC_REF_VOLTAGE         3.3f         /**< ADC 参考电压 */
#define INA240_ADC_MAX_VALUE           4095.0f      /**< ADC 原始值最大值 */

#define INA240_GAIN                    50.0f        /**< INA240 增益 */
#define INA240_SHUNT_RESISTOR          0.010f       /**< INA240 采样电阻 */
#define INA240_DEFAULT_OFFSET_VOLTAGE  1.65f        /**< INA240 默认零点偏置电压 */
#define INA240_CALIBRATE_DELAY_MS      1U           /**< 校准零点偏置时每次采样的间隔，单位 ms */

#define INA240_CURRENT_A_DIRECTION     1.0f         /**< A 相电流方向修正 */
#define INA240_CURRENT_B_DIRECTION     1.0f         /**< B 相电流方向修正 */

/* ============================================================
 * 模块内部变量
 * ============================================================ */

static INA240_Data_t s_ina240_data;                 /**< INA240 数据结构体实例 */
static float s_volt_to_current_gain = 0.0f;         /**< 电压转换为电流的系数 */

/* ============================================================
 * 内部工具函数
 * ============================================================ */

/**
 * @brief 检查 INA240 参数是否合法
 *
 * @return INA240_OK 参数正常
 * @return INA240_ERROR_PARAM 参数错误
 */
static INA240_Status_t INA240_CheckConfig(void)
{
    if (INA240_GAIN <= 0.0f)
    {
        return INA240_ERROR_PARAM;
    }

    if (INA240_SHUNT_RESISTOR <= 0.0f)
    {
        return INA240_ERROR_PARAM;
    }

    if (INA240_ADC_REF_VOLTAGE <= 0.0f)
    {
        return INA240_ERROR_PARAM;
    }

    if (INA240_ADC_MAX_VALUE <= 0.0f)
    {
        return INA240_ERROR_PARAM;
    }

    return INA240_OK;
}

/* ============================================================
 * 对外接口
 * ============================================================ */

 /**
 * @brief 初始化 INA240 模块
 *
 * 主要是清零数据结构体，设置默认偏置电压，并计算电压转电流的系数。
 */
void INA240_Init(void)
{
    memset(&s_ina240_data, 0, sizeof(s_ina240_data));

    s_ina240_data.offset_a = INA240_DEFAULT_OFFSET_VOLTAGE;
    s_ina240_data.offset_b = INA240_DEFAULT_OFFSET_VOLTAGE;

    if (INA240_CheckConfig() == INA240_OK)
    {
        s_volt_to_current_gain =
            1.0f / (INA240_GAIN * INA240_SHUNT_RESISTOR);
    }
    else
    {
        s_volt_to_current_gain = 0.0f;
    }
}

/**
 * @brief 更新 INA240 数据
 *
 * 从 ADC 获取最新数据，并计算电流值。
 *
 * @return INA240_OK 更新成功
 * @return INA240_ERROR_PARAM 参数错误
 */
INA240_Status_t INA240_Update(void)
{
    if (INA240_CheckConfig() != INA240_OK)
    {
        return INA240_ERROR_PARAM;
    }

    if (s_volt_to_current_gain <= 0.0f)
    {
        return INA240_ERROR_PARAM;
    }

    /*
     * 从 BSP_ADC 获取 ADC DMA 缓冲区中的原始值。
     *
     * 当前映射：
     * Current A -> ADC Rank 1 -> PA0 / ADC1_IN1
     * Current B -> ADC Rank 2 -> PA1 / ADC1_IN2
     */
    s_ina240_data.raw_a = BSP_ADC_GetPhaseARaw();
    s_ina240_data.raw_b = BSP_ADC_GetPhaseBRaw();

    /*
     * ADC 原始值转换为 INA240 输出电压。
     */
    s_ina240_data.voltage_a = INA240_RawToVoltage(s_ina240_data.raw_a);
    s_ina240_data.voltage_b = INA240_RawToVoltage(s_ina240_data.raw_b);

    /*
     * 电流换算：
     *
     * INA240 输出电压：
     * vout = offset + current * shunt_resistor * gain
     *
     * 所以：
     * current = (vout - offset) / (shunt_resistor * gain)
     */
    s_ina240_data.current_a =
        (s_ina240_data.voltage_a - s_ina240_data.offset_a)
        * s_volt_to_current_gain
        * INA240_CURRENT_A_DIRECTION;

    s_ina240_data.current_b =
        (s_ina240_data.voltage_b - s_ina240_data.offset_b)
        * s_volt_to_current_gain
        * INA240_CURRENT_B_DIRECTION;

    /*
     * 三相电流满足：
     * ia + ib + ic = 0
     *
     * 如果只采 A/B 两相，C 相可以通过该公式估算。
     */
    s_ina240_data.current_c =
        -(s_ina240_data.current_a + s_ina240_data.current_b);

    return INA240_OK;
}

/**
 * @brief 校准 INA240 零点偏置
 *
 * @param sample_count 采样次数
 */
void INA240_CalibrateOffset(uint16_t sample_count)
{
    uint32_t sum_a = 0U;
    uint32_t sum_b = 0U;
    uint16_t i;

    if (sample_count == 0U)
    {
        return;
    }

    /*
     * 校准前应确保：
     * 1. 电机没有通电
     * 2. PWM 没有输出
     * 3. 功率管没有导通
     */
    for (i = 0U; i < sample_count; i++)
    {
        sum_a += BSP_ADC_GetPhaseARaw();
        sum_b += BSP_ADC_GetPhaseBRaw();

        Platform_DelayMs(INA240_CALIBRATE_DELAY_MS);
    }

    s_ina240_data.offset_a =
        INA240_RawToVoltage((uint16_t)(sum_a / sample_count));

    s_ina240_data.offset_b =
        INA240_RawToVoltage((uint16_t)(sum_b / sample_count));
}


/**
 * @brief 将 ADC 原始值转换为电压
 *
 * @param raw ADC 原始值，范围通常为 0 ~ 4095。
 * @return 转换后的电压值，单位 V。
 */
float INA240_RawToVoltage(uint16_t raw)
{
    return ((float)raw / INA240_ADC_MAX_VALUE) * INA240_ADC_REF_VOLTAGE;
}


/**
 * @brief 获取 A 相 ADC 原始值
 */
uint16_t INA240_GetRawA(void)
{
    return s_ina240_data.raw_a;
}

/**
 * @brief 获取 B 相 ADC 原始值
 */
uint16_t INA240_GetRawB(void)
{
    return s_ina240_data.raw_b;
}

/**
 * @brief 获取 A 相电压
 */
float INA240_GetVoltageA(void)
{
    return s_ina240_data.voltage_a;
}

/**
 * @brief 获取 B 相电压
 */
float INA240_GetVoltageB(void)
{
    return s_ina240_data.voltage_b;
}

/**
 * @brief 获取 A 相电流
 */
float INA240_GetCurrentA(void)
{
    return s_ina240_data.current_a;
}

/**
 * @brief 获取 B 相电流
 */
float INA240_GetCurrentB(void)
{
    return s_ina240_data.current_b;
}

/**
 * @brief 获取 C 相电流
 */
float INA240_GetCurrentC(void)
{
    return s_ina240_data.current_c;
}

/**
 * @brief 获取 A 相偏置电压
 */
float INA240_GetOffsetA(void)
{
    return s_ina240_data.offset_a;
}

/**
 * @brief 获取 B 相偏置电压
 */
float INA240_GetOffsetB(void)
{
    return s_ina240_data.offset_b;
}

/**
 * @brief 获取完整 INA240 数据结构体指针
 *
 * 注意：
 * 返回的是模块内部静态变量地址，只建议读，不建议外部修改。
 */
const INA240_Data_t *INA240_GetData(void)
{
    return &s_ina240_data;
}

