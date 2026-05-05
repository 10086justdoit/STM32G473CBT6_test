#include "motor_service.h"

#include "main.h"

#include "platform_delay.h"

#include "bsp_spi.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"

#include "as5048a.h"
#include "ina240.h"
#include "low_pass_filter.h"
#include "foc.h"

#include <math.h>
#include <string.h>

/* ============================================================
 * Motor Service 配置区
 * ============================================================ */

/*
 * 是否要求编码器初始化必须成功。
 *
 * 0：编码器失败也允许开环 PWM 测试
 * 1：编码器失败则 Motor_Service_Init() 返回错误
 */
#define MOTOR_SERVICE_REQUIRE_ENCODER          0U
#define MOTOR_SERVICE_BUS_VOLTAGE              12.0f  /* 母线电压 */
#define MOTOR_SERVICE_POLE_PAIRS               11U    /* 极对数 */

#define MOTOR_SERVICE_OPEN_LOOP_UQ_DEFAULT     0.5f   /* 开环电压幅值 */
#define MOTOR_SERVICE_OPEN_LOOP_FREQ_DEFAULT   1.0f   /* 开环电角度旋转频率 */

#define MOTOR_SERVICE_CURRENT_LPF_ALPHA        0.05f  /* 电流低通滤波系数，范围 0 ~ 1 */
#define MOTOR_SERVICE_CURRENT_CAL_SAMPLES      100U   /* INA240 零点校准采样次数 */

#define MOTOR_SERVICE_ALIGN_VOLTAGE            1.0f   /* 传感器对齐电压 */
#define MOTOR_SERVICE_ALIGN_STEP_COUNT         500U   /* 传感器对齐步数 */
#define MOTOR_SERVICE_ALIGN_STEP_DELAY_MS      2U     /* 传感器对齐步 delay */
#define MOTOR_SERVICE_ALIGN_HOLD_DELAY_MS      400U   /* 传感器对齐保持 delay */
#define MOTOR_SERVICE_ALIGN_ANGLE              FOC_3PI_2 /* 传感器对齐目标角度，单位 rad，范围 0 ~ 2pi */

/* ============================================================
 * 模块内部变量
 * ============================================================ */

static Motor_Service_Data_t s_motor;    /* 电机服务数据结构体实例 */

static LowPassFilter_t s_lpf_ia;        /* A 相电流低通滤波器实例 */
static LowPassFilter_t s_lpf_ib;        /* B 相电流低通滤波器实例 */


/* ============================================================
 * 内部函数
 * ============================================================ */

/* 清除数据，恢复到默认状态 */
static void Motor_Service_ClearData(void)
{
    memset(&s_motor, 0, sizeof(s_motor));

    s_motor.mode = MOTOR_SERVICE_MODE_IDLE;
    s_motor.driver_enabled = 0U;
    s_motor.encoder_ok = 0U;

    s_motor.open_loop_uq = MOTOR_SERVICE_OPEN_LOOP_UQ_DEFAULT;
    s_motor.open_loop_frequency_hz = MOTOR_SERVICE_OPEN_LOOP_FREQ_DEFAULT;

    s_motor.direction = 1;
}

/* 更新传感器数据，读取编码器和电流传感器，计算角度和 Clarke/Park 变换 */
static Motor_Service_Status_t Motor_Service_UpdateSensorData(float dt_s)
{
    AS5048A_Status_t encoder_status;
    INA240_Status_t ina_status;

    encoder_status = AS5048A_Update(dt_s);

    if (encoder_status == AS5048A_OK)
    {
        s_motor.encoder_ok = 1U;

        s_motor.raw_angle = AS5048A_GetRawAngle();
        s_motor.mechanical_angle = AS5048A_GetAngleRad();
        s_motor.electrical_angle = FOC_GetElectricalAngle(s_motor.mechanical_angle);
    }
    else
    {
        s_motor.encoder_ok = 0U;

/*
* 编码器读取失败，开环模式可以继续，FOC 模式无法继续。
* 注意：如果编码器安装位置不正确或者电机未正确对齐，也会导致编码器数据异常，FOC 计算错误，此时也应该返回编码器错误。
*/
#if MOTOR_SERVICE_REQUIRE_ENCODER
        return MOTOR_SERVICE_ERROR_ENCODER;
#endif
    }

    ina_status = INA240_Update();

    if (ina_status != INA240_OK)
    {
        return MOTOR_SERVICE_ERROR_ADC;
    }

    s_motor.ia = INA240_GetCurrentA();
    s_motor.ib = INA240_GetCurrentB();
    s_motor.ic = INA240_GetCurrentC();

    /*
     * 推荐只滤波实际采样的 A/B 两相，
     * C 相由滤波后的 A/B 相计算，保证 ia + ib + ic = 0。
     */
    s_motor.ia_lpf = LowPassFilter_Update(&s_lpf_ia, s_motor.ia);
    s_motor.ib_lpf = LowPassFilter_Update(&s_lpf_ib, s_motor.ib);
    s_motor.ic_lpf = -(s_motor.ia_lpf + s_motor.ib_lpf);

    s_motor.current_alpha_beta =
        FOC_Clarke(s_motor.ia_lpf, s_motor.ib_lpf);

    s_motor.current_dq =
        FOC_Park(s_motor.current_alpha_beta, s_motor.electrical_angle);

    return MOTOR_SERVICE_OK;
}

/* 将 FOC 计算得到的占空比应用到 PWM 输出 */
static Motor_Service_Status_t Motor_Service_ApplyFOCOutput(void)
{
    s_motor.duty = FOC_GetDutyABC();
    s_motor.svpwm = FOC_GetSVPWMData();

    BSP_PWM_SetDutyABC(s_motor.duty.duty_a,
                       s_motor.duty.duty_b,
                       s_motor.duty.duty_c);

    return MOTOR_SERVICE_OK;
}

/* 根据电压幅值和电角度输出 PWM，占空比根据电压幅值计算，电角度根据频率积分 */
static Motor_Service_Status_t Motor_Service_OutputVoltageElectrical(float ud,
                                                                    float uq,
                                                                    float electrical_angle)
{
    FOC_Status_t foc_status;

    foc_status = FOC_SetVoltageDQElectrical(ud, uq, electrical_angle);

    if (foc_status != FOC_OK)
    {
        return MOTOR_SERVICE_ERROR_FOC;
    }

    return Motor_Service_ApplyFOCOutput();
}

/* ============================================================
 * 对外接口
 * ============================================================ */

/* 电机服务初始化，配置底层模块，校准传感器，初始化 FOC 参数 */
Motor_Service_Status_t Motor_Service_Init(void)
{
    AS5048A_Status_t encoder_status;

    Motor_Service_ClearData();  /* 清除数据，恢复到默认状态 */
    Platform_DelayInit();

    BSP_SPI_Init();
    BSP_ADC_Init();
    BSP_PWM_Init();

    /*
     * 先关闭驱动，防止初始化阶段误输出。
     */
    BSP_PWM_DisableDriver();
    s_motor.driver_enabled = 0U;

    /*
     * 启动 PWM。
     * 注意：PWM 启动不等于驱动使能，PB15 仍然是关闭状态。
     */
    if (BSP_PWM_Start() != 0)
    {
        return MOTOR_SERVICE_ERROR_PWM;
    }

    /*
     * 中性占空比。
     */
    BSP_PWM_SetDutyABC(0.5f, 0.5f, 0.5f);

    /*
     * 启动 ADC DMA。
     */
    if (BSP_ADC_Start() != 0)
    {
        return MOTOR_SERVICE_ERROR_ADC;
    }

    /*
     * 初始化 INA240，并在驱动关闭状态下校准零点。
     */
    INA240_Init();
    INA240_CalibrateOffset(MOTOR_SERVICE_CURRENT_CAL_SAMPLES);

    /*
     * 初始化低通滤波器。
     */
    LowPassFilter_Init(&s_lpf_ia, MOTOR_SERVICE_CURRENT_LPF_ALPHA);
    LowPassFilter_Init(&s_lpf_ib, MOTOR_SERVICE_CURRENT_LPF_ALPHA);

    /*
     * 初始化编码器。
     */
    encoder_status = AS5048A_Init();

    if (encoder_status == AS5048A_OK)
    {
        s_motor.encoder_ok = 1U;
    }
    else
    {
        s_motor.encoder_ok = 0U;

#if MOTOR_SERVICE_REQUIRE_ENCODER
        return MOTOR_SERVICE_ERROR_ENCODER;
#endif
    }

    /*
     * 初始化 FOC 算法模块。
     */
    FOC_Init();

    FOC_SetBusVoltage(MOTOR_SERVICE_BUS_VOLTAGE);       /* 设置母线电压 */
    FOC_SetPolePairs(MOTOR_SERVICE_POLE_PAIRS);         /* 设置极对数 */
    FOC_SetDirection(1);                                /* 设置默认方向，1 或 -1，正向或反向 */
    FOC_SetZeroElectricAngle(0.0f);                     /* 设置默认电角度零位，单位 rad，范围 0 ~ 2pi */

    s_motor.zero_electric_angle = FOC_GetZeroElectricAngle();  /* 读取电角度零位，保存到数据结构 */
    s_motor.direction = FOC_GetDirection();                    /* 读取方向，保存到数据结构 */

    return MOTOR_SERVICE_OK;
}

/* 电机服务更新，读取传感器数据，执行控制算法，输出 PWM */
Motor_Service_Status_t Motor_Service_Update(float dt_s)
{
    Motor_Service_Status_t status;

    if (dt_s <= 0.0f)
    {
        return MOTOR_SERVICE_ERROR_PARAM;
    }

    status = Motor_Service_UpdateSensorData(dt_s);

    if (status != MOTOR_SERVICE_OK)
    {
        return status;
    }

    if (s_motor.mode == MOTOR_SERVICE_MODE_OPEN_LOOP_ELECTRICAL)
    {
        s_motor.open_loop_electrical_angle +=
            FOC_2PI * s_motor.open_loop_frequency_hz * dt_s;

        s_motor.open_loop_electrical_angle =
            FOC_NormalizeAngle(s_motor.open_loop_electrical_angle);

        status = Motor_Service_OutputVoltageElectrical(0.0f,
                                                       s_motor.open_loop_uq,
                                                       s_motor.open_loop_electrical_angle);

        if (status != MOTOR_SERVICE_OK)
        {
            return status;
        }
    }
    else
    {
        /*
         * IDLE 模式下保持中性占空比。
         */
        BSP_PWM_SetDutyABC(0.5f, 0.5f, 0.5f);

        s_motor.duty.duty_a = 0.5f;
        s_motor.duty.duty_b = 0.5f;
        s_motor.duty.duty_c = 0.5f;
    }

    return MOTOR_SERVICE_OK;
}

/* 开环电角度模式，根据设定的电压幅值和频率输出 PWM，占空比根据电压幅值计算，电角度根据频率积分 */
void Motor_Service_StartOpenLoopElectrical(float uq, float frequency_hz)
{
    s_motor.open_loop_uq = uq;
    s_motor.open_loop_frequency_hz = frequency_hz;
    s_motor.open_loop_electrical_angle = 0.0f;

    s_motor.mode = MOTOR_SERVICE_MODE_OPEN_LOOP_ELECTRICAL;
}

/* 停止电机，进入空闲模式，输出中性占空比，关闭驱动 */
void Motor_Service_Stop(void)
{
    s_motor.mode = MOTOR_SERVICE_MODE_IDLE;

    BSP_PWM_SetDutyABC(0.5f, 0.5f, 0.5f);
    Motor_Service_DisableDriver();
}

/**
 * @brief 使能驱动，允许输出 PWM 信号，电机可以转动
 */
void Motor_Service_EnableDriver(void)
{
    BSP_PWM_EnableDriver();
    s_motor.driver_enabled = 1U;
}

/**
 * @brief 禁止驱动，PWM 输出占空比被强制为 0，电机无法转动
 *
 * 注意：调用该函数后 PWM 信号仍然在输出，但占空比为 0，相当于刹车状态。
 * 如果需要完全停止 PWM 输出，可以调用 BSP_PWM_Stop()，但通常不建议频繁启停 PWM。
 */
void Motor_Service_DisableDriver(void)
{
    BSP_PWM_DisableDriver();
    s_motor.driver_enabled = 0U;
}

/* 设置开环电压幅值 */
void Motor_Service_SetOpenLoopVoltage(float uq)
{
    s_motor.open_loop_uq = uq;
}

/* 设置开环电角度旋转频率 */
void Motor_Service_SetOpenLoopFrequency(float frequency_hz)
{
    s_motor.open_loop_frequency_hz = frequency_hz;
}

/**
 * 编码器电角度零位对齐
 */
Motor_Service_Status_t Motor_Service_AlignSensor(void)
{
    uint16_t i;

    float angle_mid;
    float angle_end;
    float move;

    float mechanical_angle;
    float zero_offset;

    Motor_Service_Status_t status;

    /*
     * 对齐前先进入空闲模式。
     */
    s_motor.mode = MOTOR_SERVICE_MODE_IDLE;

    /*
     * 使能驱动，开始对齐。
     */
    Motor_Service_EnableDriver();

    /*
     * 正向扫一圈电角度。
     */
    for (i = 0U; i < MOTOR_SERVICE_ALIGN_STEP_COUNT; i++)
    {
        float angle;

        angle =
            MOTOR_SERVICE_ALIGN_ANGLE
            + FOC_2PI * ((float)i / (float)MOTOR_SERVICE_ALIGN_STEP_COUNT);

        status = Motor_Service_OutputVoltageElectrical(MOTOR_SERVICE_ALIGN_VOLTAGE,
                                                       0.0f,
                                                       angle);

        if (status != MOTOR_SERVICE_OK)
        {
            Motor_Service_Stop();
            return status;
        }

        Platform_DelayMs(MOTOR_SERVICE_ALIGN_STEP_DELAY_MS);
    }

    (void)AS5048A_Update(0.002f);                   /* 刷新编码器数据，读取电角度中点位置 */
    angle_mid = AS5048A_GetContinuousRad();         /* 记录电角度中点位置，后面用来判断机械运动和方向 */

    /*
     * 反向扫回。
     */
    for (i = MOTOR_SERVICE_ALIGN_STEP_COUNT; i > 0U; i--)
    {
        float angle;

        angle =
            MOTOR_SERVICE_ALIGN_ANGLE
            + FOC_2PI * ((float)i / (float)MOTOR_SERVICE_ALIGN_STEP_COUNT);

        status = Motor_Service_OutputVoltageElectrical(MOTOR_SERVICE_ALIGN_VOLTAGE,
                                                       0.0f,
                                                       angle);

        if (status != MOTOR_SERVICE_OK)
        {
            Motor_Service_Stop();
            return status;
        }

        Platform_DelayMs(MOTOR_SERVICE_ALIGN_STEP_DELAY_MS);
    }

    (void)AS5048A_Update(0.002f);                   /* 刷新编码器数据，读取电角度终点位置 */
    angle_end = AS5048A_GetContinuousRad();         /* 记录电角度终点位置，和中点位置一起判断机械运动和方向 */

    /*
     * 判断是否真的发生机械运动。
     */
    move = fabsf(angle_mid - angle_end);

    if (move < (FOC_2PI / 101.0f))
    {
        Motor_Service_Stop();
        return MOTOR_SERVICE_ERROR_ALIGN;
    }

    /*
     * 判断方向。
     *
     * 如果后面发现方向反了，可以直接把这里的 1/-1 对调。
     */
    if (angle_mid < angle_end)
    {
        FOC_SetDirection(-1);
    }
    else
    {
        FOC_SetDirection(1);
    }

    s_motor.direction = FOC_GetDirection();               /* 读取方向，保存到数据结构 */

    /*
     * 把转子吸到一个已知电角度 MOTOR_SERVICE_ALIGN_ANGLE。
     */
    status = Motor_Service_OutputVoltageElectrical(MOTOR_SERVICE_ALIGN_VOLTAGE,
                                                   0.0f,
                                                   MOTOR_SERVICE_ALIGN_ANGLE);

    if (status != MOTOR_SERVICE_OK)
    {
        Motor_Service_Stop();
        return status;
    }

    Platform_DelayMs(MOTOR_SERVICE_ALIGN_HOLD_DELAY_MS);

    /*
     * 读取当前位置，计算零电角度。
     *
     * FOC_GetElectricalAngle() 内部使用：
     * theta_e = direction * pole_pairs * theta_m - zero_offset
     *
     * 希望当前 theta_e = MOTOR_SERVICE_ALIGN_ANGLE
     *
     * 所以：
     * zero_offset = direction * pole_pairs * theta_m - MOTOR_SERVICE_ALIGN_ANGLE
     */
    (void)AS5048A_Update(0.002f);

    mechanical_angle = AS5048A_GetAngleRad();

    zero_offset =
        (float)FOC_GetDirection()
        * (float)FOC_GetPolePairs()
        * mechanical_angle
        - MOTOR_SERVICE_ALIGN_ANGLE;

    FOC_SetZeroElectricAngle(zero_offset);

    s_motor.zero_electric_angle = FOC_GetZeroElectricAngle();

    /*
     * 对齐完成后关闭输出。
     */
    Motor_Service_Stop();

    return MOTOR_SERVICE_OK;
}


/* 获取当前模式 */
Motor_Service_Mode_t Motor_Service_GetMode(void)
{
    return s_motor.mode;
}


/* 获取电机服务数据结构体指针，供外部读取数据 */
const Motor_Service_Data_t *Motor_Service_GetData(void)
{
    return &s_motor;
}
