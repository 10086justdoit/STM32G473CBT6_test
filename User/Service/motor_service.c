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
#include "pid_controller.h"

#include <math.h>
#include <string.h>

/* ============================================================
 * Motor Service 基础配置区
 * ============================================================ */

/*
 * 是否要求编码器初始化必须成功。
 *
 * 0：编码器失败也允许开环 PWM 测试
 * 1：编码器失败则 Motor_Service_Init() 返回错误
 */
#define MOTOR_SERVICE_REQUIRE_ENCODER              0U

#define MOTOR_SERVICE_BUS_VOLTAGE                  12.0f      /* 母线电压，单位 V */
#define MOTOR_SERVICE_POLE_PAIRS                   11U        /* 电机极对数 */

#define MOTOR_SERVICE_ZERO_ELECTRIC_ANGLE          4.588000f  /* 电角度零位，来自对齐测试 */
#define MOTOR_SERVICE_DIRECTION                    -1          /* 默认旋转方向，1 = 正向，-1 = 反向 */

#define MOTOR_SERVICE_OPEN_LOOP_UQ_DEFAULT         0.5f       /* 开环默认 Uq 电压，单位 V */
#define MOTOR_SERVICE_OPEN_LOOP_FREQ_DEFAULT       1.0f       /* 开环默认电角度频率，单位 Hz */

#define MOTOR_SERVICE_CURRENT_LPF_ALPHA            0.05f      /* 电流低通滤波系数 */
#define MOTOR_SERVICE_SPEED_LPF_ALPHA              0.02f      /* 速度低通滤波系数 */
#define MOTOR_SERVICE_CURRENT_CAL_SAMPLES          100U       /* INA240 零点校准采样次数 */

#define MOTOR_SERVICE_RAD_TO_RPM                   9.5492966f /* rad/s 转 rpm 系数 */

/* ============================================================
 * 电流采样相序配置
 *
 * 0：不交换，ia = rawA，ib = rawB
 * 1：交换 A/B，ia = rawB，ib = rawA
 * 2：交换 A/B，并反向 ib，ia = rawB，ib = -rawA
 * 3：交换 A/B，并反向 ia，ia = -rawB，ib = rawA
 *
 * 当前电流环已经验证通过，保持 0U。
 * ============================================================ */

#define MOTOR_SERVICE_CURRENT_MAP_MODE             0U

/* ============================================================
 * 电流环 PID 配置区
 *
 * 当前调试周期为 1ms。
 * 如果后续改成 50us ~ 100us 定时器中断，需要重新整定 PID 参数。
 * ============================================================ */

#define MOTOR_SERVICE_CURRENT_LOOP_DT_S            0.001f     /* 电流环周期，单位 s */

#define MOTOR_SERVICE_CURRENT_LOOP_V_LIMIT         1.2f       /* 电流环输出电压限制，单位 V */

#define MOTOR_SERVICE_PID_ID_KP                    2.0f       /* D 轴电流环 P 参数 */
#define MOTOR_SERVICE_PID_ID_KI                    15.0f      /* D 轴电流环 I 参数 */
#define MOTOR_SERVICE_PID_ID_KD                    0.0f       /* D 轴电流环 D 参数 */

#define MOTOR_SERVICE_PID_IQ_KP                    3.0f       /* Q 轴电流环 P 参数 */
#define MOTOR_SERVICE_PID_IQ_KI                    35.0f      /* Q 轴电流环 I 参数 */
#define MOTOR_SERVICE_PID_IQ_KD                    0.0f       /* Q 轴电流环 D 参数 */

#define MOTOR_SERVICE_PID_INTEGRAL_LIMIT           1.0f       /* PID 积分项限制，单位 V */
/* ============================================================
 * 速度环 PID 配置区
 *
 * 速度环只输出 iq_ref。
 * 电流环每 1ms 执行一次。
 * 速度环每 10ms 执行一次。
 * ============================================================ */

#define MOTOR_SERVICE_SPEED_EST_DT_S               0.100f     /* 速度估算周期，单位 s */
#define MOTOR_SERVICE_SPEED_EST_MAX_RPM            80.0f      /* 速度估算异常限幅，单位 rpm */

#define MOTOR_SERVICE_SPEED_LOOP_DT_S              0.10f     /* 速度环周期，单位 s */
#define MOTOR_SERVICE_SPEED_LOOP_DIVIDER           10U        /* 电流环 1ms，速度环 10ms */

#define MOTOR_SERVICE_SPEED_FEEDBACK_DIRECTION     -1.0f      /* 速度反馈方向 */

#define MOTOR_SERVICE_SPEED_LOOP_IQ_LIMIT          0.25f      /* 速度环输出 iq_ref 限幅，单位 A */

#define MOTOR_SERVICE_PID_SPEED_KP                 0.003f     /* 速度环 P 参数，单位 A/rpm */
#define MOTOR_SERVICE_PID_SPEED_KI                 0.004f     /* 速度环 I 参数 */
#define MOTOR_SERVICE_PID_SPEED_KD                 0.0f       /* 速度环 D 参数 */

#define MOTOR_SERVICE_PID_SPEED_INTEGRAL_LIMIT     0.10f      /* 速度环积分项限制，单位 A */

#define MOTOR_SERVICE_SPEED_START_IQ_MIN           0.14f      /* 最小启动 iq_ref，单位 A */
#define MOTOR_SERVICE_SPEED_START_EXIT_RATIO       0.70f      /* 达到目标速度 80% 后退出启动补偿 */

#define MOTOR_SERVICE_SPEED_RUN_IQ_MIN             0.06f      /* 低速保持 iq_ref，单位 A */

#define MOTOR_SERVICE_SPEED_IQ_SLEW_STEP           0.02f     /* iq_ref 每次最大变化量，单位 A */
/* ============================================================
 * 编码器对齐配置区
 * ============================================================ */

#define MOTOR_SERVICE_ALIGN_VOLTAGE                1.0f       /* 对齐电压，单位 V */
#define MOTOR_SERVICE_ALIGN_STEP_COUNT             500U       /* 对齐扫描步数 */
#define MOTOR_SERVICE_ALIGN_STEP_DELAY_MS          2U         /* 每步延时，单位 ms */
#define MOTOR_SERVICE_ALIGN_HOLD_DELAY_MS          400U       /* 对齐保持时间，单位 ms */
#define MOTOR_SERVICE_ALIGN_ANGLE                  FOC_3PI_2  /* 对齐目标电角度 */

/* ============================================================
 * 模块内部变量
 * ============================================================ */

static Motor_Service_Data_t s_motor;        /* 电机服务数据实例 */

static LowPassFilter_t s_lpf_ia;            /* A 相电流低通滤波器 */
static LowPassFilter_t s_lpf_ib;            /* B 相电流低通滤波器 */
static LowPassFilter_t s_lpf_speed;         /* 机械速度低通滤波器 */

static PID_Controller_t s_pid_id;           /* D 轴电流环 PID */
static PID_Controller_t s_pid_iq;           /* Q 轴电流环 PID */
static PID_Controller_t s_pid_speed;        /* 速度环 PID */

static uint8_t s_speed_ready = 0U;           /* 速度计算初始化标志 */
static float s_speed_last_angle = 0.0f;      /* 上一次机械角度，单位 rad */
static float s_speed_accum_angle = 0.0f;     /* 速度估算累计角度，单位 rad */
static float s_speed_accum_time = 0.0f;      /* 速度估算累计时间，单位 s */
static uint16_t s_speed_loop_count = 0U;     /* 速度环分频计数器 */

static float s_speed_iq_ref_cmd = 0.0f;     /* 速度环输出 iq_ref 斜坡缓冲 */
/* ============================================================
 * 内部函数声明
 * ============================================================ */

static void Motor_Service_ClearData(void);

static Motor_Service_Status_t Motor_Service_UpdateSensorData(float dt_s);
static void Motor_Service_UpdateSpeed(float dt_s);

static Motor_Service_Status_t Motor_Service_ApplyFOCOutput(void);
static Motor_Service_Status_t Motor_Service_OutputVoltageElectrical(float ud,
                                                                    float uq,
                                                                    float electrical_angle);

static Motor_Service_Status_t Motor_Service_RunIdle(void);
static Motor_Service_Status_t Motor_Service_RunOpenLoop(float dt_s);
static Motor_Service_Status_t Motor_Service_RunCurrentLoop(float dt_s);
static Motor_Service_Status_t Motor_Service_RunSpeedLoop(float dt_s);

/* ============================================================
 * 内部函数实现
 * ============================================================ */

/**
 * @brief 清空电机服务数据，并恢复默认状态
 */
static void Motor_Service_ClearData(void)
{
    memset(&s_motor, 0, sizeof(s_motor));

    s_motor.mode = MOTOR_SERVICE_MODE_IDLE;

    s_motor.driver_enabled = 0U;
    s_motor.encoder_ok = 0U;

    s_motor.open_loop_electrical_angle = 0.0f;
    s_motor.open_loop_uq = MOTOR_SERVICE_OPEN_LOOP_UQ_DEFAULT;
    s_motor.open_loop_frequency_hz = MOTOR_SERVICE_OPEN_LOOP_FREQ_DEFAULT;

    s_motor.raw_angle = 0U;
    s_motor.mechanical_angle = 0.0f;
    s_motor.electrical_angle = 0.0f;

    s_motor.speed_raw_rpm = 0.0f;
    s_motor.speed_rpm = 0.0f;
    s_motor.speed_ref_rpm = 0.0f;

    s_motor.ia = 0.0f;
    s_motor.ib = 0.0f;
    s_motor.ic = 0.0f;

    s_motor.ia_lpf = 0.0f;
    s_motor.ib_lpf = 0.0f;
    s_motor.ic_lpf = 0.0f;

    s_motor.id_ref = 0.0f;
    s_motor.iq_ref = 0.0f;

    s_motor.id_meas = 0.0f;
    s_motor.iq_meas = 0.0f;

    s_motor.vd = 0.0f;
    s_motor.vq = 0.0f;

    s_motor.pid_id_error = 0.0f;
    s_motor.pid_iq_error = 0.0f;
    s_motor.pid_speed_error = 0.0f;

    s_motor.pid_id_output = 0.0f;
    s_motor.pid_iq_output = 0.0f;
    s_motor.pid_speed_output = 0.0f;

    s_motor.duty.duty_a = 0.5f;
    s_motor.duty.duty_b = 0.5f;
    s_motor.duty.duty_c = 0.5f;

    s_motor.zero_electric_angle = MOTOR_SERVICE_ZERO_ELECTRIC_ANGLE;
    s_motor.direction = MOTOR_SERVICE_DIRECTION;

    s_speed_ready = 0U;
    s_speed_last_angle = 0.0f;
    s_speed_accum_angle = 0.0f;
    s_speed_accum_time = 0.0f;
    s_speed_loop_count = 0U;
    s_speed_iq_ref_cmd = 0.0f;
}
/**
 * @brief 更新机械速度
 *
 * 功能：
 * 1. 根据本次机械角度和上次机械角度计算角度增量
 * 2. 处理 0 ~ 2pi 跳变
 * 3. 累计一段时间后再计算速度，降低低速抖动
 * 4. 将机械角速度转换成 rpm
 * 5. 对速度进行低通滤波
 *
 * @param dt_s 距离上次调用的时间，单位 s
 */
static void Motor_Service_UpdateSpeed(float dt_s)
{
    float delta_angle;
    float speed_rad_s;
    float speed_rpm;

    if (dt_s <= 0.0f)
    {
        return;
    }

    if (s_speed_ready == 0U)
    {
        s_speed_last_angle = s_motor.mechanical_angle;
        s_speed_accum_angle = 0.0f;
        s_speed_accum_time = 0.0f;
        s_speed_ready = 1U;

        s_motor.speed_raw_rpm = 0.0f;
        s_motor.speed_rpm = 0.0f;

        return;
    }

    delta_angle = s_motor.mechanical_angle - s_speed_last_angle;

    if (delta_angle > 3.1415926f)
    {
        delta_angle -= FOC_2PI;
    }
    else if (delta_angle < -3.1415926f)
    {
        delta_angle += FOC_2PI;
    }

    s_speed_last_angle = s_motor.mechanical_angle;

    s_speed_accum_angle += delta_angle;
    s_speed_accum_time += dt_s;

    /*
     * 低速时不要每 1ms 计算一次速度。
     * 这里累计 10ms 后再计算一次，可以明显减少速度尖峰。
     */
    if (s_speed_accum_time >= MOTOR_SERVICE_SPEED_EST_DT_S)
    {
        speed_rad_s = s_speed_accum_angle / s_speed_accum_time;

        speed_rpm =
            speed_rad_s
            * MOTOR_SERVICE_RAD_TO_RPM
            * MOTOR_SERVICE_SPEED_FEEDBACK_DIRECTION;

        /*
         * 异常速度限幅。
         * 防止编码器瞬间跳变导致速度反馈出现巨大尖峰。
         */
        speed_rpm = FOC_LimitFloat(speed_rpm,
                                   -MOTOR_SERVICE_SPEED_EST_MAX_RPM,
                                   MOTOR_SERVICE_SPEED_EST_MAX_RPM);

        s_motor.speed_raw_rpm = speed_rpm;

        s_motor.speed_rpm =
            LowPassFilter_Update(&s_lpf_speed, s_motor.speed_raw_rpm);

        s_speed_accum_angle = 0.0f;
        s_speed_accum_time = 0.0f;
    }
}

/**
 * @brief 更新传感器数据
 *
 * 功能：
 * 1. 读取 AS5048A 角度
 * 2. 读取 INA240 电流
 * 3. 根据当前电流映射模式生成 ia / ib / ic
 * 4. 执行 Clarke / Park 变换，得到 id / iq
 *
 * @param dt_s 距离上次调用的时间，单位 s
 * @return Motor_Service_Status_t
 */
static Motor_Service_Status_t Motor_Service_UpdateSensorData(float dt_s)
{
    AS5048A_Status_t encoder_status;
    INA240_Status_t ina_status;

    float raw_current_a;
    float raw_current_b;

    encoder_status = AS5048A_Update(dt_s);

    if (encoder_status == AS5048A_OK)
    {
        s_motor.encoder_ok = 1U;

        s_motor.raw_angle = AS5048A_GetRawAngle();
        s_motor.mechanical_angle = AS5048A_GetAngleRad();
        s_motor.electrical_angle = FOC_GetElectricalAngle(s_motor.mechanical_angle);

        Motor_Service_UpdateSpeed(dt_s);
    }
    else
    {
        s_motor.encoder_ok = 0U;

#if MOTOR_SERVICE_REQUIRE_ENCODER
        return MOTOR_SERVICE_ERROR_ENCODER;
#endif
    }

    ina_status = INA240_Update();

    if (ina_status != INA240_OK)
    {
        return MOTOR_SERVICE_ERROR_ADC;
    }

    /*
     * 读取 INA240 原始电流。
     *
     * raw_current_a：硬件 INA240 A 通道
     * raw_current_b：硬件 INA240 B 通道
     */
    raw_current_a = INA240_GetCurrentA();
    raw_current_b = INA240_GetCurrentB();

#if (MOTOR_SERVICE_CURRENT_MAP_MODE == 0U)

    /*
     * 原始映射：
     * 软件 A 相 = 硬件 A 通道
     * 软件 B 相 = 硬件 B 通道
     */
    s_motor.ia = raw_current_a;
    s_motor.ib = raw_current_b;

#elif (MOTOR_SERVICE_CURRENT_MAP_MODE == 1U)

    /*
     * 交换 A/B：
     * 软件 A 相 = 硬件 B 通道
     * 软件 B 相 = 硬件 A 通道
     */
    s_motor.ia = raw_current_b;
    s_motor.ib = raw_current_a;

#elif (MOTOR_SERVICE_CURRENT_MAP_MODE == 2U)

    /*
     * 交换 A/B，并反向 B 相：
     * 软件 A 相 = 硬件 B 通道
     * 软件 B 相 = -硬件 A 通道
     */
    s_motor.ia = raw_current_b;
    s_motor.ib = -raw_current_a;

#elif (MOTOR_SERVICE_CURRENT_MAP_MODE == 3U)

    /*
     * 交换 A/B，并反向 A 相：
     * 软件 A 相 = -硬件 B 通道
     * 软件 B 相 = 硬件 A 通道
     */
    s_motor.ia = -raw_current_b;
    s_motor.ib = raw_current_a;

#else

    /*
     * 默认使用原始映射。
     */
    s_motor.ia = raw_current_a;
    s_motor.ib = raw_current_b;

#endif

    /*
     * 两电阻采样：
     * C 相电流由 ia + ib + ic = 0 推算得到。
     */
    s_motor.ic = -(s_motor.ia + s_motor.ib);

    /*
     * 对 A/B 相电流做低通滤波。
     * C 相由滤波后的 A/B 相推算，保证三相电流和为 0。
     */
    s_motor.ia_lpf = LowPassFilter_Update(&s_lpf_ia, s_motor.ia);
    s_motor.ib_lpf = LowPassFilter_Update(&s_lpf_ib, s_motor.ib);
    s_motor.ic_lpf = -(s_motor.ia_lpf + s_motor.ib_lpf);

    /*
     * Clarke 变换：abc -> alpha/beta。
     */
    s_motor.current_alpha_beta =
        FOC_Clarke(s_motor.ia_lpf, s_motor.ib_lpf);

    /*
     * Park 变换：alpha/beta -> d/q。
     */
    s_motor.current_dq =
        FOC_Park(s_motor.current_alpha_beta, s_motor.electrical_angle);

    s_motor.id_meas = s_motor.current_dq.d;
    s_motor.iq_meas = s_motor.current_dq.q;

    return MOTOR_SERVICE_OK;
}

/**
 * @brief 将 FOC 计算得到的三相占空比输出到 PWM
 */
static Motor_Service_Status_t Motor_Service_ApplyFOCOutput(void)
{
    s_motor.duty = FOC_GetDutyABC();
    s_motor.svpwm = FOC_GetSVPWMData();

    BSP_PWM_SetDutyABC(s_motor.duty.duty_a,
                       s_motor.duty.duty_b,
                       s_motor.duty.duty_c);

    return MOTOR_SERVICE_OK;
}

/**
 * @brief 根据 D/Q 轴电压和指定电角度输出 PWM
 *
 * @param ud D 轴电压
 * @param uq Q 轴电压
 * @param electrical_angle 电角度
 * @return Motor_Service_Status_t
 */
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

/**
 * @brief 空闲模式处理
 */
static Motor_Service_Status_t Motor_Service_RunIdle(void)
{
    BSP_PWM_SetDutyABC(0.5f, 0.5f, 0.5f);

    s_motor.duty.duty_a = 0.5f;
    s_motor.duty.duty_b = 0.5f;
    s_motor.duty.duty_c = 0.5f;

    s_motor.id_ref = 0.0f;
    s_motor.iq_ref = 0.0f;

    s_motor.vd = 0.0f;
    s_motor.vq = 0.0f;

    s_motor.pid_id_output = 0.0f;
    s_motor.pid_iq_output = 0.0f;
    s_motor.pid_speed_output = 0.0f;

    return MOTOR_SERVICE_OK;
}

/**
 * @brief 开环电角度模式
 *
 * 根据设定的 Uq 和电角度频率，持续旋转电压矢量。
 */
static Motor_Service_Status_t Motor_Service_RunOpenLoop(float dt_s)
{
    Motor_Service_Status_t status;

    s_motor.open_loop_electrical_angle +=
        FOC_2PI * s_motor.open_loop_frequency_hz * dt_s;

    s_motor.open_loop_electrical_angle =
        FOC_NormalizeAngle(s_motor.open_loop_electrical_angle);

    status = Motor_Service_OutputVoltageElectrical(0.0f,
                                                   s_motor.open_loop_uq,
                                                   s_motor.open_loop_electrical_angle);

    return status;
}

/**
 * @brief 电流环模式
 *
 * 根据 id_ref / iq_ref 计算 vd / vq，
 * 再通过 FOC 和 SVPWM 输出三相 PWM。
 */
static Motor_Service_Status_t Motor_Service_RunCurrentLoop(float dt_s)
{
    FOC_Status_t foc_status;

    /*
     * D 轴电流环：
     * 通常 id_ref = 0。
     */
    s_motor.vd = PID_Controller_Update(&s_pid_id,
                                       s_motor.id_ref,
                                       s_motor.id_meas,
                                       dt_s);

    /*
     * Q 轴电流环：
     * iq_ref 决定电磁转矩。
     */
    s_motor.vq = PID_Controller_Update(&s_pid_iq,
                                       s_motor.iq_ref,
                                       s_motor.iq_meas,
                                       dt_s);

    /*
     * 电压限幅：
     * 防止电流环输出过大导致驱动器进入保护。
     */
    s_motor.vd = FOC_LimitFloat(s_motor.vd,
                                -MOTOR_SERVICE_CURRENT_LOOP_V_LIMIT,
                                MOTOR_SERVICE_CURRENT_LOOP_V_LIMIT);

    s_motor.vq = FOC_LimitFloat(s_motor.vq,
                                -MOTOR_SERVICE_CURRENT_LOOP_V_LIMIT,
                                MOTOR_SERVICE_CURRENT_LOOP_V_LIMIT);

    s_motor.pid_id_error = PID_Controller_GetError(&s_pid_id);
    s_motor.pid_iq_error = PID_Controller_GetError(&s_pid_iq);

    s_motor.pid_id_output = PID_Controller_GetOutput(&s_pid_id);
    s_motor.pid_iq_output = PID_Controller_GetOutput(&s_pid_iq);

    /*
     * 保持你原来已经跑通的电流环路径：
     * 输入机械角度，由 FOC 内部根据极对数、方向和零位计算电角度。
     */
    foc_status = FOC_SetVoltageDQ(s_motor.vd,
                                  s_motor.vq,
                                  s_motor.mechanical_angle);

    if (foc_status != FOC_OK)
    {
        return MOTOR_SERVICE_ERROR_FOC;
    }

    return Motor_Service_ApplyFOCOutput();
}
/**
 * @brief 速度环模式
 *
 * 速度环每 10ms 执行一次。
 * 速度环输出 iq_ref。
 * 电流环每 1ms 执行一次，继续负责 id / iq 闭环。
 */
static Motor_Service_Status_t Motor_Service_RunSpeedLoop(float dt_s)
{
    float speed_ref_abs;
    float speed_abs;
    float speed_output_target;
    float speed_output_sign;

    if (s_motor.encoder_ok == 0U)
    {
        return MOTOR_SERVICE_ERROR_ENCODER;
    }

    s_speed_loop_count++;

    if (s_speed_loop_count >= MOTOR_SERVICE_SPEED_LOOP_DIVIDER)
    {
        s_speed_loop_count = 0U;

        speed_output_target =
            PID_Controller_Update(&s_pid_speed,
                                  s_motor.speed_ref_rpm,
                                  s_motor.speed_rpm,
                                  MOTOR_SERVICE_SPEED_LOOP_DT_S);

        s_motor.pid_speed_error = PID_Controller_GetError(&s_pid_speed);

        speed_ref_abs = fabsf(s_motor.speed_ref_rpm);
        speed_abs = fabsf(s_motor.speed_rpm);

        if (s_motor.speed_ref_rpm > 0.5f)
        {
            speed_output_sign = 1.0f;
        }
        else if (s_motor.speed_ref_rpm < -0.5f)
        {
            speed_output_sign = -1.0f;
        }
        else
        {
            speed_output_sign = 0.0f;
        }

        /*
         * 目标速度为正时，不允许输出负 iq_ref。
         * 目标速度为负时，不允许输出正 iq_ref。
         */
        if (speed_output_sign > 0.0f)
        {
            if (speed_output_target < 0.0f)
            {
                speed_output_target = 0.0f;
            }
        }
        else if (speed_output_sign < 0.0f)
        {
            if (speed_output_target > 0.0f)
            {
                speed_output_target = 0.0f;
            }
        }
        else
        {
            speed_output_target = 0.0f;
        }

        /*
         * 低速启动补偿。
         *
         * 只有实际速度低于目标速度 80% 时，给启动电流。
         */
        if ((speed_ref_abs > 1.0f) &&
            (speed_abs < (speed_ref_abs * MOTOR_SERVICE_SPEED_START_EXIT_RATIO)))
        {
            if (fabsf(speed_output_target) < MOTOR_SERVICE_SPEED_START_IQ_MIN)
            {
                speed_output_target =
                    speed_output_sign * MOTOR_SERVICE_SPEED_START_IQ_MIN;
            }
        }

        /*
         * 低速保持电流。
         *
         * 防止速度反馈短暂变大后，iq_ref 直接掉到 0，
         * 造成“转几圈停一下”的现象。
         */
        if ((speed_ref_abs > 1.0f) &&
            (speed_output_sign > 0.0f))
        {
            if ((speed_output_target > 0.0f) &&
                (speed_output_target < MOTOR_SERVICE_SPEED_RUN_IQ_MIN))
            {
                speed_output_target = MOTOR_SERVICE_SPEED_RUN_IQ_MIN;
            }
        }
        else if ((speed_ref_abs > 1.0f) &&
                 (speed_output_sign < 0.0f))
        {
            if ((speed_output_target < 0.0f) &&
                (speed_output_target > -MOTOR_SERVICE_SPEED_RUN_IQ_MIN))
            {
                speed_output_target = -MOTOR_SERVICE_SPEED_RUN_IQ_MIN;
            }
        }

        speed_output_target =
            FOC_LimitFloat(speed_output_target,
                           -MOTOR_SERVICE_SPEED_LOOP_IQ_LIMIT,
                           MOTOR_SERVICE_SPEED_LOOP_IQ_LIMIT);

        /*
         * iq_ref 斜坡限制。
         */
        if (speed_output_target > (s_speed_iq_ref_cmd + MOTOR_SERVICE_SPEED_IQ_SLEW_STEP))
        {
            s_speed_iq_ref_cmd += MOTOR_SERVICE_SPEED_IQ_SLEW_STEP;
        }
        else if (speed_output_target < (s_speed_iq_ref_cmd - MOTOR_SERVICE_SPEED_IQ_SLEW_STEP))
        {
            s_speed_iq_ref_cmd -= MOTOR_SERVICE_SPEED_IQ_SLEW_STEP;
        }
        else
        {
            s_speed_iq_ref_cmd = speed_output_target;
        }

        s_speed_iq_ref_cmd =
            FOC_LimitFloat(s_speed_iq_ref_cmd,
                           -MOTOR_SERVICE_SPEED_LOOP_IQ_LIMIT,
                           MOTOR_SERVICE_SPEED_LOOP_IQ_LIMIT);

        /*
         * 二次方向限制。
         */
        if (speed_output_sign > 0.0f)
        {
            if (s_speed_iq_ref_cmd < 0.0f)
            {
                s_speed_iq_ref_cmd = 0.0f;
            }
        }
        else if (speed_output_sign < 0.0f)
        {
            if (s_speed_iq_ref_cmd > 0.0f)
            {
                s_speed_iq_ref_cmd = 0.0f;
            }
        }
        else
        {
            s_speed_iq_ref_cmd = 0.0f;
        }

        s_motor.pid_speed_output = s_speed_iq_ref_cmd;

        s_motor.id_ref = 0.0f;
        s_motor.iq_ref = s_motor.pid_speed_output;
    }

    return Motor_Service_RunCurrentLoop(dt_s);
}
/* ============================================================
 * 对外接口实现
 * ============================================================ */

/**
 * @brief 初始化电机服务
 */
Motor_Service_Status_t Motor_Service_Init(void)
{
    AS5048A_Status_t encoder_status;

    Motor_Service_ClearData();
    Platform_DelayInit();

    BSP_SPI_Init();
    BSP_ADC_Init();
    BSP_PWM_Init();

    /*
     * 初始化阶段先关闭驱动，防止误动作。
     */
    BSP_PWM_DisableDriver();
    s_motor.driver_enabled = 0U;

    /*
     * 启动 PWM。
     * 注意：PWM 启动不等于驱动芯片使能。
     */
    if (BSP_PWM_Start() != 0)
    {
        return MOTOR_SERVICE_ERROR_PWM;
    }

    /*
     * 设置中性占空比。
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
     * 初始化电流低通滤波器。
     */
    LowPassFilter_Init(&s_lpf_ia, MOTOR_SERVICE_CURRENT_LPF_ALPHA);
    LowPassFilter_Init(&s_lpf_ib, MOTOR_SERVICE_CURRENT_LPF_ALPHA);

    /*
     * 初始化速度低通滤波器。
     */
    LowPassFilter_Init(&s_lpf_speed, MOTOR_SERVICE_SPEED_LPF_ALPHA);

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

    FOC_SetBusVoltage(MOTOR_SERVICE_BUS_VOLTAGE);
    FOC_SetPolePairs(MOTOR_SERVICE_POLE_PAIRS);
    FOC_SetDirection(MOTOR_SERVICE_DIRECTION);
    FOC_SetZeroElectricAngle(MOTOR_SERVICE_ZERO_ELECTRIC_ANGLE);

    s_motor.zero_electric_angle = FOC_GetZeroElectricAngle();
    s_motor.direction = FOC_GetDirection();

    /*
     * 初始化 D 轴电流环 PID。
     */
    PID_Controller_Init(&s_pid_id,
                        MOTOR_SERVICE_PID_ID_KP,
                        MOTOR_SERVICE_PID_ID_KI,
                        MOTOR_SERVICE_PID_ID_KD,
                        -MOTOR_SERVICE_CURRENT_LOOP_V_LIMIT,
                        MOTOR_SERVICE_CURRENT_LOOP_V_LIMIT,
                        -MOTOR_SERVICE_PID_INTEGRAL_LIMIT,
                        MOTOR_SERVICE_PID_INTEGRAL_LIMIT);

    /*
     * 初始化 Q 轴电流环 PID。
     */
    PID_Controller_Init(&s_pid_iq,
                        MOTOR_SERVICE_PID_IQ_KP,
                        MOTOR_SERVICE_PID_IQ_KI,
                        MOTOR_SERVICE_PID_IQ_KD,
                        -MOTOR_SERVICE_CURRENT_LOOP_V_LIMIT,
                        MOTOR_SERVICE_CURRENT_LOOP_V_LIMIT,
                        -MOTOR_SERVICE_PID_INTEGRAL_LIMIT,
                        MOTOR_SERVICE_PID_INTEGRAL_LIMIT);

    /*
     * 初始化速度环 PID。
     * 速度环输出单位为 A，对应 iq_ref。
     */
    PID_Controller_Init(&s_pid_speed,
                        MOTOR_SERVICE_PID_SPEED_KP,
                        MOTOR_SERVICE_PID_SPEED_KI,
                        MOTOR_SERVICE_PID_SPEED_KD,
                        -MOTOR_SERVICE_SPEED_LOOP_IQ_LIMIT,
                        MOTOR_SERVICE_SPEED_LOOP_IQ_LIMIT,
                        -MOTOR_SERVICE_PID_SPEED_INTEGRAL_LIMIT,
                        MOTOR_SERVICE_PID_SPEED_INTEGRAL_LIMIT);

    return MOTOR_SERVICE_OK;
}

/**
 * @brief 周期更新电机服务
 *
 * @param dt_s 周期时间，单位 s
 * @return Motor_Service_Status_t
 */
Motor_Service_Status_t Motor_Service_Update(float dt_s)
{
    Motor_Service_Status_t status;

    if (dt_s <= 0.0f)
    {
        return MOTOR_SERVICE_ERROR_PARAM;
    }

    /*
     * 统一更新传感器：
     * 1. 编码器角度
     * 2. 机械速度
     * 3. 电流采样
     * 4. Clarke / Park 变换
     */
    status = Motor_Service_UpdateSensorData(dt_s);

    if (status != MOTOR_SERVICE_OK)
    {
        return status;
    }

    /*
     * 根据当前模式执行控制算法。
     */
    if (s_motor.mode == MOTOR_SERVICE_MODE_OPEN_LOOP_ELECTRICAL)
    {
        status = Motor_Service_RunOpenLoop(dt_s);
    }
    else if (s_motor.mode == MOTOR_SERVICE_MODE_CURRENT_LOOP)
    {
        status = Motor_Service_RunCurrentLoop(dt_s);
    }
    else if (s_motor.mode == MOTOR_SERVICE_MODE_SPEED_LOOP)
    {
        status = Motor_Service_RunSpeedLoop(dt_s);
    }
    else
    {
        status = Motor_Service_RunIdle();
    }

    return status;
}

/**
 * @brief 启动开环电角度模式
 *
 * @param uq 开环 Uq 电压，单位 V
 * @param frequency_hz 电角度旋转频率，单位 Hz
 */
void Motor_Service_StartOpenLoopElectrical(float uq, float frequency_hz)
{
    s_motor.open_loop_uq = uq;
    s_motor.open_loop_frequency_hz = frequency_hz;
    s_motor.open_loop_electrical_angle = 0.0f;

    s_motor.mode = MOTOR_SERVICE_MODE_OPEN_LOOP_ELECTRICAL;
}

/**
 * @brief 设置开环 Uq 电压
 */
void Motor_Service_SetOpenLoopVoltage(float uq)
{
    s_motor.open_loop_uq = uq;
}

/**
 * @brief 设置开环电角度频率
 */
void Motor_Service_SetOpenLoopFrequency(float frequency_hz)
{
    s_motor.open_loop_frequency_hz = frequency_hz;
}

/**
 * @brief 启动电流环模式
 */
void Motor_Service_StartCurrentLoop(float id_ref, float iq_ref)
{
    s_motor.id_ref = id_ref;
    s_motor.iq_ref = iq_ref;

    s_motor.vd = 0.0f;
    s_motor.vq = 0.0f;

    s_motor.pid_speed_output = 0.0f;

    PID_Controller_Reset(&s_pid_id);
    PID_Controller_Reset(&s_pid_iq);
    PID_Controller_Reset(&s_pid_speed);

    s_motor.mode = MOTOR_SERVICE_MODE_CURRENT_LOOP;
}

/**
 * @brief 设置电流环 D/Q 轴电流指令
 */
void Motor_Service_SetCurrentReference(float id_ref, float iq_ref)
{
    s_motor.id_ref = id_ref;
    s_motor.iq_ref = iq_ref;
}

/**
 * @brief 启动速度环模式
 *
 * @param speed_ref_rpm 目标机械转速，单位 rpm
 */
void Motor_Service_StartSpeedLoop(float speed_ref_rpm)
{
    s_motor.speed_ref_rpm = speed_ref_rpm;

    s_motor.id_ref = 0.0f;
    s_motor.iq_ref = 0.0f;

    s_motor.vd = 0.0f;
    s_motor.vq = 0.0f;

    s_motor.pid_speed_error = 0.0f;
    s_motor.pid_speed_output = 0.0f;

    s_speed_loop_count = 0U;
    s_speed_iq_ref_cmd = 0.0f;

    PID_Controller_Reset(&s_pid_id);
    PID_Controller_Reset(&s_pid_iq);
    PID_Controller_Reset(&s_pid_speed);

    s_motor.mode = MOTOR_SERVICE_MODE_SPEED_LOOP;
}

/**
 * @brief 设置速度环目标转速
 *
 * @param speed_ref_rpm 目标机械转速，单位 rpm
 */
void Motor_Service_SetSpeedReference(float speed_ref_rpm)
{
    s_motor.speed_ref_rpm = speed_ref_rpm;
}

/**
 * @brief 停止电机
 */
void Motor_Service_Stop(void)
{
    s_motor.mode = MOTOR_SERVICE_MODE_IDLE;

    BSP_PWM_SetDutyABC(0.5f, 0.5f, 0.5f);

    s_motor.duty.duty_a = 0.5f;
    s_motor.duty.duty_b = 0.5f;
    s_motor.duty.duty_c = 0.5f;

    s_motor.id_ref = 0.0f;
    s_motor.iq_ref = 0.0f;

    s_motor.speed_ref_rpm = 0.0f;
    s_motor.pid_speed_output = 0.0f;

    s_motor.vd = 0.0f;
    s_motor.vq = 0.0f;

    s_speed_loop_count = 0U;

    PID_Controller_Reset(&s_pid_id);
    PID_Controller_Reset(&s_pid_iq);
    PID_Controller_Reset(&s_pid_speed);

    Motor_Service_DisableDriver();
}

/**
 * @brief 使能驱动芯片
 */
void Motor_Service_EnableDriver(void)
{
    BSP_PWM_EnableDriver();
    s_motor.driver_enabled = 1U;
}

/**
 * @brief 禁止驱动芯片
 *
 * 注意：
 * 这里只关闭驱动使能，不停止 PWM 定时器。
 */
void Motor_Service_DisableDriver(void)
{
    BSP_PWM_DisableDriver();
    s_motor.driver_enabled = 0U;
}

/**
 * @brief 编码器电角度零位对齐
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
     * 正向扫描一圈电角度。
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

    (void)AS5048A_Update(0.002f);
    angle_mid = AS5048A_GetContinuousRad();

    /*
     * 反向扫描回去。
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

    (void)AS5048A_Update(0.002f);
    angle_end = AS5048A_GetContinuousRad();

    /*
     * 判断转子是否真的发生了运动。
     */
    move = fabsf(angle_mid - angle_end);

    if (move < (FOC_2PI / 101.0f))
    {
        Motor_Service_Stop();
        return MOTOR_SERVICE_ERROR_ALIGN;
    }

    /*
     * 判断编码器方向。
     */
    if (angle_mid < angle_end)
    {
        FOC_SetDirection(-1);
    }
    else
    {
        FOC_SetDirection(1);
    }

    s_motor.direction = FOC_GetDirection();

    /*
     * 将转子吸到固定电角度。
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
     * 读取当前机械角度并计算零电角度。
     *
     * FOC 内部电角度计算关系：
     * theta_e = direction * pole_pairs * theta_m - zero_offset
     *
     * 希望当前 theta_e = MOTOR_SERVICE_ALIGN_ANGLE。
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

/**
 * @brief 获取当前运行模式
 */
Motor_Service_Mode_t Motor_Service_GetMode(void)
{
    return s_motor.mode;
}

/**
 * @brief 获取电机服务数据结构指针
 */
const Motor_Service_Data_t *Motor_Service_GetData(void)
{
    return &s_motor;
}

/**
 * @brief 获取 D 轴电流指令
 */
float Motor_Service_GetIdRef(void)
{
    return s_motor.id_ref;
}

/**
 * @brief 获取 Q 轴电流指令
 */
float Motor_Service_GetIqRef(void)
{
    return s_motor.iq_ref;
}

/**
 * @brief 获取 D 轴电流测量值
 */
float Motor_Service_GetIdMeas(void)
{
    return s_motor.id_meas;
}

/**
 * @brief 获取 Q 轴电流测量值
 */
float Motor_Service_GetIqMeas(void)
{
    return s_motor.iq_meas;
}

/**
 * @brief 获取 D 轴电压指令
 */
float Motor_Service_GetVd(void)
{
    return s_motor.vd;
}

/**
 * @brief 获取 Q 轴电压指令
 */
float Motor_Service_GetVq(void)
{
    return s_motor.vq;
}

/**
 * @brief 获取目标转速
 */
float Motor_Service_GetSpeedRef(void)
{
    return s_motor.speed_ref_rpm;
}

/**
 * @brief 获取当前转速
 */
float Motor_Service_GetSpeedRpm(void)
{
    return s_motor.speed_rpm;
}

/**
 * @brief 获取速度误差
 */
float Motor_Service_GetSpeedError(void)
{
    return s_motor.pid_speed_error;
}

/**
 * @brief 获取速度环输出
 */
float Motor_Service_GetSpeedOutput(void)
{
    return s_motor.pid_speed_output;
}
