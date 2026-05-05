#include "as5048a.h"

#include "main.h"
#include "bsp_spi.h"
#include "platform_delay.h"

#include <string.h>

/* ============================================================
 * AS5048A 硬件配置区
 *
 * 后续换 CS 引脚，只改这里。
 * ============================================================ */

/*
 * 当前默认 CS = PA4。
 * CubeMX 中 PA4 需要配置为 GPIO_Output。
 */
#define AS5048A_CS_GPIO_PORT            GPIOA
#define AS5048A_CS_GPIO_PIN             GPIO_PIN_4

/*
 * CS 时序延时。
 * AS5048A_CS_SETUP_DELAY_US: CS 拉低后，等待该时间再开始 SPI 传输。
 * AS5048A_CS_HOLD_DELAY_US: SPI 传输结束后，等待该时间再拉高 CS。
 */
#define AS5048A_CS_SETUP_DELAY_US       1U
#define AS5048A_CS_HOLD_DELAY_US        1U

/* ============================================================
 * AS5048A 命令与常量
 * ============================================================ */

#define AS5048A_CMD_CLEAR_ERROR         0x4001U         /* 清除 AS5048A 错误标志命令 */
#define AS5048A_CMD_NOP                 0xC000U         /* NOP 命令，常用于读取数据的第二帧 */
#define AS5048A_CMD_READ_ANGLE          0xFFFFU         /* 读取角度命令，第一帧发送该命令，第二帧接收角度数据 */

#define AS5048A_ANGLE_MASK              0x3FFFU         /* 14 位角度数据掩码，范围 0 到 16383 */
#define AS5048A_CPR                     16384.0f        /* Counts Per Revolution，单圈计数，2^14 = 16384 */
#define AS5048A_TWO_PI                  6.28318530718f  /* 2 * pi */
#define AS5048A_PI                      3.14159265359f  /* pi */
#define AS5048A_RAD_TO_DEG              57.2957795131f  /* 180 / pi */

/* ============================================================
 * 模块内部变量
 * ============================================================ */

static AS5048A_Data_t s_as5048a_data;       /* AS5048A 数据结构体实例 */

static float s_last_angle_rad = 0.0f;       /* 上一次更新的角度值，单位 rad，用于计算连续角度和角速度 */
static uint8_t s_has_last_angle = 0U;       /* 是否已经有上一次角度值的标志，0 表示没有，1 表示有 */

/* ============================================================
 * 内部函数
 * ============================================================ */


 /**
 * @brief 拉低 AS5048A 片选 CS,选中 AS5048A，准备进行 SPI 通信。
 */
static void AS5048A_CS_Low(void)
{
    HAL_GPIO_WritePin(AS5048A_CS_GPIO_PORT,
                      AS5048A_CS_GPIO_PIN,
                      GPIO_PIN_RESET);
}

/**
 * @brief 拉高 AS5048A 片选 CS,取消选中 AS5048A，结束 SPI 通信。
 */
static void AS5048A_CS_High(void)
{
    HAL_GPIO_WritePin(AS5048A_CS_GPIO_PORT,
                      AS5048A_CS_GPIO_PIN,
                      GPIO_PIN_SET);
}

/**
 * @brief 将角度差限制到 -pi 到 +pi 范围
 */
static float AS5048A_WrapDeltaRad(float delta)
{
    if (delta > AS5048A_PI)
    {
        delta -= AS5048A_TWO_PI;
    }
    else if (delta < -AS5048A_PI)
    {
        delta += AS5048A_TWO_PI;
    }

    return delta;
}

/**
 * @brief 传输一帧 AS5048A SPI 数据
 *
 * @param tx_data 待发送的数据
 * @param rx_data 接收到的数据
 * @return 状态码
 */
static AS5048A_Status_t AS5048A_TransferFrame(uint16_t tx_data,
                                              uint16_t *rx_data)
{
    int ret;

    if (rx_data == 0)
    {
        return AS5048A_ERROR_NULL;
    }

    AS5048A_CS_Low();

    Platform_DelayUs(AS5048A_CS_SETUP_DELAY_US);

    ret = BSP_SPI_AS5048A_TxRx16(tx_data, rx_data);

    AS5048A_CS_High();

    Platform_DelayUs(AS5048A_CS_HOLD_DELAY_US);

    if (ret != 0)
    {
        return AS5048A_ERROR_BUS;
    }

    return AS5048A_OK;
}


/**
 * @brief 读取 AS5048A 原始角度值
 * @param raw_angle 用于保存读取到的 14 位原始角度值
 * @return AS5048A_OK 读取成功
 * @return AS5048A_ERROR_NULL raw_angle 为空
 * @return AS5048A_ERROR_BUS SPI 通信失败
 */
static AS5048A_Status_t AS5048A_ReadRawInternal(uint16_t *raw_angle)
{
    uint16_t dummy;
    uint16_t rx_angle;
    AS5048A_Status_t status;

    if (raw_angle == 0)
    {
        return AS5048A_ERROR_NULL;
    }

    /*
     * AS5048A 读取角度具有流水线特性：
     * 第 1 帧发送 READ_ANGLE。
     * 第 2 帧发送 NOP。
     * 第 2 帧接收到的才是角度数据。
     */
    status = AS5048A_TransferFrame(AS5048A_CMD_READ_ANGLE, &dummy);
    if (status != AS5048A_OK)
    {
        return status;
    }

    status = AS5048A_TransferFrame(AS5048A_CMD_NOP, &rx_angle);
    if (status != AS5048A_OK)
    {
        return status;
    }

    *raw_angle = (uint16_t)(rx_angle & AS5048A_ANGLE_MASK);

    return AS5048A_OK;
}

/* ============================================================
 * 对外接口
 * ============================================================ */

 /**
 * @brief 初始化 AS5048A 模块
 *
 * 功能：
 * 1. 清空内部数据结构体
 * 2. 清空历史角度状态
 * 3. 将 CS 拉高，保证 AS5048A 处于未选中状态
 * 4. 可选发送清除错误标志命令
 *
 * 调用要求：
 * 1. CubeMX 已完成 SPI 初始化
 * 2. CubeMX 已完成 CS GPIO 初始化
 * 3. Platform_DelayInit() 已经调用
 * 4. BSP_SPI_Init() 已经调用
 *
 * @return AS5048A_OK 初始化成功
 */
AS5048A_Status_t AS5048A_Init(void)
{
    uint16_t dummy;
    AS5048A_Status_t status;

    memset(&s_as5048a_data, 0, sizeof(s_as5048a_data));

    s_last_angle_rad = 0.0f;
    s_has_last_angle = 0U;

    AS5048A_CS_High();

    status = AS5048A_TransferFrame(AS5048A_CMD_CLEAR_ERROR, &dummy);
    if (status != AS5048A_OK)
    {
        return status;
    }

    status = AS5048A_TransferFrame(AS5048A_CMD_NOP, &dummy);
    if (status != AS5048A_OK)
    {
        return status;
    }

    return AS5048A_OK;
}


/**
 * @brief 更新 AS5048A 当前角度、连续角度和速度
 *
 * @param dt_s 时间间隔（秒）
 * @return AS5048A_OK 更新成功
 */
AS5048A_Status_t AS5048A_Update(float dt_s)
{
    uint16_t raw;
    float angle_rad;
    float delta_rad;
    AS5048A_Status_t status;

    if (dt_s < 0.0f)
    {
        return AS5048A_ERROR_PARAM;
    }

    status = AS5048A_ReadRawInternal(&raw);
    if (status != AS5048A_OK)
    {
        return status;
    }

    angle_rad = ((float)raw / AS5048A_CPR) * AS5048A_TWO_PI;

    s_as5048a_data.raw_angle = raw;
    s_as5048a_data.angle_rad = angle_rad;
    s_as5048a_data.angle_deg = angle_rad * AS5048A_RAD_TO_DEG;

    if (s_has_last_angle == 0U)
    {
        s_as5048a_data.continuous_rad = angle_rad;

        s_as5048a_data.velocity_rad_s = 0.0f;
        s_as5048a_data.velocity_rps = 0.0f;
        s_as5048a_data.velocity_rpm = 0.0f;

        s_last_angle_rad = angle_rad;
        s_has_last_angle = 1U;

        return AS5048A_OK;
    }

    delta_rad = angle_rad - s_last_angle_rad;
    delta_rad = AS5048A_WrapDeltaRad(delta_rad);

    s_as5048a_data.continuous_rad += delta_rad;

    if (dt_s > 0.0f)
    {
        s_as5048a_data.velocity_rad_s = delta_rad / dt_s;
        s_as5048a_data.velocity_rps = s_as5048a_data.velocity_rad_s / AS5048A_TWO_PI;
        s_as5048a_data.velocity_rpm = s_as5048a_data.velocity_rps * 60.0f;
    }
    else
    {
        s_as5048a_data.velocity_rad_s = 0.0f;
        s_as5048a_data.velocity_rps = 0.0f;
        s_as5048a_data.velocity_rpm = 0.0f;
    }

    s_last_angle_rad = angle_rad;

    return AS5048A_OK;
}

/**
 * @brief 获取最新原始角度值
 *
 * @return 原始角度值，范围 0 到 16383
 */
uint16_t AS5048A_GetRawAngle(void)
{
    return s_as5048a_data.raw_angle;
}

/**
 * @brief 获取最新角度值，单位 degree
 *
 * @return 角度值，范围 0 到 360
 */
float AS5048A_GetAngleDeg(void)
{
    return s_as5048a_data.angle_deg;
}

/**
 * @brief 获取最新角度值，单位 rad
 *
 * @return 弧度值，范围 0 到 2pi
 */
float AS5048A_GetAngleRad(void)
{
    return s_as5048a_data.angle_rad;
}

/**
 * @brief 获取连续弧度角
 *
 * @return 连续弧度角，可大于 2pi，也可小于 0
 */
float AS5048A_GetContinuousRad(void)
{
    return s_as5048a_data.continuous_rad;
}

/**
 * @brief 获取最新角速度，单位 rad/s
 *
 * @return 角速度，单位 rad/s
 */
float AS5048A_GetVelocityRadPerSec(void)
{
    return s_as5048a_data.velocity_rad_s;
}

/**
 * @brief 获取最新角速度，单位 rps
 *
 * @return 角速度，单位 rps
 */
float AS5048A_GetVelocityRps(void)
{
    return s_as5048a_data.velocity_rps;
}

/**
 * @brief 获取最新角速度，单位 rpm
 *
 * @return 角速度，单位 rpm
 */
float AS5048A_GetVelocityRpm(void)
{
    return s_as5048a_data.velocity_rpm;
}

/**
 * @brief 获取完整 AS5048A 数据结构体
 *
 * 功能：
 * 外部可以通过该函数一次性读取所有角度和速度数据。
 *
 * 注意：
 * 返回的是内部静态变量地址，只建议读，不建议写。
 *
 * @return AS5048A_Data_t 数据结构体指针
 */
const AS5048A_Data_t *AS5048A_GetData(void)
{
    return &s_as5048a_data;
}

/**
 * @brief 调试接口：读取指定帧数据
 *
 * @param rx_cmd 接收命令数据的指针
 * @param rx_nop 接收 NOP 数据的指针
 * @param raw_angle 接收原始角度数据的指针
 * @return AS5048A_OK 调试读取成功
 */
AS5048A_Status_t AS5048A_DebugReadFrame(uint16_t *rx_cmd,
                                        uint16_t *rx_nop,
                                        uint16_t *raw_angle)
{
    AS5048A_Status_t status;

    if ((rx_cmd == 0) || (rx_nop == 0) || (raw_angle == 0))
    {
        return AS5048A_ERROR_NULL;
    }

    status = AS5048A_TransferFrame(AS5048A_CMD_READ_ANGLE, rx_cmd);
    if (status != AS5048A_OK)
    {
        return status;
    }

    status = AS5048A_TransferFrame(AS5048A_CMD_NOP, rx_nop);
    if (status != AS5048A_OK)
    {
        return status;
    }

    *raw_angle = (uint16_t)(*rx_nop & AS5048A_ANGLE_MASK);

    return AS5048A_OK;
}

