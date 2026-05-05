#ifndef AS5048A_H
#define AS5048A_H

#include <stdint.h>

/**
 * @brief AS5048A 模块返回状态码
 *
 * 用于表示 AS5048A 初始化、更新、通信过程中的执行状态。
 */
typedef enum
{
    AS5048A_OK = 0,              /* 操作成功 */
    AS5048A_ERROR_NULL = -1,     /* 空指针错误 */
    AS5048A_ERROR_BUS = -2,      /* SPI 总线通信错误 */
    AS5048A_ERROR_PARAM = -3     /* 参数错误 */
} AS5048A_Status_t;

typedef struct
{
    uint16_t raw_angle;   /* 原始 14 位角度值，范围 0 到 16383 */
    float angle_deg;      /* 单圈角度，单位 degree，范围 0 到 360 */
    float angle_rad;      /* 单圈角度，单位 rad，范围 0 到 2pi */
    float continuous_rad; /*  连续角度，单位 rad，可跨圈累加 */

    float velocity_rad_s; /* 角速度，单位 rad/s */
    float velocity_rps;   /* 转速，单位 rps */
    float velocity_rpm;   /* 转速，单位 rpm */
} AS5048A_Data_t;

AS5048A_Status_t AS5048A_Init(void);           /* 初始化 AS5048A 模块 */
AS5048A_Status_t AS5048A_Update(float dt_s);   /* 更新 AS5048A 数据，dt_s 是距离上次更新的时间，单位秒 */

uint16_t AS5048A_GetRawAngle(void);            /* 获取原始角度值，单位是 1/16384 圈 */
float AS5048A_GetAngleDeg(void);               /* 获取单圈角度，单位 degree */
float AS5048A_GetAngleRad(void);               /* 获取单圈角度，单位 rad */
float AS5048A_GetContinuousRad(void);          /* 获取连续角度，单位 rad */
float AS5048A_GetVelocityRadPerSec(void);      /* 获取角速度，单位 rad/s */
float AS5048A_GetVelocityRps(void);            /* 获取转速，单位 rps */
float AS5048A_GetVelocityRpm(void);            /* 获取转速，单位 rpm */

const AS5048A_Data_t *AS5048A_GetData(void);   /* 获取 AS5048A 数据结构体指针，包含所有数据字段 */

/* 调试接口：读取指定帧数据，主要用于调试 SPI 通信和数据解析过程 */
AS5048A_Status_t AS5048A_DebugReadFrame(uint16_t *rx_cmd,
                                        uint16_t *rx_nop,
                                        uint16_t *raw_angle);
#endif
