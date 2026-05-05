#ifndef FOC_H
#define FOC_H

#include <stdint.h>

/* ============================================================
 * FOC 数学常量
 * ============================================================ */

#define FOC_PI              3.14159265359f
#define FOC_PI_2            1.57079632679f
#define FOC_PI_3            1.04719755120f
#define FOC_2PI             6.28318530718f
#define FOC_3PI_2           4.71238898038f

#define FOC_SQRT3           1.73205080757f      /* sqrt(3) */
#define FOC_INV_SQRT3       0.57735026919f      /* 1 / sqrt(3) */

/* ============================================================
 * FOC 状态码
 * ============================================================ */

typedef enum
{
    FOC_OK = 0,
    FOC_ERROR_PARAM = -1
} FOC_Status_t;

/* ============================================================
 * FOC 数据结构
 * ============================================================ */

typedef struct
{
    float a;
    float b;
    float c;
} FOC_ABC_t;

typedef struct
{
    float alpha;
    float beta;
} FOC_AlphaBeta_t;

typedef struct
{
    float d;
    float q;
} FOC_DQ_t;

typedef struct
{
    float duty_a;
    float duty_b;
    float duty_c;
} FOC_DutyABC_t;

typedef struct
{
    uint8_t sector;

    float t0;
    float t1;
    float t2;

    float ta;
    float tb;
    float tc;

    float u_ref;
    float vector_angle;
} FOC_SVPWM_Data_t;

typedef struct
{
    float bus_voltage;
    float zero_electric_angle;
    uint8_t pole_pairs;
    int8_t direction;
} FOC_Config_t;

typedef struct
{
    FOC_Config_t config;

    float mechanical_angle;
    float electrical_angle;

    FOC_DQ_t voltage_dq;
    FOC_AlphaBeta_t voltage_alpha_beta;
    FOC_ABC_t voltage_abc;

    FOC_DutyABC_t duty;
    FOC_SVPWM_Data_t svpwm;
} FOC_State_t;

/* ============================================================
 * 初始化与参数配置
 * ============================================================ */

void FOC_Init(void);

void FOC_SetBusVoltage(float bus_voltage);                      /* 设置总线电压，单位 V */
void FOC_SetPolePairs(uint8_t pole_pairs);                      /* 设置电机极对数，必须大于 0 */    
void FOC_SetDirection(int8_t direction);                        /* 设置旋转方向，1 表示正向，-1 表示反向 */
void FOC_SetZeroElectricAngle(float zero_electric_angle);       /* 设置电角度零点对应的机械角度，单位 rad */

float FOC_GetBusVoltage(void);              /* 获取总线电压，单位 V */
uint8_t FOC_GetPolePairs(void);             /* 获取电机极对数 */
int8_t FOC_GetDirection(void);              /* 获取旋转方向 */
float FOC_GetZeroElectricAngle(void);       /* 获取电角度零点对应的机械角度，单位 rad */

const FOC_State_t *FOC_GetState(void);      /* 获取 FOC 状态结构体指针，包含当前角度、电压、占空比等信息 */

/* ============================================================
 * 基础数学工具
 * ============================================================ */

float FOC_LimitFloat(float value, float min_value, float max_value);  /* 限制浮点数在指定范围内 */
float FOC_NormalizeAngle(float angle_rad);                            /* 将角度规范化到 0 ~ 2pi 范围内 */


/* ============================================================
 * 坐标变换
 * ============================================================ */

FOC_AlphaBeta_t FOC_Clarke(float ia, float ib);                     /* Clark 变换 */
FOC_DQ_t FOC_Park(FOC_AlphaBeta_t input, float theta_rad);          /* Park 变换 */

FOC_AlphaBeta_t FOC_InvPark(FOC_DQ_t input, float theta_rad);       /* 逆 Park 变换 */
FOC_ABC_t FOC_InvClarke(FOC_AlphaBeta_t input);                     /* 逆 Clark 变换 */

/* ============================================================
 * 电角度与 SVPWM
 * ============================================================ */

float FOC_GetElectricalAngle(float mechanical_angle_rad);           /* 根据机械角度计算电角度，考虑极对数和零点偏移 */

/* 计算 SVPWM 占空比 */
FOC_Status_t FOC_CalcSVPWM(float u_d,
                           float u_q,
                           float electrical_angle_rad,
                           FOC_SVPWM_Data_t *svpwm,
                           FOC_DutyABC_t *duty);

/*
 * 使用机械角度输入。
 * 后面闭环 FOC 时用这个：
 * mechanical_angle_rad 来自 AS5048A。
 */
FOC_Status_t FOC_SetVoltageDQ(float u_d,
                              float u_q,
                              float mechanical_angle_rad);

/*
 * 使用电角度输入。
 * 开环测试时更推荐用这个：
 * 直接给定 electrical_angle_rad。
 */
FOC_Status_t FOC_SetVoltageDQElectrical(float u_d,
                                        float u_q,
                                        float electrical_angle_rad);

FOC_DutyABC_t FOC_GetDutyABC(void);         /* 获取最新的 ABC 占空比 */
FOC_SVPWM_Data_t FOC_GetSVPWMData(void);    /* 获取最新的 SVPWM 数据 */

#endif

