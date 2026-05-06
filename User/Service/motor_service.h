#ifndef MOTOR_SERVICE_H
#define MOTOR_SERVICE_H

#include <stdint.h>

#include "foc.h"

typedef enum
{
    MOTOR_SERVICE_OK = 0,               /**< 正常状态 */    
    MOTOR_SERVICE_ERROR_PARAM = -1,     /**< 参数错误 */
    MOTOR_SERVICE_ERROR_PWM = -2,       /**< PWM 输出错误 */
    MOTOR_SERVICE_ERROR_ADC = -3,       /**< ADC 采样错误 */
    MOTOR_SERVICE_ERROR_ENCODER = -4,   /**< 编码器读取错误 */
    MOTOR_SERVICE_ERROR_FOC = -5,       /**< FOC 计算错误 */
    MOTOR_SERVICE_ERROR_ALIGN = -6      /**< 传感器对齐错误，可能是编码器安装位置不正确或者电机未正确对齐 */
} Motor_Service_Status_t;               

typedef enum
{
    MOTOR_SERVICE_MODE_IDLE = 0,                    /**< 空闲模式，PWM 输出占空比为 0，等待启动命令 */
    MOTOR_SERVICE_MODE_OPEN_LOOP_ELECTRICAL = 1,     /**< 开环电角度模式，根据设定的电压幅值和频率输出 PWM，占空比根据电压幅值计算，电角度根据频率积分 */
    MOTOR_SERVICE_MODE_CURRENT_LOOP = 2             /**< 电流环模式，根据设定的电流指令计算 PWM 占空比 */
} Motor_Service_Mode_t;

typedef struct
{
    Motor_Service_Mode_t mode;          /**< 当前运行模式 */

    uint8_t driver_enabled;             /**< 驱动使能状态，0 = 关闭，1 = 使能 */
    uint8_t encoder_ok;                 /**< 编码器状态，0 = 错误，1 = 正常 */

    float open_loop_electrical_angle;   /**< 开环电角度，单位 rad，范围 0 ~ 2pi */
    float open_loop_uq;                 /**< 开环 Uq 电压，单位 V */
    float open_loop_frequency_hz;       /**< 开环电角度旋转频率，单位 Hz */

    uint16_t raw_angle;                 /**< 编码器原始角度值，范围 0 ~ 16383 */
    float mechanical_angle;             /**< 机械角度，单位 rad，范围 0 ~ 2pi */
    float electrical_angle;             /**< 电角度，单位 rad，范围 0 ~ 2pi */

    float ia;                           /**< A 相电流，单位 A */    
    float ib;                           /**< B 相电流，单位 A */
    float ic;                           /**< C 相电流，单位 A */

    float ia_lpf;                       /**< A 相电流低通滤波后的值，单位 A */
    float ib_lpf;                       /**< B 相电流低通滤波后的值，单位 A */
    float ic_lpf;                       /**< C 相电流低通滤波后的值，单位 A */
   
    float id_ref;                   /**< D 轴电流指令，单位 A */
    float iq_ref;                   /**< Q 轴电流指令，单位 A */

    float id_meas;                  /**< D 轴电流测量值，单位 A */
    float iq_meas;                  /**< Q 轴电流测量值，单位 A */

    float vd;                       /**< D 轴电压，单位 V */
    float vq;                       /**< Q 轴电压，单位 V */

    float pid_id_error;             /**< D 轴电流误差，单位 A */
    float pid_iq_error;             /**< Q 轴电流误差，单位 A */

    float pid_id_output;            /**< D 轴 PID 输出，单位 V */
    float pid_iq_output;            /**< Q 轴 PID 输出，单位 V */


    FOC_AlphaBeta_t current_alpha_beta; /**< Alpha-Beta 坐标系下的电流值，单位 A */
    FOC_DQ_t current_dq;                /**< D-Q 坐标系下的电流值，单位 A */        

    FOC_DutyABC_t duty;                 /**< 三相占空比，范围 0 ~ 1 */
    FOC_SVPWM_Data_t svpwm;             /**< SVPWM 中间变量，包含扇区、Ta/Tb/Tc 和参考电压等信息 */

    float zero_electric_angle;          /**< 电角度零点对应的机械角度，单位 rad，范围 0 ~ 2pi */
    int8_t direction;                   /**< 旋转方向，1 表示正向，-1 表示反向 */
} Motor_Service_Data_t;

Motor_Service_Status_t Motor_Service_Init(void);            /* 初始化电机服务，配置 FOC 参数，启动必要的外设等 */
Motor_Service_Status_t Motor_Service_Update(float dt_s);    /* 更新电机服务状态，读取传感器数据，计算 FOC，占空比等，dt_s 是距离上次调用的时间间隔，单位 s */

void Motor_Service_StartOpenLoopElectrical(float uq, float frequency_hz);   /* 启动开环电角度模式*/
void Motor_Service_Stop(void);          /* 停止电机 */

void Motor_Service_EnableDriver(void);  /* 使能驱动 */
void Motor_Service_DisableDriver(void); /* 禁止驱动 */

void Motor_Service_SetOpenLoopVoltage(float uq);             /* 设置开环模式下的 Uq 电压幅值 */
void Motor_Service_SetOpenLoopFrequency(float frequency_hz); /* 设置开环模式下的电角度旋转频率 */

void Motor_Service_StartCurrentLoop(float id_ref, float iq_ref);    /* 启动电流环模式，设置 D/Q 轴电流指令 */
void Motor_Service_SetCurrentReference(float id_ref, float iq_ref); /* 设置电流环模式下的 D/Q 轴电流指令 */

float Motor_Service_GetIdRef(void);             /* 获取 D 轴电流指令 */
float Motor_Service_GetIqRef(void);             /* 获取 Q 轴电流指令 */
float Motor_Service_GetIdMeas(void);            /* 获取 D 轴电流测量值 */
float Motor_Service_GetIqMeas(void);            /* 获取 Q 轴电流测量值 */
float Motor_Service_GetVd(void);                /* 获取 D 轴电压 */
float Motor_Service_GetVq(void);                /* 获取 Q 轴电压 */

Motor_Service_Status_t Motor_Service_AlignSensor(void);      /* 传感器对齐流程 */
    

Motor_Service_Mode_t Motor_Service_GetMode(void);            /* 获取当前运行模式 */
const Motor_Service_Data_t *Motor_Service_GetData(void);     /* 获取电机服务数据结构指针 */

#endif
