#ifndef INA240_H
#define INA240_H

#include <stdint.h>

typedef enum
{
    INA240_OK = 0,
    INA240_ERROR_PARAM = -1
} INA240_Status_t;

typedef struct
{
    /**< ADC 原始采样值，范围通常为 0 ~ 4095。 */
    uint16_t raw_a;        
    uint16_t raw_b;

    /**< INA240 输出电压，单位 V。 */
    float voltage_a;
    float voltage_b;

    /**< 零电流偏置电压，单位 V。 */
    float offset_a;
    float offset_b;

    /**< 电流值，单位 A。 根据 ia + ib + ic = 0 计算得到的 C 相电流。 */
    float current_a;
    float current_b;
    float current_c;
} INA240_Data_t;


void INA240_Init(void);                              /**< 初始化 INA240 模块 */
INA240_Status_t INA240_Update(void);                 /**< 更新 INA240 数据，读取 ADC 原始值，计算电压和电流 */
void INA240_CalibrateOffset(uint16_t sample_count);  /**< 校准零点偏置，采集 sample_count 个样本计算平均值作为偏置 */
float INA240_RawToVoltage(uint16_t raw);             /**< 将 ADC 原始值转换为电压，单位 V */


uint16_t INA240_GetRawA(void);      /**< 获取 A 相 ADC 原始值 */  
uint16_t INA240_GetRawB(void);      /**< 获取 B 相 ADC 原始值 */

float INA240_GetVoltageA(void);     /**< 获取 A 相电压，单位 V */
float INA240_GetVoltageB(void);     /**< 获取 B 相电压，单位 V */


float INA240_GetCurrentA(void);     /**< 获取 A 相电流，单位 A */
float INA240_GetCurrentB(void);     /**< 获取 B 相电流，单位 A */
float INA240_GetCurrentC(void);     /**< 获取 C 相电流，单位 A */

float INA240_GetOffsetA(void);      /**< 获取 A 相零点偏置电压，单位 V */
float INA240_GetOffsetB(void);      /**< 获取 B 相零点偏置电压，单位 V */


const INA240_Data_t *INA240_GetData(void);  /**< 获取 INA240 数据结构指针，包含原始值、电压、电流和偏置等信息 */

#endif

