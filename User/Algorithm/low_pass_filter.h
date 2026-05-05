#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

#include <stdint.h>

/**
 * @brief 一阶低通滤波器对象
 *
 * 滤波公式：
 * y(k) = y(k-1) + alpha * [x(k) - y(k-1)]
 *
 * alpha 越小，滤波越强，响应越慢。
 * alpha 越大，响应越快，滤波越弱。
 */
typedef struct
{
    float alpha;
    float output;
    uint8_t initialized;
} LowPassFilter_t;

void LowPassFilter_Init(LowPassFilter_t *filter, float alpha);      /** 初始化低通滤波器 */
void LowPassFilter_Reset(LowPassFilter_t *filter, float value);     /** 重置滤波器输出值 */

float LowPassFilter_Update(LowPassFilter_t *filter, float input);   /** 更新滤波器输出值 */
void LowPassFilter_SetAlpha(LowPassFilter_t *filter, float alpha);  /** 设置滤波系数，范围 0.0 ~ 1.0 */
float LowPassFilter_GetOutput(const LowPassFilter_t *filter);       /** 获取当前滤波器输出值 */

#endif
