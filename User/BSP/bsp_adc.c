#include "bsp_adc.h"
#include "main.h"
#include "adc.h"

/*
 * ADC DMA 缓冲区
 *
 * 注意：
 * s_adc_dma_buffer[0] 对应 CubeMX ADC Regular Rank 1
 * s_adc_dma_buffer[1] 对应 CubeMX ADC Regular Rank 2
 * s_adc_dma_buffer[2] 对应 CubeMX ADC Regular Rank 3
 */
static uint16_t s_adc_dma_buffer[BSP_ADC_CHANNEL_COUNT] = {0};

void BSP_ADC_Init(void)
{
    /*
     * ADC 已经由 CubeMX 的 MX_ADC1_Init() 初始化。
     * 这里暂时为空。
     */
}

int BSP_ADC_Start(void)
{
    HAL_StatusTypeDef status;

    /*
     * 启动前先给 DMA buffer 赋个初始值，方便调试观察。
     */
    s_adc_dma_buffer[0] = 0xAAAA;
    s_adc_dma_buffer[1] = 0xBBBB;

    /*
     * 防止重复启动 DMA 前 ADC 状态没有清理干净。
     */
    (void)HAL_ADC_Stop_DMA(&hadc1);

    /*
     * 启动 ADC + DMA。
     *
     * 注意：
     * 当前 ADC1 配置为 2 个 Regular Conversion：
     * Rank 1 = ADC_CHANNEL_1
     * Rank 2 = ADC_CHANNEL_2
     *
     * 所以 DMA buffer 长度必须是 2。
     */
    status = HAL_ADC_Start_DMA(&hadc1,
                               (uint32_t *)s_adc_dma_buffer,
                               BSP_ADC_CHANNEL_COUNT);

    if (status != HAL_OK)
    {
        return -1;
    }

   /*
     * 只用 DMA 搬运数据，不使用 DMA 中断回调。
     * 避免连续 ADC + 小缓冲区导致 DMA IRQ 风暴。
     */
    if (hadc1.DMA_Handle != 0)
    {
        __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
        __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC);
    }

    return 0;
}

int BSP_ADC_Stop(void)
{
    if (HAL_ADC_Stop_DMA(&hadc1) == HAL_OK)
    {
        return 0;
    }

    return -1;
}

uint16_t BSP_ADC_GetRaw(uint8_t channel)
{
    if (channel >= BSP_ADC_CHANNEL_COUNT)
    {
        return 0;
    }

    return s_adc_dma_buffer[channel];
}

uint16_t BSP_ADC_GetPhaseARaw(void)
{
    return BSP_ADC_GetRaw(BSP_ADC_PHASE_A);
}

uint16_t BSP_ADC_GetPhaseBRaw(void)
{
    return BSP_ADC_GetRaw(BSP_ADC_PHASE_B);
}

uint16_t BSP_ADC_GetPhaseCRaw(void)
{
    return BSP_ADC_GetRaw(BSP_ADC_PHASE_C);
}
