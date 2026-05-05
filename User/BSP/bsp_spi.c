#include "bsp_spi.h"
#include "main.h"
#include "spi.h"

void BSP_SPI_Init(void)
{
    /*
     * SPI 已经由 CubeMX 的 MX_SPIx_Init() 初始化。
     * 这里暂时为空。
     */
}

int BSP_SPI_AS5048A_TxRx16(uint16_t tx_data, uint16_t *rx_data)
{
    uint16_t rx_temp = 0;

    if (rx_data == 0)
    {
        return -1;
    }

    /*
     * 当前默认使用 SPI1。
     * 如果以后换成 SPI2，只改这里的 hspi1 为 hspi2。
     *
     * 注意：
     * CubeMX 中 SPI 必须配置为 16-bit。
     * Size = 1 表示传输 1 个 16-bit 数据帧。
     */
    if (HAL_SPI_TransmitReceive(&hspi1,
                                (uint8_t *)&tx_data,
                                (uint8_t *)&rx_temp,
                                1,
                                100) == HAL_OK)
    {
        *rx_data = rx_temp;
        return 0;
    }

    return -2;
}


