#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stdint.h>

void BSP_SPI_Init(void);

/*
 * AS5048A 使用 SPI 16-bit 传输。
 * CubeMX 中 SPI 需要配置为 16-bit。
 */
int BSP_SPI_AS5048A_TxRx16(uint16_t tx_data, uint16_t *rx_data);

#endif
