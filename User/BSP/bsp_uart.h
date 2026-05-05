#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

void BSP_UART_Init(void);
int BSP_UART_Send(const uint8_t *data, uint16_t len);

#endif
