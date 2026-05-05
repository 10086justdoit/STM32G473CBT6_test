#include "bsp_uart.h"
#include "main.h"
#include "usart.h"

void BSP_UART_Init(void)
{
    /*
     * USART 已经由 CubeMX 的 MX_USARTx_UART_Init() 初始化。
     * 这里暂时为空。
     */
}

int BSP_UART_Send(const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len == 0))
    {
        return -1;
    }

    /*
     * 如果你实际使用的是 USART2 / USART3，
     * 把 huart1 改成 huart2 / huart3。
     */
    if (HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 1000) == HAL_OK)
    {
        return 0;
    }

    return -2;
}
