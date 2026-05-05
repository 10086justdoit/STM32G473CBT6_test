#include "vofa_service.h"
#include "vofa_protocol.h"
#include "bsp_uart.h"

#include <stdarg.h>
#include <stdint.h>
#include <string.h>

#define VOFA_SERVICE_TX_BUF_SIZE     128U

void VOFA_Service_Init(void)
{
}

/*
 * JustFloat:
 * 发送 float 二进制数据。
 */
void VOFA_Service_JustFloat(const float *data, uint8_t channel_num)
{
    uint8_t tx_buf[VOFA_SERVICE_TX_BUF_SIZE];
    int16_t tx_len;

    tx_len = VOFA_Protocol_PackJustFloat(data,
                                         channel_num,
                                         tx_buf,
                                         sizeof(tx_buf));

    if (tx_len > 0)
    {
        BSP_UART_Send(tx_buf, (uint16_t)tx_len);
    }
}

/*
 * FireWater:
 * 文本格式，自动添加 \r\n。
 */
void VOFA_Service_FireWater(const char *format, ...)
{
    char tx_buf[VOFA_SERVICE_TX_BUF_SIZE];
    int16_t tx_len;
    va_list args;

    if (format == 0)
    {
        return;
    }

    va_start(args, format);

    tx_len = VOFA_Protocol_PackFireWaterV(tx_buf,
                                          sizeof(tx_buf),
                                          VOFA_LINE_END_CRLF,
                                          format,
                                          args);

    va_end(args);

    if (tx_len > 0)
    {
        BSP_UART_Send((uint8_t *)tx_buf, (uint16_t)tx_len);
    }
}

/*
 * FireWater2:
 * 文本格式，自动添加 \n。
 */
void VOFA_Service_FireWater2(const char *format, ...)
{
    char tx_buf[VOFA_SERVICE_TX_BUF_SIZE];
    int16_t tx_len;
    va_list args;

    if (format == 0)
    {
        return;
    }

    va_start(args, format);

    tx_len = VOFA_Protocol_PackFireWaterV(tx_buf,
                                          sizeof(tx_buf),
                                          VOFA_LINE_END_LF,
                                          format,
                                          args);

    va_end(args);

    if (tx_len > 0)
    {
        BSP_UART_Send((uint8_t *)tx_buf, (uint16_t)tx_len);
    }
}

/*
 * 发送普通文本，不自动换行。
 */
void VOFA_Service_SendText(const char *text)
{
    if (text == 0)
    {
        return;
    }

    BSP_UART_Send((const uint8_t *)text, (uint16_t)strlen(text));
}

/*
 * 发送普通文本，自动添加 \r\n。
 */
void VOFA_Service_SendTextLine(const char *text)
{
    char tx_buf[VOFA_SERVICE_TX_BUF_SIZE];
    int16_t tx_len;

    if (text == 0)
    {
        return;
    }

    tx_len = VOFA_Protocol_PackTextLine(text,
                                        tx_buf,
                                        sizeof(tx_buf),
                                        VOFA_LINE_END_CRLF);

    if (tx_len > 0)
    {
        BSP_UART_Send((uint8_t *)tx_buf, (uint16_t)tx_len);
    }
}

/*
 * 发送原始字节。
 */
void VOFA_Service_SendBytes(const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len == 0))
    {
        return;
    }

    BSP_UART_Send(data, len);
}

/*
 * 发送 UTF-8 中文：
 * “测试”
 *
 * 测 = E6 B5 8B
 * 试 = E8 AF 95
 *
 * 这样写不依赖 Keil 文件编码。
 * 即使你的 Keil 源文件是 GB2312，VOFA+ 选择 UTF-8 时也能显示中文。
 */
void VOFA_Service_SendChineseTestUtf8(void)
{
    static const uint8_t test_cn_utf8[] =
    {
        0xE6, 0xB5, 0x8B,
        0xE8, 0xAF, 0x95,
        '\r', '\n'
    };

    BSP_UART_Send(test_cn_utf8, sizeof(test_cn_utf8));
}

/*
 * 一次性发送三种格式：
 *
 * test
 * 测试
 * 0.000,0.000,0.000,0.000
 */
void VOFA_Service_TestFireWaterOnce(void)
{
    VOFA_Service_SendTextLine("test");

    VOFA_Service_SendChineseTestUtf8();

    VOFA_Service_FireWater("%.3f,%.3f,%.3f,%.3f",
                           0.0f,
                           0.0f,
                           0.0f,
                           0.0f);
}

