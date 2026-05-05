#include "vofa_protocol.h"
#include <string.h>
#include <stdio.h>

#define VOFA_JUSTFLOAT_TAIL_SIZE    4U

int16_t VOFA_Protocol_PackJustFloat(const float *data,
                                    uint8_t channel_num,
                                    uint8_t *tx_buf,
                                    uint16_t tx_buf_size)
{
    uint16_t data_len;

    if ((data == 0) || (tx_buf == 0) || (channel_num == 0))
    {
        return -1;
    }

    data_len = (uint16_t)(channel_num * sizeof(float));

    if (tx_buf_size < (data_len + VOFA_JUSTFLOAT_TAIL_SIZE))
    {
        return -2;
    }

    memcpy(tx_buf, data, data_len);

    /*
     * VOFA JustFloat 帧尾：
     * 00 00 80 7F
     */
    tx_buf[data_len + 0] = 0x00;
    tx_buf[data_len + 1] = 0x00;
    tx_buf[data_len + 2] = 0x80;
    tx_buf[data_len + 3] = 0x7F;

    return (int16_t)(data_len + VOFA_JUSTFLOAT_TAIL_SIZE);
}

int16_t VOFA_Protocol_PackFireWaterV(char *tx_buf,
                                     uint16_t tx_buf_size,
                                     VOFA_LineEnd_t line_end,
                                     const char *format,
                                     va_list args)
{
    uint16_t reserve_len = 0;
    uint16_t format_buf_size;
    int n;

    if ((tx_buf == 0) || (format == 0) || (tx_buf_size == 0))
    {
        return -1;
    }

    if (line_end == VOFA_LINE_END_LF)
    {
        reserve_len = 1;
    }
    else if (line_end == VOFA_LINE_END_CRLF)
    {
        reserve_len = 2;
    }
    else
    {
        reserve_len = 0;
    }

    if (tx_buf_size <= reserve_len)
    {
        return -2;
    }

    /*
     * 预留 \n 或 \r\n 的空间
     */
    format_buf_size = (uint16_t)(tx_buf_size - reserve_len);

    n = vsnprintf(tx_buf, format_buf_size, format, args);

    if (n < 0)
    {
        return -3;
    }

    if ((uint16_t)n >= format_buf_size)
    {
        return -4;
    }

    if (line_end == VOFA_LINE_END_LF)
    {
        tx_buf[n++] = '\n';
    }
    else if (line_end == VOFA_LINE_END_CRLF)
    {
        tx_buf[n++] = '\r';
        tx_buf[n++] = '\n';
    }

    return (int16_t)n;
}

int16_t VOFA_Protocol_PackTextLine(const char *text,
                                   char *tx_buf,
                                   uint16_t tx_buf_size,
                                   VOFA_LineEnd_t line_end)
{
    uint16_t text_len;
    uint16_t end_len = 0;

    if ((text == 0) || (tx_buf == 0) || (tx_buf_size == 0))
    {
        return -1;
    }

    text_len = (uint16_t)strlen(text);

    if (line_end == VOFA_LINE_END_LF)
    {
        end_len = 1;
    }
    else if (line_end == VOFA_LINE_END_CRLF)
    {
        end_len = 2;
    }
    else
    {
        end_len = 0;
    }

    if (tx_buf_size < (text_len + end_len))
    {
        return -2;
    }

    memcpy(tx_buf, text, text_len);

    if (line_end == VOFA_LINE_END_LF)
    {
        tx_buf[text_len++] = '\n';
    }
    else if (line_end == VOFA_LINE_END_CRLF)
    {
        tx_buf[text_len++] = '\r';
        tx_buf[text_len++] = '\n';
    }

    return (int16_t)text_len;
}
