#ifndef VOFA_PROTOCOL_H
#define VOFA_PROTOCOL_H

#include <stdint.h>
#include <stdarg.h>

typedef enum
{
    VOFA_LINE_END_NONE = 0,
    VOFA_LINE_END_LF,
    VOFA_LINE_END_CRLF
} VOFA_LineEnd_t;

/*
 * JustFloat:
 * float1 + float2 + ... + 00 00 80 7F
 */
int16_t VOFA_Protocol_PackJustFloat(const float *data,
                                    uint8_t channel_num,
                                    uint8_t *tx_buf,
                                    uint16_t tx_buf_size);

/*
 * FireWater / FireWater2:
 * 文本格式打包。
 *
 * FireWater  通常使用 CRLF: \r\n
 * FireWater2 通常使用 LF  : \n
 */
int16_t VOFA_Protocol_PackFireWaterV(char *tx_buf,
                                     uint16_t tx_buf_size,
                                     VOFA_LineEnd_t line_end,
                                     const char *format,
                                     va_list args);

/*
 * 普通文本行打包。
 */
int16_t VOFA_Protocol_PackTextLine(const char *text,
                                   char *tx_buf,
                                   uint16_t tx_buf_size,
                                   VOFA_LineEnd_t line_end);

#endif


/*
 * 把数据打包成 VOFA 格式
 */
