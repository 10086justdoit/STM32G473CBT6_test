#ifndef VOFA_SERVICE_H
#define VOFA_SERVICE_H

#include <stdint.h>

void VOFA_Service_Init(void);

/*
 * JustFloat:
 * 发送 float 二进制数据。
 * VOFA+ 选择 JustFloat。
 */
void VOFA_Service_JustFloat(const float *data, uint8_t channel_num);

/*
 * FireWater:
 * 文本格式，自动添加 \r\n。
 * VOFA+ 选择 FireWater。
 */
void VOFA_Service_FireWater(const char *format, ...);

/*
 * FireWater2:
 * 文本格式，自动添加 \n。
 * VOFA+ 选择 FireWater。
 */
void VOFA_Service_FireWater2(const char *format, ...);

/*
 * 发送普通文本，不自动换行。
 */
void VOFA_Service_SendText(const char *text);

/*
 * 发送普通文本，自动添加 \r\n。
 */
void VOFA_Service_SendTextLine(const char *text);

/*
 * 发送 UTF-8 原始字节。
 * 适合发送中文、特殊字符。
 */
void VOFA_Service_SendBytes(const uint8_t *data, uint16_t len);

/*
 * 发送 UTF-8 中文“测试”，自动添加 \r\n。
 */
void VOFA_Service_SendChineseTestUtf8(void);

/*
 * FireWater 一次性测试：
 * test
 * 测试
 * 0.000,0.000,0.000,0.000
 */
void VOFA_Service_TestFireWaterOnce(void);


#endif


/*
 * VOFA 服务
 *
 * 1. 调用 VOFA_Protocol 打包
 * 2. 调用 BSP_UART_Send 发送
 * 3. 决定发送哪些变量
 * 4. 决定什么时候发送
 *
 * JustFloat：
 * 只发 float 二进制数据
 * 格式：float + float + ... + 00 00 80 7F
 * UTF-8 接收区显示乱码是正常的
 * 适合高速波形、电机变量、实时曲线
 *
 * FireWater：
 * 发送 ASCII / UTF-8 文本
 * 格式：1.23,2.34,3.45\n
 * UTF-8 接收区正常显示
 * 适合日志、状态打印、低速调试、人工查看
 */
 
