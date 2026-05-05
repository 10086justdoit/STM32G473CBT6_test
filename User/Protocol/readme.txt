VOFA_Protocol  ：负责通信格式，把数据打包成 VOFA 能识别的格式

App 层
    ↓
Service 层：决定发什么
    ↓
Protocol 层：决定怎么打包
    ↓
BSP 层：决定怎么通过硬件发出去
    ↓
HAL 层
