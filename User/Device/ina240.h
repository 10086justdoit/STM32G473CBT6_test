#ifndef INA240_H
#define INA240_H

#include <stdint.h>

typedef enum
{
    INA240_OK = 0,
    INA240_ERROR_PARAM = -1
} INA240_Status_t;

typedef struct
{
    uint16_t raw_a;
    uint16_t raw_b;

    float voltage_a;
    float voltage_b;

    float offset_a;
    float offset_b;

    float current_a;
    float current_b;
} INA240_Data_t;

void INA240_Init(void);
INA240_Status_t INA240_Update(void);

void INA240_CalibrateOffset(uint16_t sample_count);

uint16_t INA240_GetRawA(void);
uint16_t INA240_GetRawB(void);

float INA240_GetVoltageA(void);
float INA240_GetVoltageB(void);

float INA240_GetCurrentA(void);
float INA240_GetCurrentB(void);

const INA240_Data_t *INA240_GetData(void);

#endif
