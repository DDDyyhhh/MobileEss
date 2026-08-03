#ifndef READ_ADC_H
#define READ_ADC_H

#include "stdint.h"
#include "adc.h"
uint16_t Read_ADC_Channel(ADC_HandleTypeDef* hadc, uint32_t channel);

#endif // READ_ADC_H