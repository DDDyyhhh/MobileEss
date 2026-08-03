/**
  ******************************************************************************
  * @file           : read_adc.c
  * @brief          : 轮询式 ADC 通道读取驱动
  * 
  * 主要功能：
  * 1. 动态配置 ADC 通道，触发单次转换并阻塞等待结果
  * 2. 适用于电池电压检测、温度传感器或其他非高速模拟量采集
  *
  * 注意事项：
  * - 采用了 `HAL_ADC_PollForConversion` 阻塞等待，虽占用 CPU 时间，但配置过程极短。
  * - 不建议在极其严格的 1ms/10ms 运动控制高优任务中密集调用，可放入后台传感器任务。
  ******************************************************************************
  */
#include "read_adc.h"
#include "adc.h"

uint16_t Read_ADC_Channel(ADC_HandleTypeDef* hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
    uint16_t value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return value;
}
