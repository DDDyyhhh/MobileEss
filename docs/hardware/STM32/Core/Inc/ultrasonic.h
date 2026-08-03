// Core/Inc/ultrasonic.h

#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "main.h"
#include "tim.h"

#define ULTRASONIC_NUM_SENSORS 4
#define ULTRASONIC_TIMEOUT_MS 50 // 测量超时时间 (ms)
#define ULTRASONIC_FILTER_SIZE 3 // 中值滤波器的窗口大小 (推荐3或5)
/**
 * @brief 超声波数据结构体 (来自你的模板)
 */
typedef struct {
    volatile uint8_t status;      ///< 捕获状态：0=等待上升沿，1=等待下降沿
    volatile uint16_t IC_Time[2]; ///< 捕获的时间戳（上升沿/下降沿）
    volatile float distance;      ///< 测量得到的距离（cm）
} UltrasonicData_t;


// typedef struct
// {
//     // 状态机
//     volatile uint8_t status; // 0:空闲, 1:等待回波 (volatile很重要!)
//     uint32_t IC_Time[2];

//     // 超时处理
//     uint32_t trigger_timestamp;

//     // 数据滤波
//     float history[ULTRASONIC_FILTER_SIZE];
//     uint8_t history_index;

//     // 最终输出
//     float distance;

// } UltrasonicData_t;
// 初始化所有超声波传感器
void Ultrasonic_Init(void);

// 核心任务函数，用于轮询触发传感器
void Ultrasonic_Process(void);

// 获取指定传感器的距离
float Ultrasonic_GetDistance_cm(uint8_t sensor_index);

// 中断回调函数 (这个需要保持，因为HAL库会调用它)
// void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif // __ULTRASONIC_H