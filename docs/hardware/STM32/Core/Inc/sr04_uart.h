#ifndef __SR04_UART_H
#define __SR04_UART_H

#include "main.h"

// 定义接收缓冲区大小
#define SR04_RX_BUF_SIZE 64
#define SR04_SENSOR_COUNT 4

// 全局变量声明
// extern float sr04_distances[SR04_SENSOR_COUNT]; // 建议改为通过函数获取，或者加 volatile

// 函数声明
void SR04_UART_Init(void);
// void SR04_UART_IRQHandler_Callback(void); // 删除这个，不再需要手动调用
void SR04_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size); // 新增回调处理
void SR04_Process_Data(uint8_t *data, uint16_t len); // 新增处理函数
float SR04_GetDistance(uint8_t index); // 新增线程安全的获取函数

#endif