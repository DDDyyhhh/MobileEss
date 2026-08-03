// #ifndef __RS485_H
// #define __RS485_H

// #include "usart.h"

// // RS485方向控制引脚定义
// #define RS485_DIR_PIN         GPIO_PIN_12
// #define RS485_DIR_PORT        GPIOC

// // 方向控制宏
// #define RS485_SET_TX_MODE()   HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_SET)
// #define RS485_SET_RX_MODE()   HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_RESET)

// // 接收缓冲区大小
// #define RS485_RX_BUFFER_SIZE  256

// // 函数声明
// void RS485_Init(void);
// HAL_StatusTypeDef RS485_SendData(uint8_t *data, uint16_t size);
// uint16_t RS485_ReceiveData(uint8_t *buffer, uint16_t buffer_size);
// void RS485_ClearRxBuffer(void);
// uint16_t RS485_GetRxDataCount(void);

// #endif /* __RS485_H */


#ifndef __RS485_H
#define __RS485_H

#include "usart.h" // 确保包含了CubeMX生成的usart.h

// RS485方向控制引脚定义 (与你项目一致)
#define RS485_DIR_PIN         GPIO_PIN_15
#define RS485_DIR_PORT        GPIOA

// 方向控制宏
#define RS485_SET_TX_MODE()   HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_SET)
#define RS485_SET_RX_MODE()   HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_RESET)

// 接收缓冲区大小
#define RS485_RX_BUFFER_SIZE  256

// 函数声明

// void RS485_Init(uint32_t baudrate); // 参数暂时保留，但实际波特率由CubeMX设置
static void RS485_Start_Receive(void);
void RS485_Init();
void RS485_SendData(uint8_t *data, uint16_t size);
uint16_t RS485_ReceiveData(uint8_t *buffer, uint16_t buffer_size);
void RS485_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
#endif /* __RS485_H */
