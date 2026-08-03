// Core/Inc/serial_port.h

#ifndef __SERIAL_PORT_H
#define __SERIAL_PORT_H

// #include "main.h"
#include "usart.h" // 包含 usart.h 以便使用 UART_HandleTypeDef
#include <stdio.h>
#include <stdarg.h> // 包含 stdarg.h

// 定义一个枚举类型来清晰地表示不同的串口
typedef enum {
    SERIAL_PORT_1 = 0, // 对应 USART1
    SERIAL_PORT_2, // 对应 USART2
    SERIAL_PORT_3, // 对应 USART3
    SERIAL_PORT_MAX
} SerialPort_ID;

// 3. 【关键】将结构体定义移到这里 (放在枚举之后)
#define RX_BUF_SIZE 64
typedef struct {
    uint8_t data[RX_BUF_SIZE];
    uint16_t len;
} UartRxMsg_t;

#define TX_BUF_SIZE 128 // 记得改成 140 或更大，防止溢出
typedef struct {
    SerialPort_ID port_id;       // 现在这里能识别 SerialPort_ID 了
    uint8_t buffer[TX_BUF_SIZE];
    uint16_t len;
} UartTxMsg_t;


// 函数声明 - 接口已修改
void Serial_Init(void); // 初始化所有串口
void Serial_SendByte(SerialPort_ID port_id, uint8_t byte);
void Serial_SendString(SerialPort_ID port_id, const char* str);
void Serial_Printf(SerialPort_ID port_id, const char *format, ...);
void Serial_SendData(SerialPort_ID port_id, const uint8_t* data, uint16_t len);


// ============================================================
//                 高级调试功能
// ============================================================

// 1. 调试开关 (注释掉这行可以全局关闭调试打印，节省空间)
#define ENABLE_SERIAL_DEBUG  0

// 2. 实际的实现函数 (不要直接调用这个，用下面的宏)
void Serial_Debug_Impl(SerialPort_ID port_id, const char* file, int line, const char* func, const char *format, ...);

// 3. 调试宏 (用户调用这个)
// 使用 ##__VA_ARGS__ 是为了处理没有参数的情况
#if ENABLE_SERIAL_DEBUG
    #define Serial_Debug(port, fmt, ...) \
        Serial_Debug_Impl(port, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)
#else
    #define Serial_Debug(port, fmt, ...) ((void)0)
#endif

#endif /* __SERIAL_PORT_H */


