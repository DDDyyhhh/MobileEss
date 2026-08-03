// Core/Src/serial_port.c
/**
  ******************************************************************************
  * @file           : serial_port.c
  * @brief          : 线程安全的高级多串口通信中间件
  * 
  * 主要功能：
  * 1. 提供了标准的 `Serial_Printf` 和二进制 `Serial_SendData` 发送接口
  * 2. 将发送数据压入 FreeRTOS 消息队列 (`Queue_TxDataHandle`)，实现真正的异步非阻塞
  * 3. 实现了带自动文件名与行号解析的宏打印接口 `Serial_Debug_Impl`
  *
  * 注意事项：
  * - 所有调用此 API 的任务都不会被串口发送耗时卡住，极大提高了实时控制的稳定性。
  * - 如果队列塞满，采用了 `10ms` 的短超时丢弃策略，防止消费者任务卡死导致生产者雪崩。
  ******************************************************************************
  */
#include "serial_port.h"
#include "usart.h" // 包含 usart.h 以使用 huart 句柄
#include "cmsis_os.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
extern osMessageQueueId_t Queue_TxDataHandle;
extern osMessageQueueId_t Queue_TxDataWirelessHandle;

// 缓冲区大小定义
#define PRINTF_BUFFER_SIZE 128

// 【核心】定义一个UART句柄指针数组
static UART_HandleTypeDef* g_uart_handles[SERIAL_PORT_MAX];

/**
 * @brief 初始化串口模块，关联枚举ID和实际的UART句柄
 */
void Serial_Init(void)
{
    // 将枚举ID与CubeMX生成的UART句柄关联起来
    g_uart_handles[SERIAL_PORT_1] = &huart1;
    g_uart_handles[SERIAL_PORT_2] = &huart2;
    g_uart_handles[SERIAL_PORT_3] = &huart3;
    // 如果有更多串口，在这里继续添加
}

/**
 * @brief 根据Port ID获取对应的UART句柄
 * @param port_id 串口ID
 * @return 对应的UART_HandleTypeDef指针，如果ID无效则返回NULL
 */
static UART_HandleTypeDef* Get_UART_Handle(SerialPort_ID port_id)
{
    if (port_id < SERIAL_PORT_MAX) {
        return g_uart_handles[port_id];
    }
    return NULL;
}

static osStatus_t Serial_QueuePutByPort(const UartTxMsg_t* msg, uint32_t timeout)
{
    if (msg == NULL)
    {
        return osErrorParameter;
    }

    if (msg->port_id == SERIAL_PORT_3)
    {
        return osMessageQueuePut(Queue_TxDataWirelessHandle, msg, 0, timeout);
    }

    return osMessageQueuePut(Queue_TxDataHandle, msg, 0, timeout);
}

/**
 * @brief 发送单字节 (改为入队模式)
 */
void Serial_SendByte(SerialPort_ID port_id, uint8_t byte)
{
    UartTxMsg_t msg;
    msg.port_id = port_id;
    msg.buffer[0] = byte;
    msg.len = 1;

    // 发送给队列 (超时设为0或10ms均可)
    Serial_QueuePutByPort(&msg, 10);
}
/**
 * @brief 发送字符串 (改为入队模式)
 */
void Serial_SendString(SerialPort_ID port_id, const char* str)
{
    if (str == NULL) return;

    UartTxMsg_t msg;
    msg.port_id = port_id;
    
    // 计算长度并防止溢出
    size_t str_len = strlen(str);
    if (str_len > TX_BUF_SIZE) str_len = TX_BUF_SIZE;
    
    msg.len = (uint16_t)str_len;
    memcpy(msg.buffer, str, str_len);

    // 发送给队列
    Serial_QueuePutByPort(&msg, 10);
}

/**
 * @brief 在指定串口上实现 printf 功能
 * @param port_id 目标串口
 * @param format 格式化字符串
 * @param ... 可变参数
 */
void Serial_Printf(SerialPort_ID port_id, const char *format, ...)
{
    UartTxMsg_t msg;
    msg.port_id = port_id;

    va_list args;
    va_start(args, format);
    // 格式化到 msg.buffer 中
    int len = vsnprintf((char*)msg.buffer, TX_BUF_SIZE, format, args);
    va_end(args);

    if (len > 0)
    {
        msg.len = len;
        // 将整包数据放入队列
        // 如果队列满了，这里选择等待 10ms，或者直接丢弃，避免卡死调用者
        Serial_QueuePutByPort(&msg, 10);
    }
}

/**
 * @brief 发送原始二进制数组 (线程安全，入队模式)
 * @param port_id 目标串口
 * @param data 数据指针
 * @param len 数据长度
 */
void Serial_SendData(SerialPort_ID port_id, const uint8_t* data, uint16_t len)
{
    if (data == NULL || len == 0) return;

    UartTxMsg_t msg;
    msg.port_id = port_id;

    // 1. 长度限幅 (防止溢出缓冲区)
    if (len > TX_BUF_SIZE) {
        len = TX_BUF_SIZE;
    }
    msg.len = len;

    // 2. 内存拷贝
    memcpy(msg.buffer, data, len);

    // 3. 发送给队列
    // timeout 设为 10ms，如果队列满了等待一下，避免丢包
    Serial_QueuePutByPort(&msg, 10);
}


/**
 * @brief 高级调试打印实现函数
 * @note  格式: [文件名:行号] 消息内容
 */
void Serial_Debug_Impl(SerialPort_ID port_id, const char* file, int line, const char* func, const char *format, ...)
{
    UartTxMsg_t msg;
    msg.port_id = port_id;

    // 1. 提取短文件名 (去除路径)
    // Windows路径用 '\\', Linux/Mac用 '/'
    const char *short_file = strrchr(file, '/');
    if (!short_file) {
        short_file = strrchr(file, '\\');
    }
    
    if (short_file) {
        short_file++; // 跳过斜杠，指向文件名开头
    } else {
        short_file = file; // 没找到斜杠，直接用原名
    }

    // 2. 格式化前缀: "[main.c:128] "
    // 我们直接在 msg.buffer 上操作，避免额外的临时缓冲区
    int offset = snprintf((char*)msg.buffer, TX_BUF_SIZE, "[%s:%d] ", short_file, line);

    // 检查缓冲区是否溢出
    if (offset < 0 || offset >= TX_BUF_SIZE) {
        return; // 前缀太长，放弃
    }

    // 3. 格式化用户消息，拼接到前缀后面
    va_list args;
    va_start(args, format);
    // 注意：剩余空间 = 总大小 - 已用的offset
    int len = vsnprintf((char*)msg.buffer + offset, TX_BUF_SIZE - offset, format, args);
    va_end(args);

    // 4. 发送入队
    if (len > 0)
    {
        msg.len = offset + len; // 总长度 = 前缀 + 内容
        
        // 如果总长度超过缓冲区，强制截断
        if (msg.len > TX_BUF_SIZE) msg.len = TX_BUF_SIZE;

        // 发送给队列 (Timeout 0, 调试信息丢了就丢了，不要阻塞系统)
        Serial_QueuePutByPort(&msg, 0);
    }
}
