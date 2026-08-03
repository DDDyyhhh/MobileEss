/**
  ******************************************************************************
  * @file           : rs485.c
  * @brief          : RS485 半双工总线底层通信驱动 (基于 DMA + UART IDLE)
  * 
  * 主要功能：
  * 1. 提供 RS485 收发方向使能引脚 (DE/RE) 的自动切换
  * 2. 发送逻辑：使用带超时的阻塞式发送 (`HAL_UART_Transmit`) 保证时序无缝衔接
  * 3. 接收逻辑：采用 `HAL_UARTEx_ReceiveToIdle_DMA` 实现不定长帧的异步接收
  * 4. 结合 FreeRTOS 二值信号量 (`Sem_RS485_RxHandle`) 将中断事件通知到任务层
  *
  * 注意事项：
  * - 【硬件时序红线】：发送完成后必须立刻调用 `RS485_SET_RX_MODE()` 切回接收模式，
  *   并马上重启 DMA 接收，否则会导致立刻返回的 Modbus 响应被从机截断/丢失。
  ******************************************************************************
  */
#include "rs485.h"
#include "usart.h"   // 需要 huart4
#include <string.h>  // 需要 memcpy
#include <stdbool.h> // 需要 bool
#include "cmsis_os.h" // 需要 osSemaphoreId_t
#include "serial_port.h" // 需要 Serial_Printf 用于调试输出
// 接收缓冲区和状态变量
static uint8_t rs485_dma_rx_buffer[RS485_RX_BUFFER_SIZE];
static volatile uint16_t rx_data_len = 0;
static volatile bool rx_data_ready = false;
uint32_t last_tx;

// 引用信号量
extern osSemaphoreId_t Sem_RS485_RxHandle; // 用于通知接收到数据
extern osSemaphoreId_t Sem_RS485_TxHandle; // 【新增】用于等待发送完成 (请在CubeMX添加)

// 启动一次新的DMA+IDLE接收
static void RS485_Start_Receive(void)
{
    rx_data_ready = false;
    // 这是HAL库专门用于接收不定长数据的函数
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rs485_dma_rx_buffer, RS485_RX_BUFFER_SIZE);
}

// 初始化函数
void RS485_Init(void)
{
    RS485_SET_RX_MODE();
    RS485_Start_Receive();
}

// 发送函数
/**
 * @brief RS485 发送数据 (DMA + 信号量阻塞)
 * @note  这会让当前任务(Task_BMS)进入阻塞状态，让出CPU给其他任务，直到发送完成
 */
// void RS485_SendData(uint8_t *data, uint16_t size)
// {
//     // 1. 停止接收 DMA
//     HAL_UART_DMAStop(&huart4);

//     // 2. 切换 GPIO 到发送模式 (拉高 DE)
//     RS485_SET_TX_MODE();

//     // 3. 启动 DMA 发送
//     if (HAL_UART_Transmit_IT(&huart4, data, size) == HAL_OK)
//     {
//         // 4. 等待发送完成信号量
//         // 注意：GPIO 切换现在由中断回调处理，这里只负责挂起任务
//         if (Sem_RS485_TxHandle != NULL) {
//             osSemaphoreAcquire(Sem_RS485_TxHandle, 50);
//         }
//     }
//     else
//     {
//         // 如果发送启动失败，立即切回接收模式防止死锁
//         RS485_SET_RX_MODE();
//         RS485_Start_Receive();
//     }
    
//     // 任务醒来后，说明发送已完成且 ISR 已经把模式切回 RX 了
//     // 这里只需要确保接收已经重启
//     // (为了双重保险，可以在这里再调一次 Start_Receive，HAL库允许重复调用)
//     RS485_Start_Receive();
// }
void RS485_SendData(uint8_t *data, uint16_t size)
{
    // 1. 必须先停止接收 DMA，防止冲突
    HAL_UART_DMAStop(&huart4);

    // 2. 切换到发送模式
    RS485_SET_TX_MODE();

    // 3. 使用阻塞发送 (Timeout 10ms 足够了)
    // 这行代码会等数据发完才返回，保证时序绝对正确
    if (HAL_UART_Transmit(&huart4, data, size, 10) != HAL_OK)
    {
        Serial_Printf(SERIAL_PORT_2, "[485] TX Error!\r\n");
    }

    // 4. 发送完毕，立即切回接收模式
    // 这一步至关重要，必须紧跟在 Transmit 后面
    RS485_SET_RX_MODE();

    // 5. 立即重启 DMA 接收，准备听 BMS 回话
    RS485_Start_Receive();
}

// 接收函数
/**
 * @brief 获取接收到的数据
 * @note  配合 ModbusRtuBus 使用
 */
uint16_t RS485_ReceiveData(uint8_t *buffer, uint16_t buffer_size)
{
    uint16_t len = 0;

    // 这里的逻辑需要适配 RTOS。
    // 如果使用了 Sem_RS485_RxHandle，通常是在任务里 Wait 信号量，然后直接读 buffer。
    // 但为了兼容你现有的 ModbusRtuBus_Process 轮询方式，我们加个临界区读数据。
    
    taskENTER_CRITICAL();
    if (rx_data_len > 0)
    {
        len = (rx_data_len < buffer_size) ? rx_data_len : buffer_size;
        memcpy(buffer, rs485_dma_rx_buffer, len);
        
        // 读完后清零长度，避免重复读取
        rx_data_len = 0; 
    }
    taskEXIT_CRITICAL();

    return len;
}

// *** 关键：UART接收事件回调函数 (由DMA IDLE中断触发) ***
// 这是整个方案的核心。HAL_UART_IRQHandler 会自动调用它。
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//     if (huart->Instance == UART4)
//     {
//         // IDLE中断被触发，意味着一帧数据接收完成
//         rx_data_len = Size;
//         rx_data_ready = true;

//         // **重要**：HAL库在调用这个回调后，DMA和接收就停止了。
//         // 我们必须在这里手动重启下一次接收，为下一帧数据做准备。
//         HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rs485_dma_rx_buffer, RS485_RX_BUFFER_SIZE);
//     }
// }

void RS485_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART4)
    {
        rx_data_len = Size;
        // rx_data_ready = true; // 不需要这个标志位了
        // Do not print from UART ISR; it can disturb RS485 timing.
        // 唤醒 BMS 任务
        if (Sem_RS485_RxHandle != NULL) {
            osSemaphoreRelease(Sem_RS485_RxHandle);
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rs485_dma_rx_buffer, RS485_RX_BUFFER_SIZE);
    }
}

// 发送完成回调 (TC)
// 注意：需要在 main.c 的 HAL_UART_TxCpltCallback 中调用这个，或者统一分发
void RS485_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // if (huart->Instance == UART4)
    // {
    //     // 1. 立即拉低 DE，释放总线
    //     RS485_SET_RX_MODE();
        
    //     // 2. 释放信号量唤醒任务
    //     if (Sem_RS485_TxHandle != NULL) {
    //         osSemaphoreRelease(Sem_RS485_TxHandle);
    //     }
    // }
}
