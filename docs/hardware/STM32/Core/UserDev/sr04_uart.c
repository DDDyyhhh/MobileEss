/**
  ******************************************************************************
  * @file           : sr04_uart.c
  * @brief          : UART 型超声波测距阵列数据解析与软件滤波处理
  * 
  * 主要功能：
  * 1. 利用 DMA IDLE 中断异步接收 4 路超声波传感器拼接的数据包 (`0xCB 0x55 0x04 ...`)
  * 2. 提供基于状态机的粘包/断帧解析
  * 3. 防抖抗干扰滤波 (`SR04_Secure_Update`)：
  *    - 突波剔除 (Glitch Filter)：当距离跳变大于 `MAX_JUMP_THRESHOLD` 时，需多次确认
  *    - 动态不对称低通滤波：靠近时 (变小) 响应快，远离时 (变大) 响应慢，极大增强避障安全性
  *
  * 注意事项：
  * - 数据解析任务剥离在中断之外（利用 FreeRTOS 队列），不会阻塞 UART 的底层接收。
  ******************************************************************************
  */

#include "sr04_uart.h"
#include "usart.h"
#include "cmsis_os.h" // 引入 FreeRTOS
#include <string.h>
#include <math.h>

// --- 宏定义 ---
#define SR04_FILTER_ALPHA 0.2f 
#define FILTER_ALPHA_SLOW  0.2f
#define FILTER_ALPHA_FAST  0.7f
#define MAX_JUMP_THRESHOLD  40.0f
#define GLITCH_CONFIRM_CNT  1

// --- 变量定义 ---

// 接收缓冲区 (DMA用)
uint8_t sr04_dma_buffer[SR04_RX_BUF_SIZE];

// 引用队列 (确保在 freertos.c 或 main.c 中定义了它)
extern osMessageQueueId_t Queue_SR04DataHandle;

// 内部静态变量 (加上 static 防止外部直接访问)
static float sr04_distances[SR04_SENSOR_COUNT] = {250.0f, 250.0f, 250.0f, 250.0f};
static float last_distances[SR04_SENSOR_COUNT] = {250.0f, 250.0f, 250.0f, 250.0f};
static uint8_t glitch_counts[SR04_SENSOR_COUNT] = {0};
static uint8_t is_initialized[SR04_SENSOR_COUNT] = {0};

// --- 内部函数声明 ---
static float SR04_Filter(float current, float last);
static void SR04_Secure_Update(uint8_t idx, float raw_val);


/**
 * @brief 初始化
 */
void SR04_UART_Init(void)
{
    // 初始化变量
    for(int i=0; i<SR04_SENSOR_COUNT; i++) {
        sr04_distances[i] = 250.0f;
        last_distances[i] = 250.0f;
        is_initialized[i] = 0;
        glitch_counts[i] = 0;
    }

    // 启动 DMA + IDLE 接收
    // 使用 HAL_UARTEx_ReceiveToIdle_DMA 替代旧的 HAL_UART_Receive_DMA
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, sr04_dma_buffer, SR04_RX_BUF_SIZE);
}

/**
 * @brief 中断回调 (在 main.c 的 HAL_UARTEx_RxEventCallback 中被调用)
 */
void SR04_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART6)
    {
        // 1. 准备消息
        UartRxMsg_t msg; 
        // 防止溢出
        msg.len = (Size > SR04_RX_BUF_SIZE) ? SR04_RX_BUF_SIZE : Size;
        memcpy(msg.data, sr04_dma_buffer, msg.len);

        // 2. 发送给队列 (不等待，防止在中断中阻塞)
        if (Queue_SR04DataHandle != NULL) {
            osMessageQueuePut(Queue_SR04DataHandle, &msg, 0, 0);
        }

        // 3. 重启接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, sr04_dma_buffer, SR04_RX_BUF_SIZE);
    }
}

/**
 * @brief 数据处理函数 (在任务中调用)
 * @note  注意：这里不能是 static，因为要在 freertos.c 中调用
 */
void SR04_Process_Data(uint8_t *buf, uint16_t len)
{
    // 协议格式: CB 55 04 AA BB CC DD
    if (len < 7) return;

    for (uint16_t i = 0; i <= len - 7; i++)
    {
        if (buf[i] == 0xCB && buf[i+1] == 0x55 && buf[i+2] == 0x04)
        {
            float raw[4];
            raw[0] = (float)buf[i+3];
            raw[1] = (float)buf[i+4];
            raw[2] = (float)buf[i+5];
            raw[3] = (float)buf[i+6];

            for(int k=0; k<4; k++) {
                SR04_Secure_Update(k, raw[k]);
            }
            
            // 跳过已处理的包
            i += 6; 
        }
    }
}

/**
 * @brief 线程安全的获取距离函数
 */
float SR04_GetDistance(uint8_t index)
{
    if (index >= SR04_SENSOR_COUNT) return 0.0f;
    return sr04_distances[index];
}

// ============================================================
//                 内部静态辅助函数
// ============================================================

static float SR04_Filter(float current, float last)
{
    if (current <= 30.0f || current > 250.0f) {
        return last; 
    }

    if (current < last) {
        return last + FILTER_ALPHA_FAST * (current - last);
    } else {
        return last + FILTER_ALPHA_SLOW * (current - last);
    }
}

static void SR04_Secure_Update(uint8_t idx, float raw_val)
{
    // 1. 基础物理量程限制 (过滤掉传感器本身的盲区和超量程死数据)
    // 假设超声波盲区是 30cm，最大有效距离 250cm
    if (raw_val < 30.0f) raw_val = 30.0f;
    if (raw_val > 250.0f) raw_val = 250.0f;

    // 2. 初始化第一帧数据
    if (is_initialized[idx] == 0) {
        last_distances[idx] = raw_val;
        sr04_distances[idx] = raw_val;
        is_initialized[idx] = 1;
        return;
    }

    // 3. 突变(毛刺)拦截逻辑
    float diff = fabsf(raw_val - last_distances[idx]);

    // 如果相邻两帧数据跳变超过了 40cm (比如从 150 突然变成 10)
    if (diff > MAX_JUMP_THRESHOLD) 
    {
        glitch_counts[idx]++;
        
        // 如果是第一次发生这么大的跳变，我们怀疑它是超声波的杂波(毛刺)，直接丢弃这一帧
        if (glitch_counts[idx] <= GLITCH_CONFIRM_CNT) {
            return; 
        } 
        // 如果连续 2 帧都是这个突变值，说明这不是杂波，而是真的冲出来一个障碍物！立刻接受！
        else {
            glitch_counts[idx] = 0;
        }
    } 
    else 
    {
        // 数据变化在 40cm 以内，属于正常的移动，计数器清零
        glitch_counts[idx] = 0;
    }

    // 4. 直接更新数值，零延迟！
    // 抛弃了以前的 alpha 平滑，现在的雷达数据将做到所见即所得
    last_distances[idx] = raw_val;
    sr04_distances[idx] = raw_val;
}