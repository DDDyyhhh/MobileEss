/**
  ******************************************************************************
  * @file           : app_Report.c
  * @author         : [Your Name/Team]
  * @version        : V1.1
  * @date           : 2026-03-11
  * @brief          : 系统状态上报与指示任务，负责 LED 交互与 telemetry 遥测回传
  *
  * @note           任务优先级: 普通 (Normal)
  * 
  * 主要功能：
  * 1. 周期性 (10Hz / 100ms) 轮询全局状态 `g_app.g_system_report_state`
  * 2. 控制 RGB 状态指示灯 (WS2812 或 PWM LED)：
  *    - 红闪：物理碰撞故障
  *    - 蓝闪：急停按下 / 通信连接中
  *    - 红慢闪：通信丢失
  *    - 绿常亮：正常手动模式
  *    - 白/青/粉：巡航状态
  * 3. 调用 `receiver_resport()` 将电量、锁定状态、档位等信息回传给遥控器
  *
  * 注意事项：
  * - 作为一个后台表现层任务，大量使用了 `osDelay`，不会抢占核心运动控制的 CPU 资源
  * - 数据回传必须依赖底层线程安全的串口发送机制 (`Queue_TxDataHandle`)，禁止直接调用 HAL_UART_Transmit
  ******************************************************************************
  */

#include "app_context.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "serial_port.h"
#include "tricolor_led.h"
#include "cruise_control.h"
#include "pairing_receiver.h"
#include "connet_check.h"
#include "usart.h" 
#include <stdlib.h> 
#include <string.h>
#include "A28.h"
//  引用 CubeMX 生成的句柄 >>>
extern osMutexId_t ODrivePort1MutexHandle;
extern osMutexId_t ODrivePort2MutexHandle;
extern osSemaphoreId_t ODrivePort1ResponseSemHandle;
extern osSemaphoreId_t ODrivePort2ResponseSemHandle;

//  引用中断填充的全局缓冲区 >>>
extern uint8_t g_odrive_port1_rx_buffer[64];
extern volatile uint16_t g_odrive_port1_rx_len;
extern uint8_t g_odrive_port2_rx_buffer[64];
extern volatile uint16_t g_odrive_port2_rx_len;

/**
 * @brief  【线程安全】向ODrive发送请求并阻塞等待响应
 * @param  port_id: 目标串口 (SERIAL_PORT_1 或 SERIAL_PORT_2)
 * @param  cmd:     要发送的指令字符串
 * @param  timeout_ms: 等待响应的超时时间
 * @return float:   从响应中解析出的浮点数值；如果超时或失败，返回 -1.0f
 */
static float ODrive_Request_Response(SerialPort_ID port_id, const char* cmd, uint32_t timeout_ms)
{
    osMutexId_t target_mutex = NULL;
    osSemaphoreId_t target_sem = NULL;
    uint8_t* target_rx_buf = NULL;
    volatile uint16_t* target_rx_len_ptr = NULL;

    if (port_id == SERIAL_PORT_1) {
        target_mutex = ODrivePort1MutexHandle;
        target_sem = ODrivePort1ResponseSemHandle;
        target_rx_buf = g_odrive_port1_rx_buffer;
        target_rx_len_ptr = &g_odrive_port1_rx_len;
    } else if (port_id == SERIAL_PORT_2) {
        target_mutex = ODrivePort2MutexHandle;
        target_sem = ODrivePort2ResponseSemHandle;
        target_rx_buf = g_odrive_port2_rx_buffer;
        target_rx_len_ptr = &g_odrive_port2_rx_len;
    } else {
        return -1.0f;
    }

    float result = -1.0f;

    if (osMutexAcquire(target_mutex, 10) == osOK)
    {
        *target_rx_len_ptr = 0; // 清空上次接收
        
        Serial_SendData(port_id, (uint8_t*)cmd, strlen(cmd));

        if (osSemaphoreAcquire(target_sem, timeout_ms) == osOK)
        {
            char* line = strtok((char*)target_rx_buf, "\n\r");
            while(line != NULL)
            {
                if ((*line >= '0' && *line <= '9') || *line == '-') {
                    result = atof(line);
                    break; 
                }
                line = strtok(NULL, "\n\r");
            }
        }
        
        osMutexRelease(target_mutex);
    }
    return result;
}


// 辅助函数：根据 ID 获取句柄 (为了在当前文件进行接收操作)
static UART_HandleTypeDef* Get_UART_Handle_Local(SerialPort_ID port_id)
{
    switch(port_id) {
        case SERIAL_PORT_1: return &huart1;
        case SERIAL_PORT_2: return &huart2;
        default: return NULL;
    }
}

// =================================================================
// <<<---  同步阻塞发送 + 零延时极速抓取 --->>>
// =================================================================
static float ODrive_Read_Temp_Direct(UART_HandleTypeDef* huart, uint8_t axis) 
{
    if (huart == NULL) return -1.0f;

    char cmd[64];
    sprintf(cmd, "r axis%d.fet_thermistor.temperature\n", axis);
    

    // 1. 等待之前的通信任务(如底盘指令)让出串口
    uint32_t wait_start = HAL_GetTick();
    while (huart->gState != HAL_UART_STATE_READY) 
    {
        if (HAL_GetTick() - wait_start > 10) return -1.0f; 
        osDelay(1);
    }

    // 2. 清洗硬件：把可能存在的错误标志和上一轮残留的字节全部干掉
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_FLUSH_DRREGISTER(huart);

    // 3. 【核心修正】使用阻塞发送，抛弃队列！
    // 这样发完的瞬间，代码立刻往下走，不会漏掉 ODrive 的极速回复
    HAL_UART_Transmit(huart, (uint8_t*)cmd, strlen(cmd), 10);

    // 4. 立即开始读取，绝对不能加 osDelay！
    uint8_t line_buf[64] = {0}; 
    uint8_t idx = 0;
    uint32_t rx_start = HAL_GetTick();
    
    // 给 50ms 超时，足够读取 ODrive 吐出来的回显和真值
    while (HAL_GetTick() - rx_start < 50) 
    { 
        uint8_t ch;
        // 极速轮询单字节
        HAL_StatusTypeDef status = HAL_UART_Receive(huart, &ch, 1, 2);

        if (status == HAL_OK) 
        {
            if (ch == '\n' || ch == '\r') 
            {
                if (idx > 0) 
                {
                    line_buf[idx] = '\0'; 
                    // 找到了数字开头 (兼容 "36" 或 "-5")，这就是我们要的温度
                    if ((line_buf[0] >= '0' && line_buf[0] <= '9') || line_buf[0] == '-')
                    {
                        return atof((char*)line_buf); 
                    }
                    // 如果是回显，清空继续找
                    idx = 0;
                    memset(line_buf, 0, sizeof(line_buf));
                }
            }
            else if (idx < 63) 
            {
                // 只收录有效 ASCII 字符，过滤乱码
                if (ch >= 32 && ch <= 126) line_buf[idx++] = ch;
            }
        }
        else if (status == HAL_ERROR || status == HAL_BUSY)
        {
            // 如果万一又发生了溢出，立刻强行清除错误并继续读！
            __HAL_UART_CLEAR_OREFLAG(huart);
            volatile uint32_t tmp = huart->Instance->DR; // 吞掉坏数据
            (void)tmp;
        }
    }
    
    return -1.0f; // 彻底超时
}

/* USER CODE BEGIN Header_StartTask_Report */
/**
 * @brief Function implementing the Task_Report thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_Report */
void StartTask_Report(void *argument)
{
  /* USER CODE BEGIN StartTask_Report */
  // 初始延时，等待系统稳定
  osDelay(1000);
  static uint32_t last_temp_query_time = 0;
  /* Infinite loop */
  for (;;)
  {
    // 10Hz 刷新率 (100ms)
    osDelay(100);

    // ============================================================
    // 1. LED 状态管理 (优先级排布)
    // ============================================================
    
    // 优先级 1：通信丢失 (无论是否碰撞，只要失联就是最高危机)
    //if (!CheckCommunicationStatus() || g_app.g_system_report_state == REPORT_FAULT_COMM_LOST)
   // {
        // 通信丢失：红灯慢闪
        // RGB_LED_Blink(LED_1, RGB_COLOR_RED, 500);
        // RGB_LED_Blink(LED_2, RGB_COLOR_RED, 500);
   // }
    // 优先级 2：物理安全锁存 (急停/碰撞/倾翻等)
    //else
     if (g_app.g_collision_latched)
    {
        if (g_app.g_system_report_state == REPORT_FAULT_EMERGENCY_BTN)
        {
            RGB_LED_Blink(LED_1, RGB_COLOR_BLUE, 200); // 急停：蓝灯快闪
            RGB_LED_Blink(LED_2, RGB_COLOR_BLUE, 200);
        }
        else if (g_app.g_system_report_state == REPORT_FAULT_COLLISION)
        {
            RGB_LED_Blink(LED_1, RGB_COLOR_RED, 200); // 碰撞：红灯快闪
            RGB_LED_Blink(LED_2, RGB_COLOR_RED, 200);
        }
        else 
        {
            // 其他未定义的严重故障：默认红灯快闪警告
            RGB_LED_Blink(LED_1, RGB_COLOR_RED, 200);
            RGB_LED_Blink(LED_2, RGB_COLOR_RED, 200);
        }
    }
    // 优先级 3：巡航模式
    else if (Cruise_GetState() != CRUISE_OFF)
    {
        // 巡航：白灯常亮 (或设置你喜欢的颜色)
        RGB_LED_SetColor(LED_1, RGB_COLOR_WHITE);
        RGB_LED_SetColor(LED_2, RGB_COLOR_WHITE);
    }
    // 优先级 4：正常手动模式
    else
    {
        // 正常：绿灯常亮
        RGB_LED_SetColor(LED_1, RGB_COLOR_GREEN);
        RGB_LED_SetColor(LED_2, RGB_COLOR_GREEN);
    }

    // 执行 LED 闪烁逻辑 (计算时间差)
    RGB_LED_ProcessBlink();

    // ============================================================
    // <<<--- 每 5 秒轮询一次 ODrive 温度 --->>>
    // ============================================================
    // if (HAL_GetTick() - last_temp_query_time >= 5000) 
    // {
    //     last_temp_query_time = HAL_GetTick();
    //     char cmd[64];
    //     float temp;

    //     // --- 读取右轮 ODrive (UART1 / PORT_1) ---
    //     sprintf(cmd, "r axis%d.fet_thermistor.temperature\n", 0);
    //     temp = ODrive_Request_Response(SERIAL_PORT_1, cmd, 50);
    //     if (temp > -0.5f) g_app.odrive_temps[0] = (uint8_t)temp;

    //     sprintf(cmd, "r axis%d.fet_thermistor.temperature\n", 1);
    //     temp = ODrive_Request_Response(SERIAL_PORT_1, cmd, 50);
    //     if (temp > -0.5f) g_app.odrive_temps[1] = (uint8_t)temp;

    //     // --- 读取左轮 ODrive (UART2 / PORT_2) ---
    //     sprintf(cmd, "r axis%d.fet_thermistor.temperature\n", 0);
    //     temp = ODrive_Request_Response(SERIAL_PORT_2, cmd, 50);
    //     if (temp > -0.5f) g_app.odrive_temps[2] = (uint8_t)temp;

    //     sprintf(cmd, "r axis%d.fet_thermistor.temperature\n", 1);
    //     temp = ODrive_Request_Response(SERIAL_PORT_2, cmd, 50);
    //     if (temp > -0.5f) g_app.odrive_temps[3] = (uint8_t)temp;
    // }
    
    // ============================================================

    // 2. 上报数据给遥控器
    // 确保 receiver_resport 内部使用的是 Serial_Send_Data
    receiver_resport();
    // A28_Apply_Config(); // 确保 A28 的配置被应用，保持通信稳定
  }
  /* USER CODE END StartTask_Report */
}