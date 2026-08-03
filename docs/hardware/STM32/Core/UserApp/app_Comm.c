/**
 ******************************************************************************
 * @file           : app_Comm.c
 * @author         : [Your Name/Team]
 * @version        : V1.1
 * @date           : 2026-03-11
 * @brief          : 无线通信接收任务，负责接收并解析遥控器下发的数据包
 *
 * @note           任务优先级: 较高 (Above Normal)
 *
 * 主要功能：
 * 1. 依赖 UART DMA + IDLE 中断，通过消息队列 (Queue_RxDataHandle) 异步接收数据
 * 2. 帧头帧尾协议解析：摇杆数据、按键/旋钮状态、动态配对数据
 * 3. 通信链路监控：2秒内未收到合法数据包将触发超时机制，重启底层 DMA 接收
 *
 * 注意事项：
 * - 数据解析采用滑动窗口/逐字节扫描，防止粘包或断帧导致的数据错位。
 * - DMA 超时重启时，执行了 Dummy Read 和错误标志位清除，防止 UART 硬件 ORE 死锁。
 ******************************************************************************
 */
#include "app_context.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "serial_port.h"
#include "pairing_receiver.h"
#include "lifter.h"
#include "chassis_control.h"
extern uint8_t rx_dma_buffer[RX_BUF_SIZE];
extern osMessageQueueId_t Queue_RxDataHandle;

/* USER CODE BEGIN Header_StartTask_Comm */
/**
 * @brief Function implementing the Task_Comm thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_Comm */
void StartTask_Comm(void *argument)
{
  /* USER CODE BEGIN StartTask_Comm */

  UartRxMsg_t msg;
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_dma_buffer, RX_BUF_SIZE);
  // 初始化配对逻辑
  // pairing_receiver_init();
  /* Infinite loop */
  for (;;)
  {
    // 阻塞等待数据
    if (osMessageQueueGet(Queue_RxDataHandle, &msg, NULL, 2000) == osOK)
    {
      // --- 这里放入你原来的解析逻辑 ---
      // 注意：现在是对 msg.data 进行操作，而不是 rx_buffer

      // 遍历接收到的数据包 (保留你原来的 for 循环扫描逻辑)
      for (uint16_t i = 0; i < msg.len; i++)
      {
        // 1. 遥控器数据 (0xAB ... 0xBA)
        if (msg.data[i] == 0xAB && (i + 9 < msg.len) && msg.data[i + 9] == 0xBA)
        {
          g_app.last_receive_timestamp = HAL_GetTick();
          g_app.communication_connected = 1;
          g_app.Joystick[0] = ((msg.data[i + 1] << 8) | msg.data[i + 2]) & 0x0FFF;
          g_app.Joystick[1] = ((msg.data[i + 3] << 8) | msg.data[i + 4]) & 0x0FFF;
          g_app.Joystick[2] = ((msg.data[i + 5] << 8) | msg.data[i + 6]) & 0x0FFF;
          g_app.Joystick[3] = ((msg.data[i + 7] << 8) | msg.data[i + 8]) & 0x0FFF;
        }
        // 2. 按键/开关数据 (0xCC ... 0xDD)
        else if (msg.data[i] == 0xCC && (i + 14 < msg.len) && msg.data[i + 14] == 0xDD)
        {
          for (uint8_t k = 0; k < 8; k++)
          {
            // 【修正】必须加上 i
            g_app.key[k] = msg.data[i + 1 + k];
          }

          if (Ctrl_Mode == 0)
          {
            g_app.knob_gear = msg.data[i + 9];
            g_app.knob_blade = msg.data[i + 10];
            g_app.knob_height = msg.data[i + 11];
          }
          else
          {
            // 如果是 API 模式，仅仅更新档位 (gear)，忽略遥控器发来的刀盘和推杆状态
            g_app.knob_gear = msg.data[i + 9];
          }

          // ============================================================
          // <<<---  解析触摸屏设置包 --->>>
          // ============================================================
          uint8_t config_flags = msg.data[i + 12];

          // 位提取 (Unpacking)
          g_app.cfg_ultrasonic_en = (config_flags & (1 << 0)) ? 1 : 0;
          g_app.cfg_collision_en = (config_flags & (1 << 1)) ? 1 : 0;
          g_app.cfg_reverse_limit_en = (config_flags & (1 << 2)) ? 1 : 0;
          g_app.cfg_tilt_sensor_en = (config_flags & (1 << 3)) ? 1 : 0;
          g_app.cfg_lawn_protect_en  = (config_flags & (1 << 6)) ? 1 : 0;

          // --- B. 触发动作类：只负责置 1，由 app_Motion 执行完后清 0 ---
          // 逻辑：只有当收到 1 时才改变变量，收到 0 时不操作
          if (config_flags & (1 << 4))
          {
            g_app.trigger_blade_init = 1;
          }
          if (config_flags & (1 << 5))
          {
            g_app.trigger_cruise_req = 1;
          }
          // 提取操作模式
          g_app.cfg_op_mode = msg.data[i + 13];

          i += 14; // 优化：跳过这个包
        }
        // 3. 配对数据 (0xAA ... 0xBB)
        // 注意：这里需要传递指针，或者修改 pairing_handle 接收固定长度
        // else if (msg.len >= 3 && msg.data[0] == 0xAA && msg.data[msg.len - 1] == 0xBB)
        // {
        //   pairing_receiver_handle_message(msg.data, msg.len);
        // }

        // 检查当前扫描到的字节 msg.data[i]，而不是 msg.data[0]
        else if (msg.data[i] == 0xAA)
        {
          // 因为配对包长度不固定(3~6字节)，我们需要向后搜索包尾 0xBB
          // 限制搜索范围为 10 字节，防止误判
          for (uint16_t j = 1; j < 10 && (i + j < msg.len); j++)
          {
            if (msg.data[i + j] == 0xBB)
            {
              // 找到了完整包！
              // 1. 传入当前包的起始地址: &msg.data[i] (关键！)
              // 2. 传入当前包的实际长度: j + 1
              Serial_Printf(SERIAL_PORT_2, "[DEBUG] Valid Pairing Packet Found! Len: %d\r\n", j + 1);
              pairing_receiver_handle_message(&msg.data[i], j + 1);

              i += j; // 优化：跳过这个包的长度，继续扫描下一个
              break;  // 退出内层搜索
            }
          }
        }
      }
    }
    else
    {
      // --- B. 队列超时 (2秒无数据) ---

      // 只有在之前连接过的情况下，才认为是故障并尝试重启
      // (防止刚上电遥控器没开时一直重启)
      // 或者干脆无条件重启，反正重启一下也没坏处

      Serial_Printf(SERIAL_PORT_2, "[Comm] RX Timeout! Restarting DMA...\r\n");

      // 1. 强制停止接收
      HAL_UART_AbortReceive(&huart3);
      HAL_UART_DMAStop(&huart3);

      // 2. 清除错误标志 (ORE/NE/FE)
      __HAL_UART_CLEAR_OREFLAG(&huart3);
      __HAL_UART_CLEAR_NEFLAG(&huart3);
      __HAL_UART_CLEAR_FEFLAG(&huart3);

      // 读取数据寄存器以清除某些标志位 (Dummy Read)
      volatile uint32_t tmp = huart3.Instance->DR;
      (void)tmp;

      // 3. 重新启动 DMA 接收
      // 注意：控制板用的是 ReceiveToIdle，不需要像遥控器那样重置 last_dma_pos
      HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_dma_buffer, RX_BUF_SIZE);
    }
  }
  /* USER CODE END StartTask_Comm */
}