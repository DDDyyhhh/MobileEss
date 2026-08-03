/**
 ******************************************************************************
 * @file           : app_SerialTX.c
 * @brief          : Shared TX task for UART1/UART2 (debug and local links)
 ******************************************************************************
 */

#include "app_context.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "serial_port.h"
#include "chassis_control.h"
#include "odrive_can.h"

extern osMessageQueueId_t Queue_TxDataHandle;
extern osSemaphoreId_t Sem_TxCompleteHandle;
extern osMessageQueueId_t Queue_TxDataWirelessHandle;
extern osSemaphoreId_t Sem_TxCompleteWirelessHandle;
extern osMessageQueueId_t Queue_CanTxHandle;
void StartTask_SerialTx(void *argument)
{
  UartTxMsg_t msg;
  UART_HandleTypeDef *target_huart = NULL;

  for (;;)
  {
    if (osMessageQueueGet(Queue_TxDataHandle, &msg, NULL, osWaitForever) == osOK)
    {
      switch (msg.port_id)
      {
      case SERIAL_PORT_1:
        target_huart = &huart1;
        break;
      case SERIAL_PORT_2:
        target_huart = &huart2;
        break;
      default:
        target_huart = NULL;
        break;
      }

      if (target_huart != NULL)
      {
        if (HAL_UART_Transmit_DMA(target_huart, msg.buffer, msg.len) == HAL_OK)
        {
          (void)osSemaphoreAcquire(Sem_TxCompleteHandle, 100);
        }
      }
    }
  }
}

void StartTask_WirelessTx(void *argument)
{
  UartTxMsg_t msg;

  for (;;)
  {
    if (osMessageQueueGet(Queue_TxDataWirelessHandle, &msg, NULL, osWaitForever) == osOK)
    {
      if (msg.port_id != SERIAL_PORT_3)
      {
        continue;
      }

      if (HAL_UART_Transmit_DMA(&huart3, msg.buffer, msg.len) == HAL_OK)
      {
        (void)osSemaphoreAcquire(Sem_TxCompleteWirelessHandle, 100);
      }
    }
  }
}

void StartTask_CANTx(void *argument)
{
    CAN_TxMsg_t msg;
    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox;
    osStatus_t status;

    for (;;)
    {
        // 1. 永远等待队列里的消息 (不占CPU)
        status = osMessageQueueGet(Queue_CanTxHandle, &msg, NULL, osWaitForever);
        
        if (status == osOK)
        {
            // 2. 组装硬件帧头
            tx_header.StdId = msg.stdId & 0x7FF;
            tx_header.IDE   = CAN_ID_STD;
            
            // 【核心修复】：根据队列里带的 rtr 标志位，正确设置帧类型
            tx_header.RTR   = (msg.rtr == 1) ? CAN_RTR_REMOTE : CAN_RTR_DATA;
            
            tx_header.DLC   = msg.dlc;
            tx_header.TransmitGlobalTime = DISABLE;

            // 3. 硬件邮箱管理 (防止硬件溢出)
            uint32_t start_tick = HAL_GetTick();
            while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) 
            {
                osDelay(1); // 释放 CPU，给硬件1ms时间去物理线上发报文
                
                // 超时 5ms 就放弃这一帧，去处理队列里的下一帧
                // (发生这种情况通常是因为 CAN 的 CAN_H/L 线没接，或者总线路上没有活着的节点)
                if (HAL_GetTick() - start_tick > 5) {
                    goto NEXT_FRAME; 
                }
            }

            // 4. 正式写入 STM32 硬件发送邮箱
            // (注意：如果是 RTR 帧，HAL 库会忽略 data 指针，所以传 msg.data 没问题)
            HAL_CAN_AddTxMessage(&hcan1, &tx_header, msg.data, &mailbox);
            
NEXT_FRAME:
            continue;
        }
    }
}