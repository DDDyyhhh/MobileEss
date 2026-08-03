/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pairing_receiver.h"
#include "E34.h"
#include "serial_port.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "rs485.h"
#include "AXEBMS_RS485.h"
#include "modbus_rtu_bus.h"
#include "tricolor_led.h"
#include "lifter.h"
#include "mower_config.h"
#include "sr04_uart.h"
#include "cruise_control.h"
#include "heartbeat.h"
#include "bsp_imu_solver.h"
#include "icm42688.h"
#include "car_Control.h"
#include "BladeMotor.h"
#include "connet_check.h"
#include "app_context.h"
#include "chassis_control.h"
#include "odrive_can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Motion */
osThreadId_t Task_MotionHandle;
const osThreadAttr_t Task_Motion_attributes = {
  .name = "Task_Motion",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task_Comm */
osThreadId_t Task_CommHandle;
const osThreadAttr_t Task_Comm_attributes = {
  .name = "Task_Comm",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task_BMS_Sensor */
osThreadId_t Task_BMS_SensorHandle;
const osThreadAttr_t Task_BMS_Sensor_attributes = {
  .name = "Task_BMS_Sensor",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Report */
osThreadId_t Task_ReportHandle;
const osThreadAttr_t Task_Report_attributes = {
  .name = "Task_Report",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_SerialTx */
osThreadId_t Task_SerialTxHandle;
const osThreadAttr_t Task_SerialTx_attributes = {
  .name = "Task_SerialTx",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_WirelessTx */
osThreadId_t Task_WirelessTxHandle;
const osThreadAttr_t Task_WirelessTx_attributes = {
  .name = "Task_WirelessTx",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t Task_CANTXHandle;
const osThreadAttr_t Task_CANTX_attributes = {
  .name = "Task_CANTx",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
osThreadId_t Task_CANRXHandle;
const osThreadAttr_t Task_CANRX_attributes = {
  .name = "Task_CANRx",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Queue_RxData */
osMessageQueueId_t Queue_RxDataHandle;
const osMessageQueueAttr_t Queue_RxData_attributes = {
  .name = "Queue_RxData"
};
/* Definitions for Queue_TxData */
osMessageQueueId_t Queue_TxDataHandle;
const osMessageQueueAttr_t Queue_TxData_attributes = {
  .name = "Queue_TxData"
};
/* Definitions for Queue_TxDataWireless */
osMessageQueueId_t Queue_TxDataWirelessHandle;
const osMessageQueueAttr_t Queue_TxDataWireless_attributes = {
  .name = "Queue_TxDataWireless"
};
/* Definitions for Queue_SR04Data */
osMessageQueueId_t Queue_SR04DataHandle;
const osMessageQueueAttr_t Queue_SR04Data_attributes = {
  .name = "Queue_SR04Data"
};
/* Definitions for myMutex_Serial */
osMutexId_t myMutex_SerialHandle;
const osMutexAttr_t myMutex_Serial_attributes = {
  .name = "myMutex_Serial"
};
/* Definitions for ODrivePort1Mutex */
osMutexId_t ODrivePort1MutexHandle;
const osMutexAttr_t ODrivePort1Mutex_attributes = {
  .name = "ODrivePort1Mutex"
};
/* Definitions for ODrivePort2Mutex */
osMutexId_t ODrivePort2MutexHandle;
const osMutexAttr_t ODrivePort2Mutex_attributes = {
  .name = "ODrivePort2Mutex"
};
/* Definitions for Sem_TxComplete */
osSemaphoreId_t Sem_TxCompleteHandle;
const osSemaphoreAttr_t Sem_TxComplete_attributes = {
  .name = "Sem_TxComplete"
};
/* Definitions for Sem_TxCompleteWireless */
osSemaphoreId_t Sem_TxCompleteWirelessHandle;
const osSemaphoreAttr_t Sem_TxCompleteWireless_attributes = {
  .name = "Sem_TxCompleteWireless"
};
/* Definitions for Sem_RS485_Rx */
osSemaphoreId_t Sem_RS485_RxHandle;
const osSemaphoreAttr_t Sem_RS485_Rx_attributes = {
  .name = "Sem_RS485_Rx"
};
/* Definitions for Sem_RS485_Tx */
osSemaphoreId_t Sem_RS485_TxHandle;
const osSemaphoreAttr_t Sem_RS485_Tx_attributes = {
  .name = "Sem_RS485_Tx"
};
/* Definitions for ODrivePort1ResponseSem */
osSemaphoreId_t ODrivePort1ResponseSemHandle;
const osSemaphoreAttr_t ODrivePort1ResponseSem_attributes = {
  .name = "ODrivePort1ResponseSem"
};
/* Definitions for ODrivePort2ResponseSem */
osSemaphoreId_t ODrivePort2ResponseSemHandle;
const osSemaphoreAttr_t ODrivePort2ResponseSem_attributes = {
  .name = "ODrivePort2ResponseSem"
};

osMessageQueueId_t Queue_CanTxHandle;
const osMessageQueueAttr_t Queue_CanTx_attributes = { .name = "Queue_CanTx" };
osMessageQueueId_t Queue_CanRxHandle;
const osMessageQueueAttr_t Queue_CanRx_attributes = { .name = "Queue_CanRx" };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask_Motion(void *argument);
void StartTask_Comm(void *argument);
void StartTask_BMS_Sensor(void *argument);
void StartTask_Report(void *argument);
void StartTask_SerialTx(void *argument);
void StartTask_WirelessTx(void *argument);
void StartTask_CANTx(void *argument);
void StartTask_CANRx(void *argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of myMutex_Serial */
  myMutex_SerialHandle = osMutexNew(&myMutex_Serial_attributes);

  /* creation of ODrivePort1Mutex */
  ODrivePort1MutexHandle = osMutexNew(&ODrivePort1Mutex_attributes);

  /* creation of ODrivePort2Mutex */
  ODrivePort2MutexHandle = osMutexNew(&ODrivePort2Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Sem_TxComplete */
  Sem_TxCompleteHandle = osSemaphoreNew(1, 1, &Sem_TxComplete_attributes);

  /* creation of Sem_TxCompleteWireless */
  Sem_TxCompleteWirelessHandle = osSemaphoreNew(1, 1, &Sem_TxCompleteWireless_attributes);

  /* creation of Sem_RS485_Rx */
  Sem_RS485_RxHandle = osSemaphoreNew(1, 1, &Sem_RS485_Rx_attributes);

  /* creation of Sem_RS485_Tx */
  Sem_RS485_TxHandle = osSemaphoreNew(1, 1, &Sem_RS485_Tx_attributes);

  /* creation of ODrivePort1ResponseSem */
  ODrivePort1ResponseSemHandle = osSemaphoreNew(1, 1, &ODrivePort1ResponseSem_attributes);

  /* creation of ODrivePort2ResponseSem */
  ODrivePort2ResponseSemHandle = osSemaphoreNew(1, 1, &ODrivePort2ResponseSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Queue_RxData */
  Queue_RxDataHandle = osMessageQueueNew (16, 68, &Queue_RxData_attributes);

  /* creation of Queue_TxData */
  Queue_TxDataHandle = osMessageQueueNew (16, 140, &Queue_TxData_attributes);

  /* creation of Queue_TxDataWireless */
  Queue_TxDataWirelessHandle = osMessageQueueNew (16, 140, &Queue_TxDataWireless_attributes);

  /* creation of Queue_SR04Data */
  Queue_SR04DataHandle = osMessageQueueNew (4, 68, &Queue_SR04Data_attributes);

  Queue_CanTxHandle = osMessageQueueNew(256, sizeof(CAN_TxMsg_t), &Queue_CanTx_attributes);
  Queue_CanRxHandle = osMessageQueueNew(128, sizeof(CAN_RxMsg_t), &Queue_CanRx_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Task_Motion */
  Task_MotionHandle = osThreadNew(StartTask_Motion, NULL, &Task_Motion_attributes);

  /* creation of Task_Comm */
  Task_CommHandle = osThreadNew(StartTask_Comm, NULL, &Task_Comm_attributes);

  /* creation of Task_BMS_Sensor */
  Task_BMS_SensorHandle = osThreadNew(StartTask_BMS_Sensor, NULL, &Task_BMS_Sensor_attributes);

  /* creation of Task_Report */
  Task_ReportHandle = osThreadNew(StartTask_Report, NULL, &Task_Report_attributes);

  /* creation of Task_SerialTx */
  Task_SerialTxHandle = osThreadNew(StartTask_SerialTx, NULL, &Task_SerialTx_attributes);

  /* creation of Task_WirelessTx */
  Task_WirelessTxHandle = osThreadNew(StartTask_WirelessTx, NULL, &Task_WirelessTx_attributes);

  /* creation of Task_CANTX */
  Task_CANTXHandle = osThreadNew(StartTask_CANTx, NULL, &Task_CANTX_attributes);

  /* creation of Task_CANRX */
  Task_CANRXHandle = osThreadNew(StartTask_CANRx, NULL, &Task_CANRX_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

