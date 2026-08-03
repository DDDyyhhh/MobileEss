// communication.h 或添加到现有头文件
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "stm32f4xx_hal.h"

// 通信状态检测
extern volatile uint32_t last_receive_timestamp;
extern volatile uint8_t communication_connected;

#define COMMUNICATION_TIMEOUT_MS 300

// 函数声明
uint8_t CheckCommunicationStatus(void);
uint32_t GetLastReceiveTimeAgo(void);

void Suspend_Heartbeat_Check(uint8_t suspend);
#endif



// // communication.h 或添加到现有头文件
// #ifndef COMMUNICATION_H
// #define COMMUNICATION_H

// #include "stm32f4xx_hal.h"

// // 通信状态检测
// extern volatile uint32_t last_receive_timestamp;
// extern volatile uint8_t communication_connected;

// // #define COMMUNICATION_TIMEOUT_MS 2500

// // 【新增】定义通信状态枚举
// typedef enum {
//     CONN_OK,            // 0: 通信正常，一切良好
//     CONN_GRACE_PERIOD,  // 1: 通信丢失，但处于1秒的“容错期”内
//     CONN_TIMEOUT,        // 2: 通信彻底超时，必须停止
//     CONN_COLLISION
// } ConnectionStatus_t;

// // 【重命名】函数声明，新的函数名更能体现其功能
// ConnectionStatus_t GetCommunicationState(void);

// // 函数声明
// uint8_t CheckCommunicationStatus(void);
// uint32_t GetLastReceiveTimeAgo(void);



// #endif
