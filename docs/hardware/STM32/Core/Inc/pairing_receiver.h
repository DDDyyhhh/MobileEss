// pairing_receiver.h
#ifndef PAIRING_RECEIVER_H
#define PAIRING_RECEIVER_H

#include "pairing_common.h"

// 初始化接收机配对模块
void pairing_receiver_init(void);

// 处理接收到的消息
void pairing_receiver_handle_message(uint8_t* data, uint16_t length);

// 获取当前配对状态
pairing_state_t pairing_receiver_get_state(void);

// 处理配对状态机（在主循环中调用）
void pairing_receiver_process(void);
void pairing_receiver_task(void);
void receiver_resport(void);
void TestUARTTransmit(void);

// 进入无线模块透传模式，供其他模块调用
void RF_Module_Set_Trans_Mode(void);

/**
 * @brief 定期打包并发送超声波传感器数据
 * @note  这个函数应该在一个周期性任务中被调用，例如每100ms一次
 */
void Ultrasonic_Report_Task(uint8_t collision_status);
#endif