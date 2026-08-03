#ifndef MODBUS_RTU_BUS_H
#define MODBUS_RTU_BUS_H

#include <stdint.h>
#include <stdbool.h>

// Modbus总线状态机
typedef enum {
    MODBUS_STATE_IDLE,      // 空闲，可以发送新命令
    MODBUS_STATE_WAITING    // 已发送命令，等待响应
} ModbusState;

// 定义一个回调函数指针类型，当收到有效Modbus响应时，总线会调用它
// 参数: 响应帧的数据部分(PDU, 已去除地址、功能码和CRC), PDU的长度
typedef void (*ModbusRxCallback)(uint8_t* pdu, uint16_t pdu_len);

/**
 * @brief 初始化Modbus总线
 * @param callback 当收到有效响应时要调用的函数
 */
void ModbusRtuBus_Init(ModbusRxCallback callback);

/**
 * @brief Modbus总线的周期处理函数，必须在主循环中不断调用
 * @note  此函数处理数据接收和超时判断
 */
void ModbusRtuBus_Process(void);

/**
 * @brief 发送一个Modbus请求 (目前只实现功能码0x03)
 * @param devAddr  从机设备地址
 * @param startAddr 要读取的起始寄存器地址
 * @param numRegs  要读取的寄存器数量
 * @param timeout  本次通信的超时时间 (毫秒)
 * @return bool    true如果请求成功发出，false如果总线正忙
 */
bool ModbusRtuBus_ReadHoldingRegisters(uint8_t devAddr, uint16_t startAddr, uint16_t numRegs, uint32_t timeout);

#endif // MODBUS_RTU_BUS_H

