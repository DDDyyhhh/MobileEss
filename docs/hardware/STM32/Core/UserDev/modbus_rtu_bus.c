/**
  ******************************************************************************
  * @file           : modbus_rtu_bus.c
  * @brief          : Modbus RTU 协议主站 (Master) 通信层
  * 
  * 主要功能：
  * 1. 封装了标准的 0x03 (读保持寄存器) 请求帧，并自动附加 CRC16 校验码
  * 2. 维护非阻塞式的通信状态机 (`MODBUS_STATE_WAITING`) 和超时检测
  * 3. 接收到底层数据后验证地址、功能码与 CRC，成功后剥离 PDU 交由回调函数处理
  *
  * 注意事项：
  * - 协议层本身不产生任何延时，它的超时逻辑极度依赖上层的高频轮询 (`ModbusRtuBus_Process`)。
  * - 此模块与物理层解耦，底层收发完全依赖于 `rs485.c`。
  ******************************************************************************
  */
#include "modbus_rtu_bus.h"
#include "rs485.h"        // 依赖底层的RS485驱动
#include "main.h"         // 依赖HAL_GetTick
#include <string.h>

// 内部状态变量
static ModbusState      g_state = MODBUS_STATE_IDLE;
static uint32_t         g_request_sent_time = 0;
static uint32_t         g_transaction_timeout_ms = 0;
static ModbusRxCallback g_rx_callback = NULL;

// 当前正在通信的设备信息 (用于校验响应)
static uint8_t g_current_dev_addr = 0;
static uint8_t g_current_func_code = 0;

// CRC16计算函数
static uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

void ModbusRtuBus_Init(ModbusRxCallback callback) {
    g_state = MODBUS_STATE_IDLE;
    g_rx_callback = callback;
    RS485_Init(); // 初始化底层驱动
}

bool ModbusRtuBus_ReadHoldingRegisters(uint8_t devAddr, uint16_t startAddr, uint16_t numRegs, uint32_t timeout) {
    if (g_state != MODBUS_STATE_IDLE) {
        return false; // 总线忙，无法发送
    }

    uint8_t tx_frame[8];
    tx_frame[0] = devAddr;
    tx_frame[1] = 0x03; // 功能码
    tx_frame[2] = (startAddr >> 8) & 0xFF; // 地址高位
    tx_frame[3] = startAddr & 0xFF;        // 地址低位
    tx_frame[4] = (numRegs >> 8) & 0xFF;   // 数量高位
    tx_frame[5] = numRegs & 0xFF;        // 数量低位

    uint16_t crc = Modbus_CRC16(tx_frame, 6);
    tx_frame[6] = crc & 0xFF;
    tx_frame[7] = (crc >> 8) & 0xFF;

    g_current_dev_addr = devAddr;
    g_current_func_code = 0x03;
    g_transaction_timeout_ms = timeout;

    RS485_SendData(tx_frame, 8);

    g_request_sent_time = HAL_GetTick();
    g_state = MODBUS_STATE_WAITING;
    return true;
}


void ModbusRtuBus_Process(void) {
    // 1. 检查是否有新数据从底层驱动上来
    uint8_t rx_buffer[RS485_RX_BUFFER_SIZE];
    uint16_t rx_len = RS485_ReceiveData(rx_buffer, sizeof(rx_buffer));

    if (rx_len > 0 && g_state == MODBUS_STATE_WAITING) {
        // 收到数据，进行校验
        if (rx_len < 5) { // 最小响应帧长度: 地址+功能码+字节数+CRC(2)
            g_state = MODBUS_STATE_IDLE; // 帧错误，放弃本次通信
            return;
        }

        // 校验CRC
        uint16_t received_crc = (rx_buffer[rx_len - 1] << 8) | rx_buffer[rx_len - 2];
        uint16_t calculated_crc = Modbus_CRC16(rx_buffer, rx_len - 2);

        // 校验地址和功能码
				
        if (calculated_crc == received_crc &&
            rx_buffer[0] == g_current_dev_addr &&
            rx_buffer[1] == g_current_func_code) {
            
            // 响应有效！
            if (g_rx_callback != NULL) {
							 
                // 调用回调，传递PDU部分
                uint8_t byte_count = rx_buffer[2];
                // 再次检查长度是否匹配
                if (byte_count == rx_len - 5) { // PDU长度 = 总长 - 地址 - 功能码 - 字节数 - CRC
                     g_rx_callback(&rx_buffer[3], byte_count);
                }
            }
            g_state = MODBUS_STATE_IDLE; // 通信成功，总线恢复空闲
            return;
        }
    }

    // 2. 检查通信是否超时
    if (g_state == MODBUS_STATE_WAITING) {
        if (HAL_GetTick() - g_request_sent_time > g_transaction_timeout_ms) {
            // 通信超时
            if (g_rx_callback != NULL) {
                g_rx_callback(NULL, 0); // 使用NULL和0长度来通知上层超时
            }
            g_state = MODBUS_STATE_IDLE; // 超时，总线恢复空闲
        }
    }
}