// pairing_common.h
#ifndef PAIRING_COMMON_H
#define PAIRING_COMMON_H

#include <stdint.h>
#include <stdbool.h>

// 默认地址（出厂设置）
#define DEFAULT_ADDRESS 0x0000

// 配对协议消息类型
typedef enum {
    PAIRING_REQ = 0xFA,    // 配对请求
    PAIRING_ACK = 0xFB,    // 配对确认
    PAIRING_ADDR = 0xFC,   // 地址交换
    PAIRING_CONFIRM = 0xFD, // 配对确认完成
    PAIRING_TEST = 0xFE,   // 测试通信
    PAIRING_TEST_OK = 0xFF // 测试成功响应
} pairing_cmd_t;

// 配对状态
typedef enum {
    PAIRING_IDLE,
    PAIRING_STARTED,
    PAIRING_ADDR_SENT,
    PAIRING_ADDR_RECEIVED,
    PAIRING_CONFIG_SENT,
    PAIRING_SUCCESS,
    PAIRING_FAILED
} pairing_state_t;


#endif // PAIRING_COMMON_H