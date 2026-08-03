#ifndef __HEARTBEAT_H
#define __HEARTBEAT_H

#include "main.h"

// --- 配置参数 ---
#define HEARTBEAT_TIMEOUT_MS    5000  // 超过1000ms没收到数据，视为断连（触发急停）
#define SIGNAL_LEVEL_4_MS       2000   // 原 100
#define SIGNAL_LEVEL_3_MS       2500   // 原 200
#define SIGNAL_LEVEL_2_MS       3500  // 原 500
#define SIGNAL_LEVEL_1_MS       4000  // 原 900
// > 900ms -> 0格信号

typedef enum {
    LINK_LOST = 0,
    LINK_ALIVE = 1
} LinkState_t;

typedef struct {
    uint32_t last_packet_tick;  // 上一次收到合法包的时间戳
    LinkState_t state;          // 当前连接状态
    uint8_t signal_level;       // 信号等级 (0-4)
    uint32_t lost_count;        // 累计丢包/断连次数 (用于调试)
} Heartbeat_Handle_t;

// 函数声明
void Heartbeat_Init(Heartbeat_Handle_t *hb);
void Heartbeat_Feed(Heartbeat_Handle_t *hb);
void Heartbeat_Check(Heartbeat_Handle_t *hb);
uint8_t Heartbeat_GetSignalLevel(Heartbeat_Handle_t *hb);

#endif