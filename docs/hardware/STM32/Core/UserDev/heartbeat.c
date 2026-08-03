#include "heartbeat.h"

/**
 * @brief 初始化心跳结构体
 */
void Heartbeat_Init(Heartbeat_Handle_t *hb)
{
    hb->last_packet_tick = HAL_GetTick();
    hb->state = LINK_LOST; // 默认未连接，直到收到第一个包
    hb->signal_level = 0;
    hb->lost_count = 0;
}

/**
 * @brief 喂狗：每当收到合法的串口数据包时调用一次
 */
void Heartbeat_Feed(Heartbeat_Handle_t *hb)
{
    uint32_t current_tick = HAL_GetTick();
    uint32_t interval = current_tick - hb->last_packet_tick;
    
    hb->last_packet_tick = current_tick;
    hb->state = LINK_ALIVE;

    // 根据包到达的时间间隔，计算信号质量
    if (interval < SIGNAL_LEVEL_4_MS)      hb->signal_level = 4;
    else if (interval < SIGNAL_LEVEL_3_MS) hb->signal_level = 3;
    else if (interval < SIGNAL_LEVEL_2_MS) hb->signal_level = 2;
    else if (interval < SIGNAL_LEVEL_1_MS) hb->signal_level = 1;
    else                                   hb->signal_level = 0;
}

/**
 * @brief 检查：在主循环中调用，判断是否超时
 */
void Heartbeat_Check(Heartbeat_Handle_t *hb)
{
    if (HAL_GetTick() - hb->last_packet_tick > HEARTBEAT_TIMEOUT_MS)
    {
        if (hb->state == LINK_ALIVE)
        {
            hb->lost_count++; // 记录一次断连事件
        }
        hb->state = LINK_LOST;
        hb->signal_level = 0;
    }
}

uint8_t Heartbeat_GetSignalLevel(Heartbeat_Handle_t *hb)
{
    if (hb->state == LINK_LOST) return 0;
    return hb->signal_level;
}