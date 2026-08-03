/**
 ******************************************************************************
 * @file           : connet_check.c
 * @brief          : Communication heartbeat monitor
 ******************************************************************************
 */

#include "connet_check.h"
#include <stdio.h>
#include "app_context.h"
#include "serial_port.h"

static uint8_t g_suspend_heartbeat_check = 0;

uint32_t GetLastReceiveTimeAgo(void)
{
    if (g_app.last_receive_timestamp == 0)
    {
        return 0xFFFFFFFF;
    }
    return HAL_GetTick() - g_app.last_receive_timestamp;
}

void Suspend_Heartbeat_Check(uint8_t suspend)
{
    g_suspend_heartbeat_check = suspend;
}

uint8_t CheckCommunicationStatus(void)
{
    static uint8_t last_reported_connected = 0xFF;
    static uint32_t last_dbg_tick = 0;

    if (g_suspend_heartbeat_check)
    {
        uint32_t now = HAL_GetTick();
        if ((last_reported_connected != 1U) || ((now - last_dbg_tick) >= 500U))
        {
            last_dbg_tick = now;
            last_reported_connected = 1U;
           // Serial_Printf(SERIAL_PORT_2, "[COMM] suspend=1 connected=1 now=%lu\r\n", now);
        }
        return 1;
    }

    uint32_t current_time = HAL_GetTick();
    uint32_t delta = current_time - g_app.last_receive_timestamp;

    if (g_app.communication_connected && (delta > COMMUNICATION_TIMEOUT_MS))
    {
        g_app.communication_connected = 0;
    }

    if ((last_reported_connected != g_app.communication_connected) ||
        ((current_time - last_dbg_tick) >= 500U))
    {
        last_dbg_tick = current_time;
        last_reported_connected = g_app.communication_connected;
        // Serial_Printf(SERIAL_PORT_2,
        //               "[COMM] connected=%u last_rx=%lu now=%lu delta=%lu timeout=%u\r\n",
        //               g_app.communication_connected,
        //               g_app.last_receive_timestamp,
        //               current_time,
        //               delta,
        //               (unsigned int)COMMUNICATION_TIMEOUT_MS);
    }

    return g_app.communication_connected;
}
