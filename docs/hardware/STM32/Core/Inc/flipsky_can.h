#ifndef FLIPSKY_CAN_H
#define FLIPSKY_CAN_H

#include <stdint.h>
#include "blade_motor_config.h"

typedef struct
{
    uint8_t online;
    uint8_t crc_valid;
    int32_t erpm;
    float motor_current_a;
    float battery_current_a;
    float duty;
    float mosfet_temp_c;
    float motor_temp_c;
    float battery_voltage_v;
    uint32_t last_rx_tick;
} FlipskyCanFeedback_t;

typedef struct
{
    uint32_t tx_current_percent_count;
    uint32_t tx_enqueue_fail_count;
    uint32_t rx_realtime_0_count;
    uint32_t rx_realtime_1_count;
    uint32_t rx_realtime_2_count;
    uint32_t rx_crc_error_count;
} FlipskyCanStats_t;

void Flipsky_CAN_Init(void);
uint8_t Flipsky_CAN_SetCurrentPercent(float percent);
uint8_t Flipsky_CAN_Stop(void);
void Flipsky_CAN_ProcessRx(uint32_t ext_id, const uint8_t *data, uint8_t dlc);
const FlipskyCanFeedback_t *Flipsky_CAN_GetFeedback(void);
const FlipskyCanStats_t *Flipsky_CAN_GetStats(void);
uint8_t Flipsky_CAN_IsOnline(void);
// 在 flipsky_can.h 中添加声明
uint8_t Flipsky_CAN_SetDutyCycle(float duty_percent);
// 在 flipsky_can.h 中添加声明
uint8_t Flipsky_CAN_SetSpeedErpm(float erpm);
#endif
