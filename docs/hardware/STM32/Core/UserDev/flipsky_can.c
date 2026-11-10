#include "flipsky_can.h"
#include "blade_motor_config.h"
#include "odrive_can.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <string.h>

#define FLIPSKY_CMD_CURRENT_PERCENT FLIPSKY_CAN_CMD_CURRENT_PERCENT
#define FLIPSKY_CMD_REALTIME_0      FLIPSKY_CAN_CMD_REALTIME_0
#define FLIPSKY_CMD_REALTIME_1      FLIPSKY_CAN_CMD_REALTIME_1
#define FLIPSKY_CMD_REALTIME_2      FLIPSKY_CAN_CMD_REALTIME_2

static FlipskyCanFeedback_t s_feedback;
static FlipskyCanStats_t s_stats;

static uint16_t Flipsky_Crc16_Modbus(const uint8_t *data, uint32_t length)
{
    static const uint16_t crc_table[16] = {
        0x0000U, 0xC0C1U, 0xC181U, 0x0140U,
        0xC301U, 0x03C0U, 0x0280U, 0xC241U,
        0xC601U, 0x06C0U, 0x0780U, 0xC741U,
        0x0500U, 0xC5C1U, 0xC481U, 0x0440U
    };
    uint16_t crc = 0xFFFFU;
    while (length-- != 0U)
    {
        crc ^= *data++;
        crc = (uint16_t)((crc >> 4) ^ crc_table[crc & 0x0FU]);
        crc = (uint16_t)((crc >> 4) ^ crc_table[crc & 0x0FU]);
    }
    return crc;
}

static void Flipsky_WriteI32BE(uint8_t *data, int32_t value)
{
    data[0] = (uint8_t)((uint32_t)value >> 24);
    data[1] = (uint8_t)((uint32_t)value >> 16);
    data[2] = (uint8_t)((uint32_t)value >> 8);
    data[3] = (uint8_t)value;
}

static int32_t Flipsky_ReadI32BE(const uint8_t *data)
{
    return (int32_t)(((uint32_t)data[0] << 24) |
                     ((uint32_t)data[1] << 16) |
                     ((uint32_t)data[2] << 8) |
                     data[3]);
}

static int16_t Flipsky_ReadI16BE(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint32_t Flipsky_ExtId(uint8_t command)
{
    return ((uint32_t)FLIPSKY_ESC_ID << 8) | command;
}

void Flipsky_CAN_Init(void)
{
    memset(&s_feedback, 0, sizeof(s_feedback));
    memset(&s_stats, 0, sizeof(s_stats));
}

uint8_t Flipsky_CAN_SetCurrentPercent(float percent)
{
    uint8_t data[6];
    uint16_t crc;
    int32_t scaled;

    if (!isfinite(percent) || percent < 0.0f || percent > 100.0f)
    {
        return 0U;
    }

    scaled = (int32_t)((percent / 100.0f) * 100000.0f);
    Flipsky_WriteI32BE(data, scaled);
    crc = Flipsky_Crc16_Modbus(data, 4U);
    data[4] = (uint8_t)(crc >> 8);
    data[5] = (uint8_t)crc;

    if (CAN_Enqueue_Ext_Message(Flipsky_ExtId(FLIPSKY_CMD_CURRENT_PERCENT),
                                 data, sizeof(data), 0U) == 0U)
    {
        s_stats.tx_enqueue_fail_count++;
        return 0U;
    }

    s_stats.tx_current_percent_count++;
    return 1U;
}

uint8_t Flipsky_CAN_Stop(void)
{
    return Flipsky_CAN_SetCurrentPercent(0.0f);
}

void Flipsky_CAN_ProcessRx(uint32_t ext_id, const uint8_t *data, uint8_t dlc)
{
    uint8_t esc_id;
    uint8_t command;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if (data == NULL || ext_id > 0x1FFFFFFFU)
    {
        return;
    }

    esc_id = (uint8_t)((ext_id >> 8) & 0xFFU);
    command = (uint8_t)(ext_id & 0xFFU);
    if (esc_id != FLIPSKY_ESC_ID || dlc != 8U)
    {
        return;
    }

    received_crc = (uint16_t)(((uint16_t)data[6] << 8) | data[7]);
    calculated_crc = Flipsky_Crc16_Modbus(data, 6U);
    if (received_crc != calculated_crc)
    {
        s_feedback.crc_valid = 0U;
        s_stats.rx_crc_error_count++;
        return;
    }

    s_feedback.crc_valid = 1U;
    s_feedback.online = 1U;
    s_feedback.last_rx_tick = HAL_GetTick();

    switch (command)
    {
    case FLIPSKY_CMD_REALTIME_0:
        s_stats.rx_realtime_0_count++;
        s_feedback.motor_current_a = Flipsky_ReadI32BE(data) * 0.001f;
        s_feedback.battery_current_a = Flipsky_ReadI32BE(data + 4) * 0.001f;
        break;

    case FLIPSKY_CMD_REALTIME_1:
        s_stats.rx_realtime_1_count++;
        s_feedback.erpm = Flipsky_ReadI32BE(data);
        s_feedback.duty = Flipsky_ReadI32BE(data + 4) * 0.001f;
        break;

    case FLIPSKY_CMD_REALTIME_2:
        s_stats.rx_realtime_2_count++;
        s_feedback.battery_voltage_v = Flipsky_ReadI16BE(data) * 0.01f;
        s_feedback.motor_temp_c = Flipsky_ReadI16BE(data + 2) * 0.01f;
        s_feedback.mosfet_temp_c = Flipsky_ReadI16BE(data + 4) * 0.01f;
        break;

    default:
        break;
    }
}

const FlipskyCanFeedback_t *Flipsky_CAN_GetFeedback(void)
{
    return &s_feedback;
}

const FlipskyCanStats_t *Flipsky_CAN_GetStats(void)
{
    return &s_stats;
}

uint8_t Flipsky_CAN_IsOnline(void)
{
    if (s_feedback.online == 0U)
    {
        return 0U;
    }

    if ((HAL_GetTick() - s_feedback.last_rx_tick) > FLIPSKY_CAN_RX_TIMEOUT_MS)
    {
        s_feedback.online = 0U;
    }
    return s_feedback.online;
}
// 定义占空比控制指令码 (0x02)
#define FLIPSKY_CAN_CMD_DUTY_GEAR_CRC16 0x02U

/**
 * @brief 发送占空比控制指令 (0x02)
 * @param duty_percent 占空比百分比 (0.0f ~ 100.0f)
 * @return 1: 成功入队, 0: 失败
 */
uint8_t Flipsky_CAN_SetDutyCycle(float duty_percent)
{
    uint8_t data[7]; // 占空比指令数据长度为 7 字节
    uint16_t crc;
    int32_t scaled;

    if (!isfinite(duty_percent) || duty_percent < 0.0f || duty_percent > 100.0f)
    {
        return 0U;
    }

    // 占空比换算：如 10% -> 0.1 * 1000000 = 100000 (0x000186A0)
    scaled = (int32_t)((duty_percent / 100.0f) * 1000000.0f);

    // 1. 写入前 4 字节大端序 int32 数值
    Flipsky_WriteI32BE(data, scaled);

    // 2. 第 5 字节 (data[4])：0x00 代表无挡位限制
    data[4] = 0x00U;

    // 3. 计算前 5 字节 (data[0] ~ data[4]) 的 CRC16 校验码
    crc = Flipsky_Crc16_Modbus(data, 5U);
    // 【修正高低字节位置】：低字节放 data[5]，高字节放 data[6]
    data[5] = (uint8_t)(crc & 0xFFU); // 发送 0x36
    data[6] = (uint8_t)(crc >> 8U);   // 发送 0xCE

    // 4. 发送扩展帧 (EID = 0x00009A02)
    if (CAN_Enqueue_Ext_Message(Flipsky_ExtId(FLIPSKY_CAN_CMD_DUTY_GEAR_CRC16),
                                 data, sizeof(data), 0U) == 0U)
    {
        s_stats.tx_enqueue_fail_count++;
        return 0U;
    }

    s_stats.tx_current_percent_count++;
    return 1U;
}


// 定义转速控制指令码 (0x18)
#define FLIPSKY_CAN_CMD_ERPM_GEAR 0x18U

/**
 * @brief 发送转速控制指令 (0x18，不带 CRC)
 * @param erpm 电子转速 ERPM (目标RPM * 极对数)
 * @return 1: 成功入队, 0: 失败
 */
uint8_t Flipsky_CAN_SetSpeedErpm(float erpm)
{
    uint8_t data[5];
    int32_t scaled_erpm;

    if (!isfinite(erpm) || erpm < 0.0f)
    {
        return 0U;
    }

    // ERPM 为直接的整数，不需要乘以 1000000！
    scaled_erpm = (int32_t)erpm;

    // 1. 写入 4 字节大端序 ERPM 数值
    Flipsky_WriteI32BE(data, scaled_erpm);

    // 2. 第 5 字节：0x00 代表无挡位限制
    data[4] = 0x00U;

    // 3. 发送指令 0x18 (EID = 0x00009A18)
    return CAN_Enqueue_Ext_Message(Flipsky_ExtId(FLIPSKY_CAN_CMD_ERPM_GEAR),
                                 data, 5U, 0U);
}