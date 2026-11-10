/**
 ******************************************************************************
 * @file           : mssd_can.c
 * @brief          : 美思固 MSSD-60EHB_2D CAN 通信协议实现
 *
 * MSSD 报文约定（CAN 标准帧）：
 *   CAN ID       = 驱动器节点 ID
 *   DATA[0]      = 功能码
 *   DATA[1..2]   = 0x4000 + Modbus 寄存器地址，小端
 *   DATA[3]      = 子索引，当前固定为 0
 *   DATA[4..7]   = 数据，小端（32 位速度按厂家示例打包）
 ******************************************************************************
 */

#include "mssd_can.h"
#include <stddef.h>

/* 由 odrive_can.c 提供的按通道保留最新值的控制发送接口。 */
extern uint8_t CAN_Enqueue_Keyed_Control_Message(uint8_t control_key,
                                                  uint32_t id, uint8_t *data,
                                                  uint8_t len, uint8_t is_remote);
extern uint8_t CAN_Enqueue_Message(uint32_t id, uint8_t *data,
                                   uint8_t len, uint8_t is_remote);

volatile MSSD_Data_t g_mssd_drives[MSSD_MAX_NODES] = {0};

static uint8_t mssd_send_command(uint8_t node_id, uint8_t command,
                                 uint16_t modbus_reg, uint32_t value)
{
    uint8_t data[8] = {0};
    uint16_t index;

    if (node_id == 0U || node_id >= MSSD_MAX_NODES)
        return 0U;

    index = (uint16_t)(0x4000U + modbus_reg);
    data[0] = command;
    data[1] = (uint8_t)(index & 0xFFU);
    data[2] = (uint8_t)(index >> 8);
    data[3] = 0U;

    if (command == MSSD_CMD_WRITE_4BYTE)
    {
        /* 手册示例：1000 -> 00 00 E8 03，-500 -> FF FF 0C FE */
        data[4] = (uint8_t)(value >> 16);
        data[5] = (uint8_t)(value >> 24);
        data[6] = (uint8_t)(value & 0xFFU);
        data[7] = (uint8_t)(value >> 8);
    }
    else
    {
        data[4] = (uint8_t)(value & 0xFFU);
        data[5] = (uint8_t)(value >> 8);
        data[6] = (uint8_t)(value >> 16);
        data[7] = (uint8_t)(value >> 24);
    }

    if (command == MSSD_CMD_WRITE_4BYTE &&
        (modbus_reg == MSSD_REG_SPEED_RIGHT || modbus_reg == MSSD_REG_SPEED_LEFT))
    {
        uint8_t control_key = (uint8_t)(((node_id - 2U) * 2U) +
                                        (modbus_reg == MSSD_REG_SPEED_RIGHT ? 1U : 2U));
        return CAN_Enqueue_Keyed_Control_Message(control_key, node_id, data, 8U, 0U);
    }

    return CAN_Enqueue_Message(node_id, data, 8U, 0U);
}

uint8_t MSSD_Set_Velocity(uint8_t node_id, MSSD_Channel_t ch,
                          int32_t speed_rpm)
{
    uint16_t reg = (ch == MSSD_CH_RIGHT) ? MSSD_REG_SPEED_RIGHT
                                         : MSSD_REG_SPEED_LEFT;
    return mssd_send_command(node_id, MSSD_CMD_WRITE_4BYTE, reg,
                             (uint32_t)speed_rpm);
}

uint8_t MSSD_Stop_Motor(uint8_t node_id, MSSD_Channel_t ch)
{
    uint16_t reg = (ch == MSSD_CH_RIGHT) ? MSSD_REG_STOP_RIGHT
                                         : MSSD_REG_STOP_LEFT;
    return mssd_send_command(node_id, MSSD_CMD_WRITE_2BYTE, reg, 1U);
}

uint8_t MSSD_Req_Register(uint8_t node_id, uint16_t reg_addr)
{
    return mssd_send_command(node_id, MSSD_CMD_READ, reg_addr, 0U);
}

uint8_t MSSD_Req_Velocity(uint8_t node_id, MSSD_Channel_t ch)
{
    uint16_t reg = (ch == MSSD_CH_RIGHT) ? MSSD_REG_ACT_SPD_R
                                         : MSSD_REG_ACT_SPD_L;

    return MSSD_Req_Register(node_id, reg);
}

uint8_t MSSD_Req_Power(uint8_t node_id)
{
    uint8_t ok_voltage = MSSD_Req_Register(node_id, MSSD_REG_BUS_VOLTAGE);
    uint8_t ok_current_r = MSSD_Req_Register(node_id, MSSD_REG_CURR_BUS_R);
    uint8_t ok_current_l = MSSD_Req_Register(node_id, MSSD_REG_CURR_BUS_L);

    return (uint8_t)(ok_voltage && ok_current_r && ok_current_l);
}

uint8_t MSSD_Req_Temperature(uint8_t node_id)
{
    uint8_t ok_r = MSSD_Req_Register(node_id, MSSD_REG_TEMP_DRV_R);
    uint8_t ok_l = MSSD_Req_Register(node_id, MSSD_REG_TEMP_DRV_L);

    return (uint8_t)(ok_r && ok_l);
}

static uint16_t mssd_unpack_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void mssd_update_speed(volatile MSSD_Data_t *drive,
                              MSSD_Channel_t ch, int16_t value,
                              uint32_t now)
{
    if (ch == MSSD_CH_RIGHT)
    {
        drive->raw_speed_r = value;
        drive->actual_rpm_r = value;
        drive->speed_r_update_tick = now;
    }
    else
    {
        drive->raw_speed_l = value;
        drive->actual_rpm_l = value;
        drive->speed_l_update_tick = now;
    }
}

void MSSD_Parse_Rx_Message(uint32_t can_id, uint8_t *data, uint8_t dlc)
{
    volatile MSSD_Data_t *drive;
    uint16_t index;
    uint16_t modbus_reg;
    uint16_t value;
    uint32_t now;

    /* 当前固件数组只管理节点 1~3，实际启用节点为 2、3。 */
    if (data == NULL || can_id == 0U || can_id >= MSSD_MAX_NODES || dlc < 3U)
        return;

    if (data[0] == MSSD_ACK_2BYTE && dlc < 6U)
        return;

    if (data[0] != MSSD_ACK_2BYTE)
    {
        /* 写成功/错误响应也属于合法通信，可用于在线判定。 */
        if (data[0] != MSSD_ACK_SUCCESS && data[0] != MSSD_ACK_ERROR)
            return;
    }

    index = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
    if (index < 0x4000U)
        return;

    modbus_reg = (uint16_t)(index - 0x4000U);
    now = HAL_GetTick();
    drive = &g_mssd_drives[can_id];
    drive->is_online = 1U;
    drive->last_update_tick = now;

    if (data[0] != MSSD_ACK_2BYTE)
        return;

    value = mssd_unpack_u16_le(&data[4]);

    switch (modbus_reg)
    {
    case MSSD_REG_ACT_SPD_R:
        mssd_update_speed(drive, MSSD_CH_RIGHT, (int16_t)value, now);
        break;
    case MSSD_REG_ACT_SPD_L:
        mssd_update_speed(drive, MSSD_CH_LEFT, (int16_t)value, now);
        break;
    case MSSD_REG_TEMP_DRV_R:
        drive->temp_r = (int16_t)value;
        break;
    case MSSD_REG_TEMP_DRV_L:
        drive->temp_l = (int16_t)value;
        break;
    case MSSD_REG_CURR_BUS_R:
        drive->bus_current_r = (float)(int16_t)value * 0.01f;
        break;
    case MSSD_REG_CURR_BUS_L:
        drive->bus_current_l = (float)(int16_t)value * 0.01f;
        break;
    case MSSD_REG_BUS_VOLTAGE:
        drive->bus_voltage = (float)value;
        break;
    default:
        break;
    }
}
