/**
 ******************************************************************************
 * @file           : oid_can.c
 * @brief          : 欧艾迪 OID FOC 驱动器 CAN 通信协议实现 (V1.2)
 * @note           大端序 (Big-Endian) 编码，与 MSSD 小端序相反
 ******************************************************************************
 */

#include "oid_can.h"
#include <string.h>
#include "odrive_can.h"

// 声明 freertos.c 中的 CAN 发送入队函数
extern uint8_t CAN_Enqueue_Message(uint32_t id, uint8_t *data, uint8_t len, uint8_t is_remote);

// 实例化驱动器数据数组
volatile OID_Data_t g_oid_drives[OID_MAX_NODES] = {0};
volatile uint32_t g_oid_speed_query_failures = 0;

// 大端序编码
/* ==================================================================== */

/**
 * @brief 发送 5 字节 CAN 帧 (DATA[0]=cmd, DATA[1~4]=int32 大端)
 *        用于速度/加速度/位置等控制指令
 */
static void oid_send_cmd_int32(uint8_t node_id, uint8_t cmd, int32_t value)
{
    uint8_t tx_data[8] = {0};
    tx_data[0] = cmd;
    // 大端序: 高字节在前
    tx_data[1] = (uint8_t)((uint32_t)value >> 24);
    tx_data[2] = (uint8_t)((uint32_t)value >> 16);
    tx_data[3] = (uint8_t)((uint32_t)value >> 8);
    tx_data[4] = (uint8_t)((uint32_t)value >> 0);

    if (cmd == OID_CMD_SPEED)
    {
        CAN_Enqueue_Control_Message(node_id, tx_data, 5, 0);
    }
    else
    {
        CAN_Enqueue_Message(node_id, tx_data, 5, 0);
    }
}

/**
 * @brief 发送 3 字节 CAN 帧 (DATA[0]=cmd, DATA[1~2]=int16 大端)
 *        用于电流/占空比等控制指令
 */
static void oid_send_cmd_int16(uint8_t node_id, uint8_t cmd, int16_t value)
{
    uint8_t tx_data[8] = {0};
    tx_data[0] = cmd;
    tx_data[1] = (uint8_t)((uint16_t)value >> 8);
    tx_data[2] = (uint8_t)((uint16_t)value >> 0);

    CAN_Enqueue_Message(node_id, tx_data, 3, 0);
}

/* ==================================================================== */
/*              控制命令实现                                              */
/* ==================================================================== */

void OID_Set_Speed(uint8_t node_id, int32_t speed_erpm)
{
    // 0x02 + int32 大端 erpm
    oid_send_cmd_int32(node_id, OID_CMD_SPEED, speed_erpm);
}

void OID_Set_Accel(uint8_t node_id, int32_t accel_erpmss)
{
    // 0x0A + int32 大端 erpm/s²
    oid_send_cmd_int32(node_id, OID_CMD_ACCEL, accel_erpmss);
}

void OID_Set_Decel(uint8_t node_id, int32_t decel_erpmss)
{
    // 0x10 + int32 大端 erpm/s²
    oid_send_cmd_int32(node_id, OID_CMD_DECEL, decel_erpmss);
}

void OID_Brake(uint8_t node_id)
{
    // 0x07 + 4字节零 = 刹车
    oid_send_cmd_int32(node_id, OID_CMD_BRAKE, 0);
}

void OID_Heartbeat(uint8_t node_id)
{
    // 发送状态字读取请求作为心跳 (DATA[0]=0x00, DLC=1)
    // 驱动器收到后会回复状态，起到保活作用
    uint8_t tx_data[8] = {0};
    tx_data[0] = 0x00;
    CAN_Enqueue_Message(node_id, tx_data, 1, 0);
}

uint8_t OID_Req_Parameter(uint8_t node_id, uint8_t sub_index)
{
    // 0x0F + sub_index
    uint8_t tx_data[8] = {0};
    tx_data[0] = OID_CMD_READ_PARAM;
    tx_data[1] = sub_index;

    // 记住当前节点正在等待哪个参数
    if (node_id < OID_MAX_NODES)
    {
        g_oid_drives[node_id].pending_subidx = sub_index;
    }

    return CAN_Enqueue_Message(node_id, tx_data, 2, 0);
}

uint8_t OID_Req_Actual_Speed(uint8_t node_id)
{
    uint8_t result = OID_Req_Parameter(node_id, OID_PARAM_ACTUAL_SPEED);
    if (!result)
    {
        g_oid_speed_query_failures++;
    }
    return result;
}

/* ==================================================================== */
/*              CAN 接收解析                                              */
/* ==================================================================== */

void OID_Parse_Rx_Message(uint32_t can_id, uint8_t *data, uint8_t dlc)
{
    // CAN ID 范围检查
    if (can_id == 0 || can_id >= OID_MAX_NODES || data == NULL)
        return;

    volatile OID_Data_t *drive = &g_oid_drives[can_id];
    drive->is_online = 1;
    drive->last_update_tick = HAL_GetTick();

    uint8_t cmd = data[0];

    // ================================================================
    // 处理 0x0F 读参数响应
    // ================================================================
    if (cmd == OID_CMD_READ_PARAM && dlc >= 3)
    {
        uint8_t sub_idx = data[1];

        switch (sub_idx)
        {
        case OID_PARAM_ERROR:       // 错误码 (DLC=4, uint16)
            if (dlc >= 4)
                drive->error_code = (uint16_t)((data[2] << 8) | data[3]);
            break;

        case OID_PARAM_ACTUAL_SPEED: // 实时电转速 (DLC=6, int32 erpm)
            if (dlc >= 6)
            {
                drive->actual_speed_erpm = (int32_t)((uint32_t)data[2] << 24) |
                                            ((uint32_t)data[3] << 16) |
                                            ((uint32_t)data[4] << 8)  |
                                            (uint32_t)data[5];
                drive->actual_speed_update_tick = drive->last_update_tick;
            }
            break;

        case OID_PARAM_BUS_VOLTAGE: // 母线电压 (DLC=4, uint16, 单位 V)
            if (dlc >= 4)
                drive->bus_voltage = (float)((data[2] << 8) | data[3]);
            break;

        case OID_PARAM_BUS_CURRENT: // 母线电流 (DLC=4, uint16, 单位 10mA)
            if (dlc >= 4)
                drive->bus_current = (float)((int16_t)((data[2] << 8) | data[3])) * 0.01f;
            break;

        case OID_PARAM_TEMPERATURE: // 电机温度 (DLC=4, uint16, 单位 ℃)
            if (dlc >= 4)
                drive->motor_temp = (int16_t)((data[2] << 8) | data[3]);
            break;

        case OID_PARAM_POS_MULTI:   // 多圈位置 (DLC=6, uint32, 单位 0.01°)
            if (dlc >= 6)
                drive->encoder_pos = (int32_t)((uint32_t)data[2] << 24) |
                                       ((uint32_t)data[3] << 16) |
                                       ((uint32_t)data[4] << 8)  |
                                       (uint32_t)data[5];
            break;

        case OID_PARAM_POWER:       // 功率 (DLC=4, uint16, 单位 W)
            if (dlc >= 4)
                drive->power = (uint16_t)((data[2] << 8) | data[3]);
            break;

        case OID_PARAM_PHASE_CURRENT: // 相电流 (DLC=4, uint16, 单位 10mA)
            if (dlc >= 4)
                drive->phase_current = (float)(int16_t)((data[2] << 8) | data[3]) * 0.01f;
            break;

        default:
            break;
        }

        // 清除等待标志
        drive->pending_subidx = 0xFF;
        return;
    }

    // ================================================================
    // 其他可能的响应可以在此扩展
    // ================================================================
    // 例如: 状态字 0x00、速度直读 0x02 等
}
