/**
 ******************************************************************************
 * @file           : mssd_can.c
 * @brief          : 美思固 MSSD-60EHB_2D CAN 通信协议实现
 ******************************************************************************
 */

#include "mssd_can.h"
#include <string.h>

// 声明你在 freertos.c 里写好的入队函数
extern uint8_t CAN_Enqueue_Message(uint32_t id, uint8_t *data, uint8_t len, uint8_t is_remote);

// 实例化驱动器数据
volatile MSSD_Data_t g_mssd_drives[MSSD_MAX_NODES] = {0};

/**
 * @brief 内部组包函数：将寄存器地址转换为 0x4000 格式并小端打包
 */
static void MSSD_Send_Command(uint8_t node_id, uint8_t cmd, uint16_t modbus_reg, uint32_t value)
{
    uint8_t tx_data[8] = {0};

    // 1. 设置功能码
    tx_data[0] = cmd;

    // 2. 设置对象字典索引 (0x4000 + Modbus地址) -> 小端序
    uint16_t index = 0x4000 + modbus_reg;
    tx_data[1] = index & 0xFF;         // 低字节在前
    tx_data[2] = (index >> 8) & 0xFF;  // 高字节在后

    // 3. 子索引固定为 0
    tx_data[3] = 0x00;

    // 4. 数据段打包
    if (cmd == MSSD_CMD_WRITE_4BYTE)
    {
        // MSSD 32位速度数据格式:
        // 高16位先发，低16位后发；每个16位内部小端
        // 1000  = 0x000003E8 -> 00 00 E8 03
        // -500  = 0xFFFFFE0C -> FF FF 0C FE
        tx_data[4] = (value >> 16) & 0xFF;
        tx_data[5] = (value >> 24) & 0xFF;
        tx_data[6] = value & 0xFF;
        tx_data[7] = (value >> 8) & 0xFF;
    }
    else
    {
        // 其他命令保持普通小端序
        tx_data[4] = value & 0xFF;
        tx_data[5] = (value >> 8) & 0xFF;
        tx_data[6] = (value >> 16) & 0xFF;
        tx_data[7] = (value >> 24) & 0xFF;
    }

    // 5. 丢入 CAN 队列 (美思固协议规定 CAN ID 直接就是 Node ID)
    CAN_Enqueue_Message(node_id, tx_data, 8, 0);
}


/**
 * @brief 发送速度控制指令 (对应 PDF 3.1 示例)
 * @param node_id 驱动器节点号 (例如 1)
 * @param ch 电机通道 (MSSD_CH_RIGHT 或 MSSD_CH_LEFT)
 * @param speed_rpm 目标转速 (-32768 ~ 32767 RPM)
 */
void MSSD_Set_Velocity(uint8_t node_id, MSSD_Channel_t ch, int32_t speed_rpm)
{
    uint16_t target_reg = (ch == MSSD_CH_RIGHT) ? MSSD_REG_SPEED_RIGHT : MSSD_REG_SPEED_LEFT;
    
    // 写入 4 字节数据 (0x24)
    MSSD_Send_Command(node_id, MSSD_CMD_WRITE_4BYTE, target_reg, (uint32_t)speed_rpm);
}

/**
 * @brief 发送紧急停止指令 (对应 PDF 3.2 示例)
 */
void MSSD_Stop_Motor(uint8_t node_id, MSSD_Channel_t ch)
{
    uint16_t target_reg = (ch == MSSD_CH_RIGHT) ? MSSD_REG_STOP_RIGHT : MSSD_REG_STOP_LEFT;

    // 写入 2 字节数据 (0x22)，值为 1 表示停止
    MSSD_Send_Command(node_id, MSSD_CMD_WRITE_2BYTE, target_reg, 1);
}

/**
 * @brief 轮询请求电机实际速度 (对应 PDF 3.3 示例)
 * @note  由于美思固将 32 位速度拆成了高低两个 16 位寄存器，我们需要发两条读取指令
 */
void MSSD_Req_Velocity(uint8_t node_id, MSSD_Channel_t ch)
{
    uint16_t reg_h = (ch == MSSD_CH_RIGHT) ? MSSD_REG_ACT_SPD_R_H : MSSD_REG_ACT_SPD_L_H;
    uint16_t reg_l = (ch == MSSD_CH_RIGHT) ? MSSD_REG_ACT_SPD_R_L : MSSD_REG_ACT_SPD_L_L;

    // 读取指令 (0x40)，后面的 Value 填 0 即可
    MSSD_Send_Command(node_id, MSSD_CMD_READ, reg_h, 0);
    MSSD_Send_Command(node_id, MSSD_CMD_READ, reg_l, 0);
}

/**
 * @brief 轮询请求电压和电流
 */
void MSSD_Req_Power(uint8_t node_id)
{
    // 读取母线电压
    MSSD_Send_Command(node_id, MSSD_CMD_READ, MSSD_REG_BUS_VOLTAGE, 0);
    // 读取右侧和左侧电流
    MSSD_Send_Command(node_id, MSSD_CMD_READ, MSSD_REG_CURR_BUS_R, 0);
    MSSD_Send_Command(node_id, MSSD_CMD_READ, MSSD_REG_CURR_BUS_L, 0);
}

/**
 * @brief 轮询请求驱动器温度
 */
void MSSD_Req_Temperature(uint8_t node_id)
{
    // 读取左右两侧温度
    MSSD_Send_Command(node_id, MSSD_CMD_READ, MSSD_REG_TEMP_DRV_R, 0);
    MSSD_Send_Command(node_id, MSSD_CMD_READ, MSSD_REG_TEMP_DRV_L, 0);
}

/**
 * @brief 解析底座接收到的 CAN 报文 (放在 CAN RX 中断或队列消费者中调用)
 */
void MSSD_Parse_Rx_Message(uint32_t can_id, uint8_t *data, uint8_t dlc)
{
    // 美思固协议中，CAN ID 就是 Node ID (0x01 ~ 0x7F)
    if (can_id == 0 || can_id >= MSSD_MAX_NODES || dlc < 8) return;

    volatile MSSD_Data_t *drive = &g_mssd_drives[can_id];
    drive->is_online = 1;
    drive->last_update_tick = HAL_GetTick();

    uint8_t ack_cmd = data[0];
    uint16_t index = (data[2] << 8) | data[1]; // 还原索引号

    // 判断是否为读取成功的响应 (返回 2 字节)
    if (ack_cmd == MSSD_ACK_2BYTE)
    {
        uint16_t modbus_reg = index - 0x4000;
        uint16_t received_val = (data[5] << 8) | data[4]; // 提取小端数据

        switch (modbus_reg)
        {
           case MSSD_REG_ACT_SPD_R_H:
    drive->raw_speed_r_h = received_val;
    break;

        case MSSD_REG_ACT_SPD_R_L:
        drive->raw_speed_r_l = received_val;
        drive->actual_rpm_r = (int32_t)(((uint32_t)drive->raw_speed_r_h << 16) |
                                        (uint32_t)drive->raw_speed_r_l);
        break;

    case MSSD_REG_ACT_SPD_L_H:
        drive->raw_speed_l_h = received_val;
        break;

    case MSSD_REG_ACT_SPD_L_L:
        drive->raw_speed_l_l = received_val;
        drive->actual_rpm_l = (int32_t)(((uint32_t)drive->raw_speed_l_h << 16) |
                                        (uint32_t)drive->raw_speed_l_l);
        break;
        
    case MSSD_REG_TEMP_DRV_R:
        drive->temp_r = (int16_t)received_val; // 单位 ℃
        break;
        
    case MSSD_REG_TEMP_DRV_L:
        drive->temp_l = (int16_t)received_val;
        break;

    case MSSD_REG_CURR_BUS_R:
        drive->bus_current_r = (float)received_val * 0.01f; // 转换为 Amps
        break;

    case MSSD_REG_BUS_VOLTAGE:
        drive->bus_voltage = (float)received_val; // 单位 V
        break;

            }
        }
}