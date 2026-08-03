/**
 ******************************************************************************
 * @file           : mssd_can.h
 * @brief          : 美思固 MSSD-60EHB_2D 双驱无刷电机控制器 CAN 驱动
 * @note           基于 CAN 环形队列非阻塞发送，完全兼容原 ODrive 架构风格
 ******************************************************************************
 */

#ifndef __MSSD_CAN_H
#define __MSSD_CAN_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define MSSD_MAX_NODES 4  // 支持的最大驱动器数量

/* --- MSSD CAN 功能码定义 (来自 PDF 表 2.2) --- */
#define MSSD_CMD_WRITE_1BYTE    0x21
#define MSSD_CMD_WRITE_2BYTE    0x22
#define MSSD_CMD_WRITE_3BYTE    0x23
#define MSSD_CMD_WRITE_4BYTE    0x24
#define MSSD_CMD_READ           0x40

/* --- MSSD 状态响应码定义 (来自 PDF 表 2.4) --- */
#define MSSD_ACK_1BYTE          0x41
#define MSSD_ACK_2BYTE          0x42
#define MSSD_ACK_3BYTE          0x43
#define MSSD_ACK_4BYTE          0x44
#define MSSD_ACK_SUCCESS        0x60
#define MSSD_ACK_ERROR          0x80

/* --- 核心 Modbus 寄存器地址定义 (根据 PDF 示例推导) --- */
// 提示: 如果你拿到了完整的寄存器表，请在这里补全剩余的寄存器
#define MSSD_REG_STOP_RIGHT     0x0038  // 右电机停止 (写1停止)
#define MSSD_REG_SPEED_RIGHT    0x0039  // 右电机目标速度 (32位)
#define MSSD_REG_STOP_LEFT      0x003C  // 左电机停止 (写1停止，推测值)
#define MSSD_REG_SPEED_LEFT     0x003D  // 左电机目标速度 (32位)

#define MSSD_REG_ACT_SPD_R_H    0x0009  // 右电机实际速度 (高16位)
#define MSSD_REG_ACT_SPD_R_L    0x000A  // 右电机实际速度 (低16位)
// 左电机的实际速度寄存器请查阅完整手册后填入 (假设为 0x0B 和 0x0C)
#define MSSD_REG_ACT_SPD_L_H    0x000B  // 左电机实际速度 (高16位，推测值)
#define MSSD_REG_ACT_SPD_L_L    0x000C  // 左电机实际速度 (低16位，推测值)


// 传感器与状态寄存器
#define MSSD_REG_BUS_VOLTAGE    0x001D
#define MSSD_REG_CURR_BUS_R     0x0007  //母线电流
#define MSSD_REG_CURR_BUS_L     0x0012  //母线电流
#define MSSD_REG_TEMP_DRV_R     0x000F
#define MSSD_REG_TEMP_DRV_L     0x001A

/* --- 电机通道定义 --- */
typedef enum {
    MSSD_CH_RIGHT = 0,
    MSSD_CH_LEFT  = 1
} MSSD_Channel_t;

/* --- 驱动器状态结构体 --- */
typedef struct {
    uint8_t  is_online;
    
    // 因为 MSSD 的速度读取是分高低 16 位两次返回的，需要缓存
    uint16_t raw_speed_r_h;
    uint16_t raw_speed_r_l;
    int32_t  actual_rpm_r;   // 最终合成的右轮 RPM

    uint16_t raw_speed_l_h;
    uint16_t raw_speed_l_l;
    int32_t  actual_rpm_l;   // 最终合成的左轮 RPM

    int16_t  temp_r;         // 右侧温度 (℃)
    int16_t  temp_l;         // 左侧温度 (℃)
    float    bus_current_r;  // 右侧电流 (A)
    float    bus_current_l;  // 左侧电流 (A) (双驱控制器通常有两个电流)
    float    bus_voltage;    // 母线电压 (V)

    uint32_t last_update_tick;
} MSSD_Data_t;

extern volatile MSSD_Data_t g_mssd_drives[MSSD_MAX_NODES];

/* --- API 函数声明 --- */
void MSSD_Set_Velocity(uint8_t node_id, MSSD_Channel_t ch, int32_t speed_rpm);
void MSSD_Stop_Motor(uint8_t node_id, MSSD_Channel_t ch);
void MSSD_Req_Velocity(uint8_t node_id, MSSD_Channel_t ch);
void MSSD_Req_Power(uint8_t node_id);
void MSSD_Req_Temperature(uint8_t node_id);
void MSSD_Req_Register(uint8_t node_id, uint16_t reg_addr);
void MSSD_Parse_Rx_Message(uint32_t can_id, uint8_t *data, uint8_t dlc);

#endif /* __MSSD_CAN_H */