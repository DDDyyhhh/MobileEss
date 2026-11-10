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

#define MSSD_MAX_NODES 4  // 当前配置使用节点 2、3；数组索引即节点 ID

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

/*
 * 手册 3.3：0x09、0x0A 分别返回两个电机的实际转速，
 * 每个寄存器是一个有符号 16 位 RPM 值，不是一个 32 位数的高低字。
 */
#define MSSD_REG_ACT_SPD_R     0x0009  // 通道 R 实际速度，int16 RPM
#define MSSD_REG_ACT_SPD_L     0x000A  // 通道 L 实际速度，int16 RPM


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
    
    /* 每个通道的实际速度由一个 int16 寄存器直接给出，单位 RPM。 */
    int16_t  raw_speed_r;
    int16_t  raw_speed_l;
    uint32_t speed_r_update_tick;
    uint32_t speed_l_update_tick;
    int32_t  actual_rpm_r;   // 通道 R 实际 RPM
    int32_t  actual_rpm_l;   // 通道 L 实际 RPM

    int16_t  temp_r;         // 右侧温度 (℃)
    int16_t  temp_l;         // 左侧温度 (℃)
    float    bus_current_r;  // 右侧电流 (A)
    float    bus_current_l;  // 左侧电流 (A) (双驱控制器通常有两个电流)
    float    bus_voltage;    // 母线电压 (V)

    uint32_t last_update_tick;
} MSSD_Data_t;

extern volatile MSSD_Data_t g_mssd_drives[MSSD_MAX_NODES];

/* --- API 函数声明 --- */
uint8_t MSSD_Set_Velocity(uint8_t node_id, MSSD_Channel_t ch, int32_t speed_rpm);
uint8_t MSSD_Stop_Motor(uint8_t node_id, MSSD_Channel_t ch);
uint8_t MSSD_Req_Velocity(uint8_t node_id, MSSD_Channel_t ch);
uint8_t MSSD_Req_Power(uint8_t node_id);
uint8_t MSSD_Req_Temperature(uint8_t node_id);
uint8_t MSSD_Req_Register(uint8_t node_id, uint16_t reg_addr);
void MSSD_Parse_Rx_Message(uint32_t can_id, uint8_t *data, uint8_t dlc);

#endif /* __MSSD_CAN_H */