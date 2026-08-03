/**
 ******************************************************************************
 * @file           : oid_can.h
 * @brief          : 欧艾迪 OID FOC 驱动器 CAN 通信协议 (V1.2)
 * @note           基于 CAN 环形队列非阻塞发送，适配 chassis_control 框架
 ******************************************************************************
 */

#ifndef __OID_CAN_H
#define __OID_CAN_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define OID_MAX_NODES 5  // 支持的最大驱动器数量 (节点ID 1~4 直接作为索引)

/* ==================================================================== */
/*               OID CAN 命令定义 (DATA[0])                              */
/* ==================================================================== */
#define OID_CMD_CURRENT         0x01  // 电流控制 (int16, 单位 10mA)
#define OID_CMD_SPEED           0x02  // 速度控制 (int32, 单位 erpm)
#define OID_CMD_DUTY            0x03  // 占空比控制 (int16, -1000~1000)
#define OID_CMD_POSITION        0x04  // 位置控制 (int32, 单位 0.01°)
#define OID_CMD_BRAKE           0x07  // 刹车
#define OID_CMD_BUS_CURRENT     0x08  // 母线电流限制 (int16, 10mA)
#define OID_CMD_PHASE_CURRENT   0x09  // 相电流限制 (int16, 10mA)
#define OID_CMD_ACCEL           0x0A  // 加速度设置 (int32, erpm/s²)
#define OID_CMD_SPEED_LIMIT     0x0B  // 速度限制 (int32, erpm)
#define OID_CMD_ACCEL_LIMIT     0x0C  // 加速限制 (int32, erpm/s²)
#define OID_CMD_DECEL_LIMIT     0x0D  // 减速限制 (int32, erpm/s²)
#define OID_CMD_READ_PARAM      0x0F  // 读参数 (复合协议, DATA[1]=子索引)
#define OID_CMD_DECEL           0x10  // 减速度设置 (int32, erpm/s²)
#define OID_CMD_IO_OUTPUT       0x11  // IO 输出
#define OID_CMD_IO_DIR          0x13  // IO 方向
#define OID_CMD_BRAKE_CURRENT   0x14  // 电流刹车 (int16, 10mA)
#define OID_CMD_SOFT_START      0x15  // 软启加速度 (int16)
#define OID_CMD_SOFT_STOP       0x16  // 软停减速度 (int16)

/* ==================================================================== */
/*           0x0F 读参数子索引定义                                       */
/* ==================================================================== */
#define OID_PARAM_ERROR          0x00  // 错误码 (返回 uint16)
#define OID_PARAM_ACTUAL_SPEED   0x01  // 实时电转速 (返回 int32 erpm)
#define OID_PARAM_DUTY           0x02  // 占空比 (返回 uint16, 0~1000)
#define OID_PARAM_POWER          0x03  // 功率 (返回 uint16, W)
#define OID_PARAM_BUS_VOLTAGE    0x04  // 母线电压 (返回 uint16, V)
#define OID_PARAM_BUS_CURRENT    0x05  // 母线电流 (返回 uint16, 10mA)
#define OID_PARAM_PHASE_CURRENT  0x06  // 相电流 (返回 uint16, 10mA)
#define OID_PARAM_TEMPERATURE    0x07  // 电机温度 (返回 uint16, ℃)
#define OID_PARAM_POS_MULTI      0x08  // 多圈位置 (返回 uint32, 0.01°)
#define OID_PARAM_POS_SINGLE     0x09  // 单圈位置 (返回 uint16, 0.01°)
#define OID_PARAM_Z_SIGNAL       0x0A  // Z 信号状态
#define OID_PARAM_FIRMWARE       0x0B  // 固件版本
#define OID_PARAM_IO_STATUS      0x0C  // IO 状态

/* ==================================================================== */
/*               OID 驱动器状态数据结构                                  */
/* ==================================================================== */
typedef struct {
    uint8_t  is_online;
    uint32_t last_update_tick;

    // 反馈数据
    int32_t  actual_speed_erpm;   // 实际速度 (erpm)
    uint32_t actual_speed_update_tick;
    float    bus_voltage;         // 母线电压 (V)
    float    bus_current;         // 母线电流 (A)
    int16_t  motor_temp;          // 电机温度 (℃)
    int32_t  encoder_pos;         // 编码器多圈位置 (0.01°)
    uint16_t error_code;          // 错误码
    uint16_t power;               // 功率 (W)
    float    phase_current;       // 相电流 (A)

    // 内部状态
    uint8_t  pending_subidx;      // 正在等待的读取子索引
} OID_Data_t;

extern volatile OID_Data_t g_oid_drives[OID_MAX_NODES];

/* ==================================================================== */
/*              API 函数声明                                              */
/* ==================================================================== */

/**
 * @brief 设置电机速度
 * @param node_id CAN 节点 ID (1~4)
 * @param speed_erpm 目标速度 (erpm), 正转正数/反转负数
 */
void OID_Set_Speed(uint8_t node_id, int32_t speed_erpm);

/**
 * @brief 设置加速度 (erpm/s²)
 */
void OID_Set_Accel(uint8_t node_id, int32_t accel_erpmss);

/**
 * @brief 设置减速度 (erpm/s²)
 */
void OID_Set_Decel(uint8_t node_id, int32_t decel_erpmss);

/**
 * @brief 刹车急停
 */
void OID_Brake(uint8_t node_id);

/**
 * @brief 请求读取参数 (0x0F + sub_index 协议)
 */
void OID_Req_Parameter(uint8_t node_id, uint8_t sub_index);

/**
 * @brief 请求实时电转速 (0x0F 0x01, 响应 int32 erpm)
 */
void OID_Req_Actual_Speed(uint8_t node_id);

/**
 * @brief 向指定节点发送心跳 (状态字读取 0x00，DLC=1)
 */
void OID_Heartbeat(uint8_t node_id);

/**
 * @brief 解析 CAN 接收报文 (在 CAN RX 任务中调用)
 */
void OID_Parse_Rx_Message(uint32_t can_id, uint8_t *data, uint8_t dlc);

#endif /* __OID_CAN_H */
