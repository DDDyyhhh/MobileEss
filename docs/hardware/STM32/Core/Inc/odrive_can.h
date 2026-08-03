#ifndef __ODRIVE_CAN_H
#define __ODRIVE_CAN_H

#include <stdint.h>
#include "can.h"

// --- 配置参数 ---
#define ODRIVE_MAX_MOTORS 5 // 最大支持的电机数量 (Node ID: 0~4)

// --- ODrive 数据结构 (每个电机一份) ---
typedef struct
{
    // 实时反馈数据
    float vbus_voltage;
    float vbus_current;
    float encoder_pos; // 单位: Turns
    float encoder_vel; // 单位: Turns/s
    float iq_setpoint;
    float iq_measured;
    float sensorless_pos;
    float sensorless_vel;

    // 错误与状态
    uint32_t axis_error;
    uint8_t axis_state;
    uint32_t motor_error;
    uint32_t encoder_error;

    // 通信诊断
    uint32_t last_heartbeat_tick; // 最后一次收到心跳的时间
    uint8_t is_connected;         // 是否在线
} ODrive_Data_t;

typedef struct
{
    uint32_t stdId;
    uint8_t dlc;
    uint8_t rtr;
    uint8_t data[8];
} CAN_TxMsg_t;

typedef struct
{
    uint32_t stdId;
    uint8_t dlc;
    uint8_t rtr;
    uint8_t ide;
    uint8_t data[8];
} CAN_RxMsg_t;

// --- 全局变量声明 ---
// 使用数组管理所有电机，索引即为 Node ID
extern volatile ODrive_Data_t g_odrive_motors[ODRIVE_MAX_MOTORS];

// --- 命令 ID 定义 (保持不变) ---
#define ODRIVE_MSG_HEARTBEAT 0x001
#define ODRIVE_MSG_ESTOP 0x002
#define ODRIVE_MSG_GET_MOTOR_ERROR 0x003
#define ODRIVE_MSG_GET_ENCODER_ERROR 0x004
#define ODRIVE_MSG_GET_SENSORLESS_ERROR 0x005
#define ODRIVE_MSG_SET_AXIS_NODE_ID 0x006
#define ODRIVE_MSG_SET_AXIS_REQUESTED_STATE 0x007
#define ODRIVE_MSG_GET_ENCODER_ESTIMATES 0x009
#define ODRIVE_MSG_GET_ENCODER_COUNT 0x00A
#define ODRIVE_MSG_SET_CONTROLLER_MODES 0x00B
#define ODRIVE_MSG_SET_INPUT_POS 0x00C
#define ODRIVE_MSG_SET_INPUT_VEL 0x00D
#define ODRIVE_MSG_SET_INPUT_TORQUE 0x00E
#define ODRIVE_MSG_SET_VELOCITY_LIMIT 0x00F
#define ODRIVE_MSG_SET_ANTICOGGING 0x010
#define ODRIVE_MSG_SET_TRAJ_VEL_LIMIT 0x011
#define ODRIVE_MSG_SET_TRAJ_ACCEL_LIMIT 0x012
#define ODRIVE_MSG_SET_TRAJ_INERTIA 0x013
#define ODRIVE_MSG_GET_IQ 0x014
#define ODRIVE_MSG_GET_SENSORLESS_ESTIMATES 0x015
#define ODRIVE_MSG_REBOOT 0x016
#define ODRIVE_MSG_GET_BUS_VOLTAGE_CURRENT 0x017
#define ODRIVE_MSG_CLEAR_ERRORS 0x018

// --- 枚举定义 (保持不变) ---
typedef enum
{
    ODRIVE_STATE_UNDEFINED = 0,
    ODRIVE_STATE_IDLE = 1,
    ODRIVE_STATE_STARTUP_SEQUENCE = 2,
    ODRIVE_STATE_FULL_CALIBRATION_SEQUENCE = 3,
    ODRIVE_STATE_MOTOR_CALIBRATION = 4,
    ODRIVE_STATE_SENSORLESS_CONTROL = 5,
    ODRIVE_STATE_ENCODER_INDEX_SEARCH = 6,
    ODRIVE_STATE_ENCODER_OFFSET_CALIBRATION = 7,
    ODRIVE_STATE_CLOSED_LOOP_CONTROL = 8,
    ODRIVE_STATE_LOCKIN_SPIN = 9,
    ODRIVE_STATE_ENCODER_DIR_FIND = 10,
    ODRIVE_STATE_HOMING = 11,
    // ODRIVE_STATE_ENCODER_HALL_POLARITY_CALIB = 11,
    // ODRIVE_STATE_ENCODER_HALL_PHASE_CALIB = 12
} odrive_state_t;

typedef enum
{
    ODRIVE_CTRL_MODE_VOLTAGE_CONTROL = 0,
    ODRIVE_CTRL_MODE_TORQUE_CONTROL = 1,
    ODRIVE_CTRL_MODE_VELOCITY_CONTROL = 2,
    ODRIVE_CTRL_MODE_POSITION_CONTROL = 3
} odrive_control_mode_t;

typedef enum
{
    ODRIVE_INPUT_MODE_INACTIVE = 0,
    ODRIVE_INPUT_MODE_PASSTHROUGH = 1,
    ODRIVE_INPUT_MODE_VEL_RAMP = 2,
    ODRIVE_INPUT_MODE_POS_FILTER = 3,
    ODRIVE_INPUT_MODE_MIX_CHANNELS = 4,
    ODRIVE_INPUT_MODE_TRAP_TRAJ = 5,
    ODRIVE_INPUT_MODE_TORQUE_RAMP = 6,
    ODRIVE_INPUT_MODE_MIRROR = 7,
    // ODRIVE_INPUT_MODE_TUNING = 8
} odrive_input_mode_t;

// --- 函数声明 ---
void User_CAN_Init(void); // 初始化 CAN 过滤器

// 基础发送
uint8_t odrive_can_send_message(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t length, uint8_t is_remote);

// 控制命令
uint8_t odrive_estop(CAN_HandleTypeDef *hcan, uint8_t node_id);
uint8_t odrive_set_axis_state(CAN_HandleTypeDef *hcan, uint8_t node_id, odrive_state_t state);
uint8_t odrive_clear_errors(CAN_HandleTypeDef *hcan, uint8_t node_id);
uint8_t odrive_set_controller_modes(CAN_HandleTypeDef *hcan, uint8_t node_id, odrive_control_mode_t control_mode, odrive_input_mode_t input_mode);
uint8_t odrive_reboot(CAN_HandleTypeDef *hcan, uint8_t node_id);
// 运动控制
uint8_t odrive_set_input_pos(CAN_HandleTypeDef *hcan, uint8_t node_id, float position, float velocity_ff, float torque_ff);
uint8_t odrive_set_input_vel(CAN_HandleTypeDef *hcan, uint8_t node_id, float velocity, float torque_ff);
uint8_t odrive_set_input_torque(CAN_HandleTypeDef *hcan, uint8_t node_id, float torque);

// 数据请求 (远程帧)
uint8_t odrive_req_motor_error(CAN_HandleTypeDef *hcan, uint8_t node_id);
uint8_t odrive_req_encoder_error(CAN_HandleTypeDef *hcan, uint8_t node_id);
uint8_t odrive_req_sensorless_error(CAN_HandleTypeDef *hcan, uint8_t node_id);
uint8_t odrive_req_vbus_voltage(CAN_HandleTypeDef *hcan, uint8_t node_id);
uint8_t odrive_req_encoder_estimates(CAN_HandleTypeDef *hcan, uint8_t node_id);
uint8_t odrive_req_heartbeat(CAN_HandleTypeDef *hcan, uint8_t node_id);
uint8_t odrive_req_encoder_count(CAN_HandleTypeDef *hcan, uint8_t node_id);
uint8_t odrive_req_iq(CAN_HandleTypeDef *hcan, uint8_t node_id);
uint8_t odrive_req_sensorless_estimates(CAN_HandleTypeDef *hcan, uint8_t node_id);

// 辅助功能
uint8_t odrive_init_closed_loop(CAN_HandleTypeDef *hcan, uint8_t node_id);

// 声明函数

uint8_t CAN_Enqueue_Message(uint32_t id, uint8_t *data, uint8_t len, uint8_t is_remote);
#endif
