/**
  *******************************************************************************
  * @file           : chassis_control.h
  * @author         : [Your Name/Team]
  * @version        : V1.0
  * @date           : 2026-04-16
  * @brief          : 4轮差速底盘控制程序，支持遥控模式和API模式
  *
  * 主要功能：
  * 1. 遥控模式：处理遥控器输入，支持死区处理和转向补偿
  * 2. API模式：基于(vx, vy, w)输入计算四个轮子的速度
  * 3. 支持4轮差速底盘运动学
  * 4. 轮子参数：直径240mm，左右轴距690mm，前后轮距640mm
  *
  * 注意事项：
  * - vx: 前进速度 (m/s)
  * - vy: 侧向速度 (m/s)
  * - w: 角速度 (rad/s)
  * - 输出单位: turn/s (转每秒)
  *******************************************************************************
  */

#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include "stdint.h"
#include "serial_port.h"

// 遥控模式配置参数
#define RC_DEADZONE 0.1f            // 死区范围
#define RC_MIN_TURN_RESPONSE 0.45f  // 最小转向响应力度

// 轮子参数定义
#define WHEEL_DIAMETER 0.24f    // 轮子直径 (m)
#define WHEEL_RADIUS 0.12f      // 轮子半径 (m)
#define TRACK_WIDTH 0.64f       // 轮距 (m)
#define WHEELBASE 0.69f        // 轴距 (m)
#define PI 3.1415926535f
#define GEAR_RATIO 4.43f        // 电机减速比

#define HOST_CMD_FRAME_ID 0x05
#define HOST_STATUS_FRAME_ID 0x06
#define HOST_CMD_PAYLOAD_LEN 8
#define HOST_STATUS_PAYLOAD_LEN 5


#define API_TIMEOUT_MS 500 // 500ms 没收到上位机指令则停车

// 定义新增的反馈帧 CAN ID
#define CHASSIS_STATUS_CAN_ID    0x506  // 系统状态反馈
#define CHASSIS_ODOM_CAN_ID      0x507  // 线速度、角速度反馈
#define CHASSIS_ENC_RIGHT_CAN_ID 0x508  // 右侧编码器
#define CHASSIS_ENC_LEFT_CAN_ID  0x509  // 左侧编码器
#define CHASSIS_WHEEL_SPEED_CAN_ID 0x50A // 四轮实际线速度反馈


#define MODE_RC          0  // 纯遥控模式
#define MODE_API_STANDBY 1  // API已挂载，但处于锁定状态（动力切断）
#define MODE_API_ACTIVE  2  // API完全接管，动力开启

// ============================================================================
// 驱动器类型选择宏 (条件编译)
// ============================================================================
#define DRIVER_TYPE_ODRIVE 0
#define DRIVER_TYPE_MSSD   1
#define DRIVER_TYPE_OID    2

// 👇 修改这里来切换当前使用的底层驱动器！
//#define MOTOR_DRIVER_USED  DRIVER_TYPE_ODRIVE
//#define MOTOR_DRIVER_USED   DRIVER_TYPE_MSSD
#define MOTOR_DRIVER_USED   DRIVER_TYPE_OID
#if MOTOR_DRIVER_USED == DRIVER_TYPE_ODRIVE
    #include "odrive_can.h"
    extern volatile ODrive_Data_t g_odrive_motors[ODRIVE_MAX_MOTORS];
#elif MOTOR_DRIVER_USED == DRIVER_TYPE_MSSD
    #include "mssd_can.h"
    // 定义美思固驱动器的节点 ID 分配
    #define MSSD_NODE_RIGHT   2  // 节点 3 控制右侧电机 (CH_R=前, CH_L=后)
    #define MSSD_NODE_LEFT    3  // 节点 2 控制左侧电机 (CH_R=前, CH_L=后)
#elif MOTOR_DRIVER_USED == DRIVER_TYPE_OID
    #include "oid_can.h"
    // 定义欧艾迪 OID 驱动器节点 ID 分配 (单驱动器单电机)
    #define OID_NODE_FL    2  // 前左
    #define OID_NODE_FR    1  // 前右
    #define OID_NODE_RL    4  // 后左
    #define OID_NODE_RR    3  // 后右
    // 极对数: erpm = rpm * POLE_PAIRS
    #define OID_POLE_PAIRS  10
#endif
// ============================================================================

extern uint8_t Ctrl_Mode; // 0: 遥控模式, 1: API模式
/**
 * @brief 初始化底盘控制
 */
void chassis_control_init(void);

/**
 * @brief 使用遥控器输入控制底盘运动（遥控模式）
 * @param ly 左摇杆Y轴值 (0-4095)
 * @param rx 右摇杆X轴值 (0-4095)
 */
void set_chassis_rc_control(uint16_t ly, uint16_t rx);
void set_chassis_velocity(float vx, float vy, float omega);
void chassis_control_parse_host_frame(uint8_t frame_id, const uint8_t *payload, uint8_t len);
void chassis_control_send_status_feedback(void);
void Send_Motor_speed(uint8_t axis_id, float speed_rpm, float torque_ff, uint8_t reset_integrator);
void Control_Update(void);
void chassis_control_send_odom_feedback(void);
void chassis_control_req_odrive_data(void);

/**
 * @brief 停止底盘运动
 */
void chassis_stop(void);

#endif // __CHASSIS_CONTROL_H
