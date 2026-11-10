#ifndef APP_CONTEXT_H
#define APP_CONTEXT_H

#include "main.h"
#include <stdbool.h>
#include "icm42688.h" // 引入 IMU 结构体
#include "heartbeat.h"

// ==============================================================================
// 1. 系统配置参数 (来自 mower_config.h)
// ==============================================================================
/* ==================================================================== */
/*                        遥控器按键与摇杆定义                          */
/* ==================================================================== */
#define SPEED_RAMP_UP_STEP 0.1f
#define SPEED_RAMP_DOWN_STEP 0.5f
// --- 按键在 `key[]` 数组中的索引 ---
typedef enum
{
    KEY_INDEX_UNLOCK_A = 0,                // 解锁键A (原 key[0])
    KEY_INDEX_UNLOCK_B = 1,                // 解锁键B (原 key[1])
    KEY_INDEX_GEAR_1 = 2,                  // 档位1键 (原 key[2])
    KEY_INDEX_GEAR_2 = 3,                  // 档位2键 (原 key[3])
    KEY_INDEX_GEAR_3 = 4,                  // 档位3键 (原 key[4])
    KEY_INDEX_BLADE_SPEED_1 = 5,           // 刀盘速度1键 (原 key[5])
    KEY_INDEX_BLADE_SPEED_2_LIFTER_EN = 6, // 刀盘速度2/升降使能键 (原 key[6])
    KEY_INDEX_BLADE_SPEED_3 = 7,           // 刀盘速度3键 (原 key[7])
} RemoteKeyIndex_e;

// --- 摇杆在 `Joystick[]` 数组中的索引 ---
typedef enum
{
    // 假设 Joystick[0] 未使用或为左摇杆X
    JOYSTICK_INDEX_LEFT_X = 0,   // 左摇杆X (原 Joystick[0])
    JOYSTICK_INDEX_THROTTLE = 1, // 油门/前后 (原 Joystick[1])
    JOYSTICK_INDEX_STEERING = 2, // 转向/左右 (原 Joystick[2])
    JOYSTICK_INDEX_RIGHT_Y = 3,  // 右摇杆Y (原 Joystick[3])
} RemoteJoystickIndex_e;

// 定义系统状态
typedef enum
{
    // --- 0: 正常/手动模式 ---
    REPORT_NONE = 0,

    // --- 1~9: 严重故障 (会导致停车/红灯) ---
    REPORT_FAULT_COLLISION = 1,     // 碰撞杆触发
    REPORT_FAULT_EMERGENCY_BTN = 2, // 急停按钮触发
    REPORT_FAULT_TILT = 3,          // 倾翻触发
    REPORT_FAULT_COMM_LOST = 4,     // 通信丢失
    REPORT_FAULT_COMM_SUCCESS = 5,  // 连接成功
    REPORT_FAULT_COMM_ING = 6,      // 连接中
    REPORT_INIT_SUCCESS = 7,        // 初始化成功，等待解锁

    // --- 倒草相关状态 ---
    REPORT_DUMPING_START = 8, // 倒草模式 (正在打开 或 保持打开)
    REPORT_DUMPING_DONE = 9,  // 倒草完成 (刚关闭)
    

    // --- 10~19: 巡航工作状态 (正常运行/绿灯或蓝灯) ---
    REPORT_CRUISE_STRAIGHT = 10, // 直线巡航
    REPORT_CRUISE_LEFT = 11,     // 左转/左掉头中
    REPORT_CRUISE_RIGHT = 12,     // 右转/右掉头中

    // --- 倒草相关状态 ---
    REPORT_DUMPING_CLOSING = 13,        //  草袋正在关闭
    REPORT_FAULT_BAG_FULL = 14,         // 草袋已集满


    REPORT_BLADE_INITIALIZING = 15,     // 正在进行刀盘高度初始化
    REPORT_FAULT_BAG_DISCONNECTED = 16 // 草袋未连接
    
    
} SystemReportState_t;

// --- 按键状态值 ---
#define KEY_STATE_RELEASED 0   // 松开
#define KEY_STATE_PRESSED 1    // 短按
#define KEY_STATE_LONG_PRESS 2 // 长按

/* ==================================================================== */
/*                       割草机物理参数与行为定义                      */
/* ==================================================================== */

// --- 行走速度配置 (单位: ODrive的速度单位, 例如 turns/s) ---
// 注意：这里的speed值需要根据你的遥控器摇杆行程和电机参数进行映射和调整
// 这里的档位值可以作为最大速度的系数
// #define MOWER_GEAR_1_MAX_SPEED 1.6f
// #define MOWER_GEAR_2_MAX_SPEED 2.1f
// #define MOWER_GEAR_3_MAX_SPEED 2.6f

#define MOWER_GEAR_1_MAX_SPEED 1.15f//行驶速度
#define MOWER_GEAR_2_MAX_SPEED 1.45f
#define MOWER_GEAR_3_MAX_SPEED 1.8f

// #define MOWER_GEAR_1_MAX_SPEED 0.2f//行驶速度
// #define MOWER_GEAR_2_MAX_SPEED 0.6f
// #define MOWER_GEAR_3_MAX_SPEED 0.8f

#define MOWER_UC_1_MAX_SPEED 0.0f //超声波限速
#define MOWER_UC_2_MAX_SPEED 0.3f
#define MOWER_UC_3_MAX_SPEED 0.5f

#define MOWER_GEAR_1_MAX_TURN_SPEED 1.0f
#define MOWER_GEAR_2_MAX_TURN_SPEED 0.9f
#define MOWER_GEAR_3_MAX_TURN_SPEED 0.9f

// #define MOWER_GEAR_1_MAX_TURN_SPEED 4.0f
// #define MOWER_GEAR_2_MAX_TURN_SPEED 6.0f
// #define MOWER_GEAR_3_MAX_TURN_SPEED 8.0f


// #define MOWER_GEAR_1_MAX_TURN_SPEED 8.0f
// #define MOWER_GEAR_2_MAX_TURN_SPEED 8.5f
// #define MOWER_GEAR_3_MAX_TURN_SPEED 9.5f

// --- 刀盘电机速度配置 (单位: PWM占空比百分比或其他) ---
// #define BLADE_SPEED_LEVEL_1 45.0f
// #define BLADE_SPEED_LEVEL_2 50.0f
// #define BLADE_SPEED_LEVEL_3 55.0f

#define BLADE_SPEED_LEVEL_1 14000.0f   // 1档: 2800 RPM * 5对极 = 14000 ERPM
#define BLADE_SPEED_LEVEL_2 15500.0f  // 2档: 3100 RPM * 5对极 = 15500 ERPM
#define BLADE_SPEED_LEVEL_3 16000.0f  // 3档: 3200 RPM * 5对极 = 16000 ERPM

// #define BLADE_SPEED_LEVEL_1 10.0f
// #define BLADE_SPEED_LEVEL_2 20.0f
// #define BLADE_SPEED_LEVEL_3 30.0f

#define BLADE_SPEED_OFF 0.0f

// ==========================================
// 巡航模式参数定义
// ==========================================
// 定义三个挡位的巡航直线速度 (单位：RPM 或 m/s，取决于你的底层映射)
// #define CRUISE_GEAR_1_SPEED   1.0f  // 1档慢速
// #define CRUISE_GEAR_2_SPEED   1.5f  // 2档中速
// #define CRUISE_GEAR_3_SPEED   2.0f  // 3档快速

#define CRUISE_GEAR_1_SPEED   0.7f  // 1档慢速
#define CRUISE_GEAR_2_SPEED   1.0f  // 2档中速
#define CRUISE_GEAR_3_SPEED   1.5f  // 3档快速

// 避障降速时的安全速度 (保持不变)
// #define CRUISE_SPEED_SLOW     0.4f
#define CRUISE_SPEED_SLOW     1.2f

/* ==================================================================== */
/*                       系统状态与安全定义                          */
/* ==================================================================== */

// --- 状态数组 `receiver_status[]` 索引 ---
typedef enum
{
    STATUS_INDEX_LOCK = 0,        // 锁定状态
    STATUS_INDEX_GEAR = 1,        // 当前档位
    STATUS_INDEX_BLADE_SPEED = 2, // 当前刀盘速度等级
} SystemStatusIndex_e;

// 心跳包ID
#define MOWER_DEVICE_ID 0x02

// ==============================================================================
// 2. 运行时全局上下文
// ==============================================================================
typedef struct
{
    // --- 安全与状态 ---
    uint8_t lock;                              // 原: lock (0:解锁, 1:上锁)
    uint8_t Unlock_tips;                       // 开锁提示弹窗 0关闭 1开启
    uint8_t g_collision_latched;               // 原: g_collision_latched (0:正常, 1:锁存)
    bool unlock_stage1_complete;               // 原: unlock_stage1_complete
    SystemReportState_t g_system_report_state; // 原: g_system_report_state

    // --- 遥控器数据 (实时更新) ---
    uint16_t Joystick[4]; // 原: Joystick
    uint8_t key[8];       // 原: key
    // --- 旋钮状态 ---
    uint8_t knob_gear;               // 行走速度挡位 (0-3)
    uint8_t knob_blade;              // 割刀速度挡位 (0-3)
    uint8_t knob_height;             // 高度挡位 (0-5)
    bool communication_connected;    // 原: communication_connected
    uint32_t last_receive_timestamp; // 原: last_receive_timestamp

    // --- 车辆动态数据 ---
    float user_set_speed;       // 原: user_set_speed (根据档位设定的速度)
    float current_ramped_speed; // 原: current_ramped_speed (平滑+避障后)
    float speed;                // 原: speed (最终给电机的速度, 等于 current_ramped_speed)
    float turn_speed;           // 原: turn_speed
    uint8_t is_on_slope;        // 原: is_on_slope

    // 超声波避障使能标志 (true:开启, false:关闭)
    bool ultrasonic_enabled;

    // --- 传感器数据 ---
    float d1, d2, d3, d4; // 原: d1, d2, d3, d4 (超声波)
    uint8_t voltage_int;  // 原: voltage_int (BMS SOC)

    // --- IMU 数据 (直接内嵌) ---
    icm42688RealData_t accval;  // 原: accval
    icm42688RealData_t gyroval; // 原: gyroval

    // --- 执行器状态 ---
    bool grass_bag_status; // 原: grass_bag_status // false = 关闭 (0), true = 打开 (1)

    // --- 状态回传数组 ---
    uint8_t receiver_status[13]; // 原: receiver_status

    // --- 驱动/任务内部变量 (不建议放在全局，但为了兼容暂时保留) ---
    uint32_t last_imu_tick;                  // 原: last_imu_tick
    uint8_t g_lifter_enable_key_was_pressed; // 原: g_lifter_enable_key_was_pressed

    // --- ODrive/MSSD   4 个轴的 FET 温度 ---
    // 索引映射: [0]右前(UART1_Ax0), [1]右后(UART1_Ax1), [2]左前(UART2_Ax0), [3]左后(UART2_Ax1)
    uint8_t odrive_temps[4];

    // --- 4 个轴的实时电功率 (单位：10W，即值 15 代表 150W) ---
    uint8_t motor_powers[4];
    // --- BMS 最高温度 ---
    // 单位：摄氏度 (整数)
    uint8_t bms_temp;


    // ============================================================
    // <<<---  来自遥控器触摸屏的系统设置 --->>>
    // ============================================================
    uint8_t cfg_ultrasonic_en;    // 1:开, 0:关
    uint8_t cfg_collision_en;     // 1:开, 0:关
    uint8_t cfg_reverse_limit_en; // 1:开, 0:关
    uint8_t cfg_tilt_sensor_en;   // 1:开, 0:关
    uint8_t cfg_lawn_protect_en;  // 1:开, 0:关

    uint8_t cfg_op_mode;          // 操作模式 (0, 1, 2, 3)
    //api开关
    uint8_t cfg_api_enable ;   // 1:开, 0:关

    volatile uint8_t trigger_blade_init; // 刀盘初始化脉冲信号
    volatile uint8_t trigger_cruise_req; // 开启巡航脉冲信号

    // ==========================================
    // <<<---  草袋真实物理标志位 --->>>
    // ==========================================
    uint8_t is_bag_full;       // 1: 满载, 0: 未满
    uint8_t is_bag_connected;  // 1: 已连接, 0: 未连接

    

} AppContext_t;

// ==============================================================================
// 3. 全局实例声明
// ==============================================================================

extern AppContext_t g_app; // 全局运行时数据实例

#endif // APP_CONTEXT_H