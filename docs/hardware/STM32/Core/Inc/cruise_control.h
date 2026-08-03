#ifndef __CRUISE_CONTROL_H
#define __CRUISE_CONTROL_H

#include "main.h"
#include "bsp_imu_solver.h" // 需要用到 IMU 数据

// --- 配置参数 ---
#define CRUISE_SPEED 1.0f            // 巡航基础速度
// #define CRUISE_SPEED_SLOW 0.5f       // 遇障碍物时的慢速
#define U_TURN_SPEED 0.8f            // 掉头速度
#define U_TURN_TARGET_ANGLE 180.0f   // 掉头目标角度
#define U_TURN_ANGLE_TOLERANCE 5.0f // 掉头角度容差
#define JOYSTICK_INTERRUPT_TH 200    // 摇杆介入阈值 (偏离中心值)
#define OBSTACLE_STOP_DIST 30.0f     // 避障急停距离 (cm)

#define JOYSTICK_DEADZONE       300   // 死区 (2048 ± 300 是静止)
#define JOYSTICK_FULL_MIN       300   // 小于 300 算推满左
#define JOYSTICK_FULL_MAX       3800  // 大于 3800 算推满右
// --- 圆弧掉头参数 ---
#define U_TURN_SPEED_HIGH 1.5f // 外侧轮速度 (快)
#define U_TURN_SPEED_LOW -1.0f  // 内侧轮速度 (慢，保持负数以后退)

// --- 前馈电流参数 (单位: A) ---
#define CRUISE_BASE_FF    0.0f  // 直线巡航时的基础前馈 (克服滚动阻力)
#define U_TURN_FF         0.0f // 掉头时的强力前馈 (克服侧向摩擦)
#define REVERSE_BOOST     0.0f //内侧轮反转增强系数

// --- 超声波避障滤波参数 ---
#define OBSTACLE_FILTER_LIMIT 5  // 连续 3 次检测到才触发 (约 150ms)

// 定义掉头时的宽松阈值 (掉头时允许靠得更近)
#define U_TURN_STOP_DIST 31.0f 
// --- PID 参数 ---
#define YAW_PID_KP 0.05f
#define YAW_PID_KI 0.00f
#define YAW_PID_KD 0.00f



// --- 状态枚举 ---
typedef enum
{
    CRUISE_OFF = 0,     // 关闭
    CRUISE_STRAIGHT,    // 直线巡航
    CRUISE_U_TURN_LEFT, // 左掉头中
    CRUISE_U_TURN_RIGHT // 右掉头中
} CruiseState_t;



// --- 函数声明 ---
void Cruise_Init(void);

void Cruise_Process(uint16_t mapped_throttle, uint16_t mapped_steering);
void Cruise_Start(void);             // 开启巡航
void Cruise_Stop(void);              // 停止巡航
CruiseState_t Cruise_GetState(void); // 获取当前状态

#endif /* __CRUISE_CONTROL_H */