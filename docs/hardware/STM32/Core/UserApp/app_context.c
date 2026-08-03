/**
  ******************************************************************************
  * @file           : app_context.c
  * @brief          : 系统全局运行环境上下文，定义并初始化所有跨任务共享变量
  * 
  * 主要功能：
  * 1. 实例化 `g_app` (AppContext_t) 结构体
  * 2. 初始化整车默认安全状态（默认上锁、默认触发防撞保护、刀盘关闭等）
  * 3. 集中管理传感器数据 (超声波距离、IMU、BMS电量) 及控制指令 (摇杆、按键)
  *
  * 注意事项：
  * - 此文件中的 `g_app` 为全系统唯一的数据集散中心
  * - 在多任务（FreeRTOS）环境下，如果涉及 32-bit 以上数据的拆分读写，
  *   需评估是否增加 Mutex（互斥锁）或 taskENTER_CRITICAL() 进行保护
  * - 默认初始化策略为“最安全状态”（lock=1, latched=1），必须通过特定序列才能解锁
  ******************************************************************************
  */

#include "app_context.h"
// #include "mower_config.h" // 引入宏定义


// ==============================================================================
// 2. 定义并初始化全局运行时数据实例
// ==============================================================================
AppContext_t g_app = {
    .lock = 1,
    .Unlock_tips = 0,
    .g_collision_latched = 1,
    .unlock_stage1_complete = false,
    .g_system_report_state = REPORT_INIT_SUCCESS, // 初始化成功，等待解锁
    
    .communication_connected = false,
    .last_receive_timestamp = 0,
    .Joystick = {2048, 2048, 2048, 2048},
    .key = {0},
    
    .user_set_speed =  MOWER_GEAR_1_MAX_SPEED,
    .current_ramped_speed = 0.0f,
    .speed = 0.0f,
    .turn_speed = CRUISE_GEAR_1_SPEED,
    .is_on_slope = 1,
    .ultrasonic_enabled = true, 
    
    .d1 = 250.0f, .d2 = 250.0f, .d3 = 250.0f, .d4 = 250.0f,
    .voltage_int = 0xFF,
		
    .grass_bag_status = false,
    
    .receiver_status = { [1] = 1, [2] = 1 }, // 默认速度档位1，割刀关闭
    
    .last_imu_tick = 0,
    .g_lifter_enable_key_was_pressed = 0,

    .odrive_temps = {25, 25, 25, 25}, // 默认室温 25度

    // --- BMS 最高温度 ---
    // 单位：摄氏度 (整数)
    .bms_temp = 0, // 初始化为 0 度

    .cfg_ultrasonic_en = 1,    // 1:开, 0:关
    .cfg_collision_en = 1,     // 1:开, 0:关
    .cfg_reverse_limit_en = 1, // 1:开, 0:关
    .cfg_tilt_sensor_en = 1,   // 1:开, 0:关
    .cfg_op_mode = 0,          // 操作模式 (0, 1, 2, 3)

    .trigger_blade_init = 0,
    .trigger_cruise_req = 0

};
