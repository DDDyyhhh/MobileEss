// // mower_config.h

// #ifndef __MOWER_CONFIG_H
// #define __MOWER_CONFIG_H

// /* ==================================================================== */
// /*                       1. 遥控器按键与摇杆定义                          */
// /* ==================================================================== */

// // --- 按键在 `key[]` 数组中的索引 ---
// typedef enum {
//     KEY_INDEX_UNLOCK_A          = 0, // 解锁键A (原 key[0])
//     KEY_INDEX_UNLOCK_B          = 1, // 解锁键B (原 key[1])
//     KEY_INDEX_GEAR_1            = 2, // 档位1键 (原 key[2])
//     KEY_INDEX_GEAR_2            = 3, // 档位2键 (原 key[3])
//     KEY_INDEX_GEAR_3            = 4, // 档位3键 (原 key[4])
//     KEY_INDEX_BLADE_SPEED_1     = 5, // 刀盘速度1键 (原 key[5])
//     KEY_INDEX_BLADE_SPEED_2_LIFTER_EN = 6, // 刀盘速度2/升降使能键 (原 key[6])
//     KEY_INDEX_BLADE_SPEED_3     = 7, // 刀盘速度3键 (原 key[7])
// } RemoteKeyIndex_e;

// // --- 摇杆在 `Joystick[]` 数组中的索引 ---
// typedef enum {
//     // 假设 Joystick[0] 未使用或为左摇杆X
//     JOYSTICK_INDEX_LEFT_X       = 0, // 左摇杆X (原 Joystick[0])
//     JOYSTICK_INDEX_THROTTLE     = 1, // 油门/前后 (原 Joystick[1])
//     JOYSTICK_INDEX_STEERING     = 2, // 转向/左右 (原 Joystick[2])
//     JOYSTICK_INDEX_RIGHT_Y      = 3, // 右摇杆Y (原 Joystick[3])
// } RemoteJoystickIndex_e;

// // --- 按键状态值 ---
// #define KEY_STATE_RELEASED       0 // 松开
// #define KEY_STATE_PRESSED        1 // 短按
// #define KEY_STATE_LONG_PRESS     2 // 长按


// /* ==================================================================== */
// /*                       2. 割草机物理参数与行为定义                      */
// /* ==================================================================== */

// // --- 行走速度配置 (单位: ODrive的速度单位, 例如 turns/s) ---
// // 注意：这里的speed值需要根据你的遥控器摇杆行程和电机参数进行映射和调整
// // 这里的档位值可以作为最大速度的系数
// #define MOWER_GEAR_1_MAX_SPEED   1.5f
// #define MOWER_GEAR_2_MAX_SPEED   2.0f
// #define MOWER_GEAR_3_MAX_SPEED   2.5f

// // --- 刀盘电机速度配置 (单位: PWM占空比百分比或其他) ---
// #define BLADE_SPEED_LEVEL_1      30.0f
// #define BLADE_SPEED_LEVEL_2      35.0f
// #define BLADE_SPEED_LEVEL_3      40.0f
// #define BLADE_SPEED_OFF          0.0f




// /* ==================================================================== */
// /*                       3. 系统状态与安全定义                          */
// /* ==================================================================== */

// // --- 状态数组 `receiver_status[]` 索引 ---
// typedef enum {
//     STATUS_INDEX_LOCK          = 0, // 锁定状态
//     STATUS_INDEX_GEAR          = 1, // 当前档位
//     STATUS_INDEX_BLADE_SPEED   = 2, // 当前刀盘速度等级
// } SystemStatusIndex_e;


// //心跳包ID
// #define MOWER_DEVICE_ID 0x02

// #endif // __MOWER_CONFIG_H

