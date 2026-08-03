// /**
//  *******************************************************************************
//  * @file           : chassis_control.c
//  * @author         : [Your Name/Team]
//  * @version        : V1.0
//  * @date           : 2026-04-16
//  * @brief          : 4轮差速底盘控制程序，支持遥控模式和API模式
//  *
//  * 主要功能�?
//  * 1. 遥控模式：处理遥控器输入，支持死区处理和转向补偿
//  * 2. API模式：基�?vx, vy, w)输入计算四个轮子的速度
//  * 3. 支持4轮差速底盘运动学（vy默认忽略�?
//  * 4. 轮子参数：直�?40mm，轴�?75mm，轮�?50mm
//  *
//  * 注意事项�?
//  * - vx: 前进速度 (m/s)
//  * - vy: 侧向速度 (m/s)
//  * - w: 角速度 (rad/s)
//  * - 输出单位: turn/s (转每�?
//  *******************************************************************************
//  */
// #include <stdint.h>
// #include "stm32f4xx_hal.h"
// #include "chassis_control.h"
// #include "app_context.h" // 包含全局上下文，获取用户设定的速度
// #include "serial_port.h"
// #include "cruise_control.h"
// #include "connet_check.h"
// #include "BladeMotor.h"
// #include <math.h>
// #include <stdlib.h>
// #include <string.h>
// #include "odrive_can.h" // 包含ODrive CAN通信函数
// uint8_t Ctrl_Mode = 0;  // 0: 遥控模式, 1: API模式
// float host_vx = 0.0f;
// static float host_vy = 0.0f;
// float host_omega = 0.0f;
// static uint8_t host_blade_speed_pct = 0;
// static uint8_t host_lifter_gear = 1;
// static uint8_t host_bag_control = 0;
// //  API 上位机心跳时间戳
// uint32_t last_host_cmd_tick = 0;
// uint8_t api_lockout = 0; // API 锁存标志位：一旦遥控器摇杆动了，置 1 锁死 API，直到遥控器屏幕下发解除指令
// extern AppContext_t g_app;
// extern volatile ODrive_Data_t g_odrive_motors[ODRIVE_MAX_MOTORS];
// uint16_t mapped_throttle = 2048;
// uint16_t mapped_steering = 2048;
// /**
//  * @brief 初始化底盘控�?
//  */
// void chassis_control_init(void)
// {
//     last_host_cmd_tick = HAL_GetTick();
// }

// /**
//  * @brief 解析上位机控制帧（HOST_CMD_FRAME_ID = 0x05）
//  *
//  * 控制帧格式（标准帧，DLC=8，当前使用前7字节）：
//  * Byte0: ctrl_mode      bit0=0:RC模式, bit0=1:API模式
//  * Byte1: vx_raw         int8，线速度指令，单位 0.1 m/s
//  * Byte2: vy_raw         int8，预留（差速底盘当前忽略）
//  * Byte3: omega_raw      int8，角速度指令，单位 0.1 rad/s
//  * Byte4: blade_speed_pct uint8，割刀挡位 0-3
//  * Byte5: lifter_gear    uint8，升降档位 1~5（越界会钳位）
//  * Byte6: bag_control    uint8，0=关，非0=开
//  * Byte7: reserve        预留
//  *
//  * 解析规则：
//  * host_vx    = vx_raw * 0.1f;      // m/s
//  * host_omega = omega_raw * 0.1f;   // rad/s
//  * host_vy    = 0.0f;               // 当前不支持侧移
//  *
//  * 示例1：前进 0.1m/s，无旋转，API模式
//  * ID=0x05, DLC=8, DATA=01 01 00 00 00 01 00 00
//  *
//  * 示例2：后退 0.5m/s，右转 0.3rad/s
//  * ID=0x05, DLC=8, DATA=01 FB 00 03 00 01 00 00
//  * 说明：FB(int8)=-5 => -0.5m/s，03 => +0.3rad/s
//  */
// void chassis_control_parse_host_frame(uint8_t frame_id, const uint8_t *payload, uint8_t len)
// {
//     if (frame_id != HOST_CMD_FRAME_ID || payload == NULL || len < HOST_CMD_PAYLOAD_LEN)
//     {
//         return;
//     }

//     last_host_cmd_tick = HAL_GetTick();

//     if (payload[7] == 0xAA)
//     {
//         g_app.g_collision_latched = 0;
//         g_app.lock = 0;
//         // 恢复绿灯
//     }

//     uint8_t host_cmd = payload[0];

//     // ============================================================
//     // 【三级跳状态机逻辑】
//     // ============================================================
//     if (host_cmd == 1)
//     {
//         // --- 动作：申请进入 API 待机 ---
//         // 只要上位机发 1，无论当前在什么状态，都强制回到 API 待机状态
//         // 并且【关键】：在这里清空遥控器的拦截锁 (api_lockout)
//         Ctrl_Mode = 1;
//         api_lockout = 0;
//     }
//     else if (host_cmd == 2)
//     {
//         // --- 动作：申请开启 API 动力 ---
//         // 只有当前已经在状态 1 (待机)，且没有被遥控器拦截时，才允许跳到状态 2
//         if (Ctrl_Mode == 1 && api_lockout == 0)
//         {
//             Ctrl_Mode = 2;
//         }
//         // 如果当前是状态 0 或有 lockout，发 2 是无效的，车子不会动
//     }
//     else if (host_cmd == 0)
//     {
//         // --- 动作：主动退回遥控 ---
//         Ctrl_Mode = 0;
//         api_lockout = 0;
//     }

//     int8_t vx_raw = (int8_t)payload[1];
//     int8_t vy_raw = (int8_t)payload[2];
//     int8_t omega_raw = (int8_t)payload[3];
//     uint8_t blade_speed_pct = payload[4];
//     uint8_t lifter_gear = payload[5];
//     uint8_t bag_control = payload[6];

//     host_blade_speed_pct = (blade_speed_pct > 3) ? 3 : blade_speed_pct;
//     host_lifter_gear = (lifter_gear < 1) ? 1 : (lifter_gear > 5 ? 5 : lifter_gear);
//     host_bag_control = (bag_control != 0) ? 1 : 0;

//     g_app.grass_bag_status = host_bag_control;
//     g_app.knob_height = host_lifter_gear;

//     // g_app.knob_blade = (blade_speed_pct < 0) ? 0 : (blade_speed_pct > 3 ? 3 : blade_speed_pct); // 粗略映射�?0-3 档位

//     // --- API 模式状态同步覆盖 ---
//     if (Ctrl_Mode == 1)
//     {
//         // 强制覆盖全局应用状态
//         g_app.grass_bag_status = host_bag_control;
//         g_app.knob_height = host_lifter_gear;
//         g_app.knob_blade = host_blade_speed_pct;

//         // 【关键】同步给反馈状态缓冲区，让遥控器屏幕显示 API 设定的真实挡位
//         // 注意：根据你 pairing_receiver.c 的协议，receiver_status[0] 是割刀状态
//         g_app.receiver_status[STATUS_INDEX_LOCK] = host_blade_speed_pct;
//         g_app.receiver_status[STATUS_INDEX_BLADE_SPEED] = host_blade_speed_pct;
//     }

//     host_vx = (float)vx_raw * 0.1f;       // 假设输入范围 -10 到 +10 对应 -1.0 m/s 到 +1.0 m/s
//     host_vy = 0.0f;                       // 差速底盘不支持侧向移动
//     host_omega = (float)omega_raw * 0.1f; // 假设输入范围 -10 到 +10 对应 -1.0 rad/s 到 +1.0 rad/s
// }

// /**
//  * @brief 发送底盘状态反馈帧
//  * */
// void chassis_control_send_status_feedback(void)
// {
//     static uint32_t last_feedback_tick = 0;
//     uint32_t now_tick = HAL_GetTick();
//     if ((now_tick - last_feedback_tick) < 20U)
//     {
//         return;
//     }
//     last_feedback_tick = now_tick;

//     uint8_t tx_data[6];
//     tx_data[0] = Ctrl_Mode;
//     tx_data[1] = g_app.bms_temp;
//     tx_data[2] = (g_app.voltage_int > 100) ? 100 : g_app.voltage_int;

//     uint8_t bag_status = 0;
//     if (g_app.grass_bag_status)
//         bag_status |= 0x01;
//     if (g_app.is_bag_connected)
//         bag_status |= 0x02;
//     if (g_app.is_bag_full)
//         bag_status |= 0x04;
//     tx_data[3] = bag_status;
//     tx_data[4] = (g_app.key[0] & 0x03) | ((g_app.key[1] & 0x03) << 2) | ((g_app.key[2] & 0x03) << 4) | ((g_app.key[3] & 0x03) << 6);
//     tx_data[5] = (g_app.key[4] & 0x03) | ((g_app.key[5] & 0x03) << 2) | ((g_app.key[6] & 0x03) << 4) | ((g_app.key[7] & 0x03) << 6);

//     odrive_can_send_message(&hcan1, CHASSIS_STATUS_CAN_ID, tx_data, sizeof(tx_data), 0);
// }

// /**
//  * @brief 向 4 个 ODrive 节点手动请求编码器数据 (兼容 0.5.1 固件)
//  * @note  发送的是 RTR 远程帧，数据会在随后的 CAN RX 中断中自动更新到 g_odrive_motors 数组
//  */
// void chassis_control_req_odrive_data(void)
// {
//     static uint32_t last_req_tick = 0;
//     uint32_t now_tick = HAL_GetTick();

//     // 限制轮询频率为 50Hz (每 20ms 请求一次)，防止 CAN 总线拥堵
//     if ((now_tick - last_req_tick) < 20U)
//         return;
//     last_req_tick = now_tick;

//     // 依次向 4 个电机发送获取编码器估算值 (0x09) 的 RTR 请求
//     // 假设你的 4 个轮毂电机 Node ID 分别是 1, 2, 3, 4
//     odrive_req_encoder_estimates(&hcan1, 1);
//     odrive_req_encoder_estimates(&hcan1, 2);
//     odrive_req_encoder_estimates(&hcan1, 3);
//     odrive_req_encoder_estimates(&hcan1, 4);

//     // 注：你的 odrive_can_send_message 里已经写了“邮箱满等待逻辑”，
//     // 所以连发 4 帧不会因为 STM32 只有 3 个发送邮箱而导致丢包，非常安全。
// }

// /**
//  * @brief 计算真实里程计并发送 (线速度、角速度、编码器累计位置)
//  * @note 频率限制为 20ms (50Hz) 发送一次，防止挤占 CAN 总线带宽
//  */
// void chassis_control_send_odom_feedback(void)
// {
//     static uint32_t last_odom_tick = 0;
//     uint32_t now_tick = HAL_GetTick();

//     if ((now_tick - last_odom_tick) < 20U)
//         return;
//     last_odom_tick = now_tick;

//     float tx_vx = 0.0f;
//     float tx_omega = 0.0f;

//     // ============================================================
//     // 【核心修改】：根据当前模式选择反馈的数据源
//     // ============================================================
//     if (Ctrl_Mode == 1 || Ctrl_Mode == 2)
//     {
//         // API 模式下，直接反馈上位机发来的设定值 (Echo)
//         tx_vx = host_vx;
//         tx_omega = host_omega;
//     }
    
//     // -------------------------------------------------------------
//     // 3. 打包发送 CAN 数据帧 (ID: 0x507)
//     // -------------------------------------------------------------
//     uint8_t tx_buf[8];

//     // Byte 0-3: tx_vx (float)
//     // Byte 4-7: tx_omega (float)
//     memcpy(&tx_buf[0], &tx_vx, 4);
//     memcpy(&tx_buf[4], &tx_omega, 4);

//     odrive_can_send_message(&hcan1, CHASSIS_ODOM_CAN_ID, tx_buf, 8, 0);
//     // --- 帧 0x08: 右侧编码器绝对位置 (Turns) ---
//     // float pos_fr = g_odrive_motors[1].encoder_pos;
//     // float pos_rr = g_odrive_motors[2].encoder_pos;
//     // memcpy(&tx_buf[0], &pos_fr, 4);
//     // memcpy(&tx_buf[4], &pos_rr, 4);
//     // odrive_can_send_message(&hcan1, CHASSIS_ENC_RIGHT_CAN_ID, tx_buf, 8, 0);

//     // // --- 帧 0x09: 左侧编码器绝对位置 (Turns) ---
//     // // 左侧取反，保证前进时所有编码器位置都是单调递增的，方便上位机做里程计
//     // float pos_fl = -g_odrive_motors[3].encoder_pos;
//     // float pos_rl = -g_odrive_motors[4].encoder_pos;
//     // memcpy(&tx_buf[0], &pos_fl, 4);
//     // memcpy(&tx_buf[4], &pos_rl, 4);
//     // odrive_can_send_message(&hcan1, CHASSIS_ENC_LEFT_CAN_ID, tx_buf, 8, 0);
// }

// void Send_Motor_speed(uint8_t axis_id, float speed_rpm, float torque_ff, uint8_t reset_integrator)
// {
//     // 1. 单位转换: Turns/s
//     float vel_rps = speed_rpm;
//     odrive_set_input_vel(&hcan1, axis_id, vel_rps, torque_ff);
// }

// void set_chassis_velocity(float vx, float vy, float omega)
// {
//     // 四轮差速底盘不支持侧向移动，强制vy=0
//     vy = 0.0f;
//     // 计算轮子到中心的距离 (用于角速度转换)
//     float R = sqrtf((WHEELBASE / 2.0f) * (WHEELBASE / 2.0f) + (TRACK_WIDTH / 2.0f) * (TRACK_WIDTH / 2.0f));

//     // 计算各轮子线速度 (m/s)
//     // 4轮差速底盘运动学公式
//     float v_fl = vx - vy - omega * R; // 前左�?
//     float v_fr = vx + vy + omega * R; // 前右�?
//     float v_rl = vx + vy - omega * R; // 后左�?
//     float v_rr = vx - vy + omega * R; // 后右�?

//     // 转换为RPM (转每分钟)
//     // RPM = (线速度 m/s * 60�? / (圆周�?m)
//     float rpm_fl = (v_fl * 60.0f) / (PI * WHEEL_DIAMETER);
//     float rpm_fr = (v_fr * 60.0f) / (PI * WHEEL_DIAMETER);
//     float rpm_rl = (v_rl * 60.0f) / (PI * WHEEL_DIAMETER);
//     float rpm_rr = (v_rr * 60.0f) / (PI * WHEEL_DIAMETER);

//     // 转换为turn/s (车轮转�?，再换算到减速后电机轴转�?
//     float turns_fl = rpm_fl / 60.0f;
//     float turns_fr = rpm_fr / 60.0f;
//     float turns_rl = rpm_rl / 60.0f;
//     float turns_rr = rpm_rr / 60.0f;

//     float motor_turns_fl = turns_fl * GEAR_RATIO;
//     float motor_turns_fr = turns_fr * GEAR_RATIO;
//     float motor_turns_rl = turns_rl * GEAR_RATIO;
//     float motor_turns_rr = turns_rr * GEAR_RATIO;

//     // 发送到电机
//     // 电机映射: 0-前右, 1-后右, 2-前左, 3-后左
//     // 根据现有代码，右侧电机速度取反
//     Send_Motor_speed(1, motor_turns_fr, 0.0f, 0);  // 前左，取�?
//     Send_Motor_speed(2, motor_turns_rr, 0.0f, 0);  // 后左，取�?
//     Send_Motor_speed(3, -motor_turns_fl, 0.0f, 0); // 前右，正�?
//     Send_Motor_speed(4, -motor_turns_rl, 0.0f, 0); // 后右，正�?
// }

// /**
//  * @brief 停止底盘运动
//  */
// void chassis_stop(void)
// {
//     // 发送零速度到所有电�?
//     Send_Motor_speed(1, 0.0f, 0.0f, 0);
//     Send_Motor_speed(2, 0.0f, 0.0f, 0);
//     Send_Motor_speed(3, 0.0f, 0.0f, 0);
//     Send_Motor_speed(4, 0.0f, 0.0f, 0);
// }

// /**
//  * @brief 使用遥控器输入控制底盘运动（遥控模式�?
//  * @param ly 左摇杆Y轴�?(0-4095)
//  * @param rx 右摇杆X轴�?(0-4095)
//  */
// void set_chassis_rc_control(uint16_t ly, uint16_t rx)
// {
//     // 1. 归一化输�?(-1.0 ~ 1.0)
//     float forward_in = ((float)ly - 2048.0f) / 2048.0f;  // 霍尔摇杆
//     float turn_in = ((float)2048 - (float)rx) / 2048.0f; // 减速器轮子

//     // 2. 死区处理
//     if (fabsf(forward_in) < RC_DEADZONE)
//         forward_in = 0.0f;

//     // ============================================================
//     // <<<---  转向起步跃变补偿 (克服草地静摩�? --->>>
//     // ============================================================
//     // 设定最小转向响应力�?(0.35 表示只要轻推，瞬间输�?35% 的转向力�?
//     // 如果觉得起步还是肉，改大�?0.45f；如果起步太窜，减小�?0.25f

//     if (fabsf(turn_in) <= RC_DEADZONE)
//     {
//         turn_in = 0.0f; // 在死区内，绝对静�?
//     }
//     else
//     {
//         // 提取方向符号 (1.0 为右�?1.0 为左)
//         float sign = (turn_in > 0.0f) ? 1.0f : -1.0f;

//         // 计算超出死区后的有效行程比例 (0.0 ~ 1.0)
//         float active_ratio = (fabsf(turn_in) - RC_DEADZONE) / (1.0f - RC_DEADZONE);

//         // 重新映射：将 0.0~1.0 的行程，映射�?0.35~1.0 的超强响应区�?
//         turn_in = sign * (RC_MIN_TURN_RESPONSE + active_ratio * (1.0f - RC_MIN_TURN_RESPONSE));
//         if (turn_in > 1.0f)
//             turn_in = 1.0f;
//         if (turn_in < -1.0f)
//             turn_in = -1.0f;
//     }
//     float vx = forward_in * g_app.user_set_speed;
//     float vy = 0.0f; // 差速底盘不支持侧向移动
//     float omega = turn_in * g_app.user_set_speed * g_app.turn_speed;

//     set_chassis_velocity(vx, vy, omega);
// }

// static void get_rc_control_inputs(uint16_t *throttle, uint16_t *steering)
// {

//     if (!CheckCommunicationStatus())
//     {
//         *throttle = 2048;
//         *steering = 2048;
//         return;
//     }
//     uint16_t raw_lx = g_app.Joystick[JOYSTICK_INDEX_LEFT_X];
//     uint16_t raw_ly = g_app.Joystick[JOYSTICK_INDEX_THROTTLE];
//     uint16_t raw_rx = g_app.Joystick[JOYSTICK_INDEX_STEERING];
//     uint16_t raw_ry = g_app.Joystick[JOYSTICK_INDEX_RIGHT_Y];

//     switch (g_app.cfg_op_mode)
//     {
//     case 0: // 双手1: 左手控前�?LY)，右手控左右(RX)
//         *throttle = raw_ly;
//         *steering = raw_rx;
//         break;
//     case 1: // 双手2: 左手控左�?LX)，右手控前后(RY)
//         *throttle = raw_ry;
//         *steering = raw_lx;
//         break;
//     case 2: // 单手1: 左手全包，控前后(LY)和左�?LX)
//         *throttle = raw_ly;
//         *steering = raw_lx;
//         break;
//     case 3: // 单手2: 右手全包，控前后(RY)和左�?RX)
//         *throttle = raw_ry;
//         *steering = raw_rx;
//         break;
//     default:
//         *throttle = raw_ly;
//         *steering = raw_rx;
//         break;
//     }
// }

// void Control_Update(void)
// {

//     // =========================================================
//     // 1. 异步轮询请求 ODrive 编码器数据
//     // ODrive 0.5.1 需要主机主动索要，数据会在接下来几毫秒内进 RX 中断
//     // =========================================================

  

//     // 1. 每一帧都必须做的：轮询 ODrive 数据（这是控制闭环的基础）
//     // chassis_control_req_odrive_data();

    

//     // 发送线速度、角速度与编码器数据

//     // =========================================================
//     //  无论什么模式，底层物理安全必须置顶！
//     // =========================================================
//     if (g_app.g_collision_latched)
//     {
//         chassis_stop();
//         return;
//     }

//     // --- 状态 0：纯遥控 ---
//     if (Ctrl_Mode == 0)
//     {
//         get_rc_control_inputs(&mapped_throttle, &mapped_steering);
//         set_chassis_rc_control(mapped_throttle, mapped_steering);
//     }
//     // --- 状态 1：API 待机 (已挂载但禁止动力) ---
//     else if (Ctrl_Mode == 1)
//     {
//         chassis_stop(); // 动力切断

//         // 安全检测：如果此时遥控器动了，虽然已经是状态1，但我们还是设为api_lockout
//         // 这样可以防止上位机在状态1时，人去动摇杆，上位机却突然发指令跳到状态2
//         uint16_t lt, ls;
//         get_rc_control_inputs(&lt, &ls);
//         if (abs((int)lt - 2048) > JOYSTICK_DEADZONE || abs((int)ls - 2048) > JOYSTICK_DEADZONE)
//         {
//             api_lockout = 1;
//         }
//     }
//     // --- 状态 2：API 运行 (动力开启) ---
//     else if (Ctrl_Mode == 2)
//     {
//         // 1. API 心跳超时检查
//         if (HAL_GetTick() - last_host_cmd_tick > API_TIMEOUT_MS)
//         {
//             chassis_stop();
//             return;
//         }

//         // 2. 遥控器最高优先级拦截
//         uint16_t lt, ls;
//         get_rc_control_inputs(&lt, &ls);
//         if (abs((int)lt - 2048) > JOYSTICK_DEADZONE || abs((int)ls - 2048) > JOYSTICK_DEADZONE)
//         {
//             Ctrl_Mode = 0;   // 瞬间踢回遥控模式
//             api_lockout = 1; // 锁死 API
//             chassis_stop();  // 刹车
//             return;
//         }

//         // 3. 正常执行 API 运动指令
//         set_chassis_velocity(host_vx, host_vy, host_omega);
//     }
// }
