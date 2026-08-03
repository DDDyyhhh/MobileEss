/**
 *******************************************************************************
 * @file           : chassis_control.c
 * @author         : [Your Name/Team]
 * @version        : V1.1
 * @date           : 2026-04-16
 * @brief          : 4轮差速底盘控制程序，支持遥控模式和API模式
 *
 * 主要功能：
 * 1. 遥控模式：处理遥控器输入，支持死区处理和转向补偿
 * 2. API模式：基于 (vx, vy, w) 输入计算四个轮子的速度
 * 3. 支持4轮差速底盘运动学（vy 默认忽略）
 * 4. 轮子参数：直径240mm，左右轴距690mm，前后轮距640mm
 *5.最大速度约 2.5 m/s（根据实际测试调整）
 * 注意事项：
 * - vx: 前进速度 (m/s)
 * - vy: 侧向速度 (m/s)
 * - w:  角速度 (rad/s)
 * - 输出单位: turn/s (转/秒)
 *******************************************************************************
 */
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "app_context.h" // 包含全局上下文，获取用户设定的速度
#include "serial_port.h"
#include "cruise_control.h"
#include "connet_check.h"
#include "BladeMotor.h"
#include "lifter.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "odrive_can.h" // 包含ODrive CAN通信函数
#include "tricolor_led.h"
uint8_t Ctrl_Mode = 0; // 0: 遥控模式, 1: API模式
float host_vx = 0.0f;
static float host_vy = 0.0f;
float host_omega = 0.0f;
static uint8_t host_blade_speed_pct = 0;
static uint8_t host_lifter_gear = 1;
static uint8_t host_bag_control = 0;
//  API 上位机心跳时间戳
uint32_t last_host_cmd_tick = 0;
uint8_t api_lockout = 0; // API 锁存标志位：一旦遥控器摇杆动了，置 1 锁死 API，直到遥控器屏幕下发解除指令
extern AppContext_t g_app;
extern volatile ODrive_Data_t g_odrive_motors[ODRIVE_MAX_MOTORS];
uint16_t mapped_throttle = 2048;
uint16_t mapped_steering = 2048;
/**
 * @brief 初始化底盘控制
 */
void chassis_control_init(void)
{
    last_host_cmd_tick = HAL_GetTick();
}

/**
 * @brief 解析上位机控制帧（HOST_CMD_FRAME_ID = 0x005）
 *
 * 控制帧格式（标准帧，DLC=8）：
 * Byte0: ctrl_mode      0=RC模式, 1=API待机, 2=API运行
 * Byte1: vx_raw         int8，线速度指令，单位 0.1 m/s
 * Byte2: vy_raw         int8，预留（差速底盘当前忽略）
 * Byte3: omega_raw      int8，角速度指令，单位 0.1 rad/s
 * Byte4: blade_speed    uint8，割刀挡位 0-3
 * Byte5: lifter_gear    uint8，升降档位 1-5（越界会钳位）
 * Byte6: bag_control    uint8，0=关，非0=开
 * Byte7: magic_key      0xAA=强制解除物理碰撞锁

 * 解析规则：
 * host_vx    = vx_raw * 0.1f;      // m/s
 * host_omega = omega_raw * 0.1f;   // rad/s
 * host_vy    = 0.0f;               // 当前不支持侧移
 *
 * 示例1：前进 0.1m/s，无旋转，API模式
 * ID=0x05, DLC=8, DATA=01 01 00 00 00 01 00 AA
 *
 * 示例2：后退 0.5m/s，右转 0.3rad/s
 * ID=0x05, DLC=8, DATA=01 FB 00 03 00 01 00 AA
 * 说明：FB(int8)=-5 => -0.5m/s，03 => +0.3rad/s
 */
void chassis_control_parse_host_frame(uint8_t frame_id, const uint8_t *payload, uint8_t len)
{
    if (frame_id != HOST_CMD_FRAME_ID || payload == NULL || len < HOST_CMD_PAYLOAD_LEN)
    {
        return;
    }

    last_host_cmd_tick = HAL_GetTick();

    if (payload[7] == 0xAA)
    {
        g_app.g_collision_latched = 0;
        g_app.lock = 0;
        RGB_LED_SetColor(LED_1, RGB_COLOR_GREEN);
        // 【新增】：强制清除弹窗状态，让遥控器屏幕恢复正常
        if (g_app.g_system_report_state == 7 || g_app.g_system_report_state == 1)
        {
            g_app.g_system_report_state = 0; // 0 就是 REPORT_NONE
        }
    }

    uint8_t host_cmd = payload[0];

    // ============================================================
    // 【三级跳状态机逻辑】
    // ============================================================
    if (host_cmd == 1)
    {
        // --- 动作：申请进入 API 待机 ---
        // 只要上位机发 1，无论当前在什么状态，都强制回到 API 待机状态
        // 并且【关键】：在这里清空遥控器的拦截锁 (api_lockout)
        Ctrl_Mode = 1;
        api_lockout = 0;
    }
    else if (host_cmd == 2)
    {
        // --- 动作：申请开启 API 动力 ---
        // 只有当前已经在状态 1 (待机)，且没有被遥控器拦截时，才允许跳到状态 2
        if (Ctrl_Mode == 1 && api_lockout == 0)
        {
            Ctrl_Mode = 2;
        }
        // 如果当前是状态 0 或有 lockout，发 2 是无效的，车子不会动
    }
    else if (host_cmd == 0)
    {
        // --- 动作：主动退回遥控 ---
        if (Ctrl_Mode == 1 || Ctrl_Mode == 2)
        {
            // 只要从 API 模式退出来，必须强制锁死底盘，让操作员双摇杆解锁！
            g_app.lock = 1;
            g_app.g_collision_latched = 1;
            g_app.g_system_report_state = 7; // 弹窗：请手动解锁
            g_app.receiver_status[STATUS_INDEX_LOCK] = 0;
            g_app.receiver_status[STATUS_INDEX_BLADE_SPEED] = 0;
        }
        // --- 动作：主动退回遥控 ---
        Ctrl_Mode = 0;
        api_lockout = 0;
    }

    int8_t vx_raw = (int8_t)payload[1];
    int8_t vy_raw = (int8_t)payload[2];
    int8_t omega_raw = (int8_t)payload[3];
    uint8_t blade_speed_pct = payload[4];
    uint8_t lifter_gear = payload[5];
    uint8_t bag_control = payload[6];

    host_blade_speed_pct = (blade_speed_pct > 3) ? 3 : blade_speed_pct;
    host_lifter_gear = (lifter_gear < 1) ? 1 : (lifter_gear > 5 ? 5 : lifter_gear);
    host_bag_control = (bag_control != 0) ? 1 : 0;

    // --- API 模式状态同步覆盖 ---
    if (Ctrl_Mode == 2)
    {
        // 强制覆盖全局应用状态
        Lifter_SetGrassBagTarget(host_bag_control);
        g_app.knob_height = host_lifter_gear;
        g_app.knob_blade = host_blade_speed_pct;

        // 【关键】同步给反馈状态缓冲区，让遥控器屏幕显示 API 设定的真实挡位
        // 注意：根据你 pairing_receiver.c 的协议，receiver_status[0] 是割刀状态
        g_app.receiver_status[STATUS_INDEX_LOCK] = host_blade_speed_pct;
        g_app.receiver_status[STATUS_INDEX_BLADE_SPEED] = host_blade_speed_pct;
    }

    host_vx = (float)vx_raw * 0.1f;       // 假设输入范围 -10 到 +10 对应 -1.0 m/s 到 +1.0 m/s
    host_vy = 0.0f;                       // 差速底盘不支持侧向移动
    host_omega = (float)omega_raw * 0.1f; // 假设输入范围 -10 到 +10 对应 -1.0 rad/s 到 +1.0 rad/s
}

/**
 * @brief 发送底盘状态反馈帧
 * */
void chassis_control_send_status_feedback(void)
{
    static uint32_t last_feedback_tick = 0;
    uint32_t now_tick = HAL_GetTick();
    if ((now_tick - last_feedback_tick) < 20U)
    {
        return;
    }
    last_feedback_tick = now_tick;

    uint8_t tx_data[6];
    tx_data[0] = Ctrl_Mode;
    tx_data[1] = g_app.bms_temp;
    tx_data[2] = (g_app.voltage_int > 100) ? 100 : g_app.voltage_int;

    uint8_t bag_status = 0;
    if (g_app.grass_bag_status)
        bag_status |= 0x01;
    if (g_app.is_bag_connected)
        bag_status |= 0x02;
    if (g_app.is_bag_full)
        bag_status |= 0x04;
    tx_data[3] = bag_status;
    tx_data[4] = (g_app.key[0] & 0x03) | ((g_app.key[1] & 0x03) << 2) | ((g_app.key[2] & 0x03) << 4) | ((g_app.key[3] & 0x03) << 6);
    tx_data[5] = (g_app.key[4] & 0x03) | ((g_app.key[5] & 0x03) << 2) | ((g_app.key[6] & 0x03) << 4) | ((g_app.key[7] & 0x03) << 6);

    odrive_can_send_message(&hcan1, CHASSIS_STATUS_CAN_ID, tx_data, sizeof(tx_data), 0);
}

/**
 * @brief 向 4 个 ODrive 节点手动请求编码器数据 (兼容 0.5.1 固件)
 * @note  发送的是 RTR 远程帧，数据会在随后的 CAN RX 中断中自动更新到 g_odrive_motors 数组
 */
/**
 * @brief 向底层驱动器手动请求数据 (例如编码器速度)
 */
void chassis_control_req_motor_data(void)
{
    static uint32_t last_req_tick = 0;
    static uint8_t req_step = 0;

    uint32_t now_tick = HAL_GetTick();

    // 反馈降频：每 30ms 只请求一个通道
    if ((now_tick - last_req_tick) < 30U)
        return;
    last_req_tick = now_tick;

#if MOTOR_DRIVER_USED == DRIVER_TYPE_ODRIVE

    odrive_req_encoder_estimates(&hcan1, 1);
    odrive_req_encoder_estimates(&hcan1, 2);
    odrive_req_encoder_estimates(&hcan1, 3);
    odrive_req_encoder_estimates(&hcan1, 4);

#elif MOTOR_DRIVER_USED == DRIVER_TYPE_MSSD

    // MSSD 错峰查询状态机 (总周期 20ms * 8 = 160ms)
    // - 速度查询会被执行 4 次 (平均 40ms 一次，保证控制平滑)
    // - 功率查询执行 2 次 (160ms 更新一轮)
    // - 温度查询执行 2 次 (160ms 更新一轮)
    switch (req_step)
    {
    case 0:
        MSSD_Req_Velocity(MSSD_NODE_RIGHT, MSSD_CH_RIGHT);
        MSSD_Req_Velocity(MSSD_NODE_RIGHT, MSSD_CH_LEFT);
        break;
    case 1:
        MSSD_Req_Velocity(MSSD_NODE_LEFT, MSSD_CH_RIGHT);
        MSSD_Req_Velocity(MSSD_NODE_LEFT, MSSD_CH_LEFT);
        break;
    case 2:
        MSSD_Req_Power(MSSD_NODE_RIGHT); // 查右侧电压电流
        break;
    case 3:
        MSSD_Req_Power(MSSD_NODE_LEFT); // 查左侧电压电流
        break;
    case 4:
        MSSD_Req_Velocity(MSSD_NODE_RIGHT, MSSD_CH_RIGHT); // 再次穿插速度请求
        MSSD_Req_Velocity(MSSD_NODE_RIGHT, MSSD_CH_LEFT);
        break;
    case 5:
        MSSD_Req_Velocity(MSSD_NODE_LEFT, MSSD_CH_RIGHT); // 再次穿插速度请求
        MSSD_Req_Velocity(MSSD_NODE_LEFT, MSSD_CH_LEFT);
        break;
    case 6:
        MSSD_Req_Temperature(MSSD_NODE_RIGHT); // 查右侧温度
        break;
    case 7:
        MSSD_Req_Temperature(MSSD_NODE_LEFT); // 查左侧温度
        break;
    }

    req_step++;
    if (req_step > 7)
    {
        req_step = 0;
    }

#elif MOTOR_DRIVER_USED == DRIVER_TYPE_OID
    // OID 错峰轮询状态机 (总周期 30ms * 10 = 300ms)
    // - 错误码 4 次 → 相电流 2 次 → 电压 2 次 → 温度 2 次
    switch (req_step)
    {
    case 0:
        OID_Req_Parameter(OID_NODE_FL, OID_PARAM_ERROR);
        break;
    case 1:
        OID_Req_Parameter(OID_NODE_FR, OID_PARAM_ERROR);
        break;
    case 2:
        OID_Req_Parameter(OID_NODE_RL, OID_PARAM_ERROR);
        break;
    case 3:
        OID_Req_Parameter(OID_NODE_RR, OID_PARAM_ERROR);
        break;
    case 4:
        OID_Req_Parameter(OID_NODE_FL, OID_PARAM_PHASE_CURRENT);
        OID_Req_Parameter(OID_NODE_FR, OID_PARAM_PHASE_CURRENT);
        break;
    case 5:
        OID_Req_Parameter(OID_NODE_RL, OID_PARAM_PHASE_CURRENT);
        OID_Req_Parameter(OID_NODE_RR, OID_PARAM_PHASE_CURRENT);
        break;
    case 6:
        OID_Req_Parameter(OID_NODE_FL, OID_PARAM_BUS_VOLTAGE);
        OID_Req_Parameter(OID_NODE_FR, OID_PARAM_BUS_VOLTAGE);
        break;
    case 7:
        OID_Req_Parameter(OID_NODE_RL, OID_PARAM_BUS_VOLTAGE);
        OID_Req_Parameter(OID_NODE_RR, OID_PARAM_BUS_VOLTAGE);
        break;
    case 8:
        OID_Req_Parameter(OID_NODE_FL, OID_PARAM_TEMPERATURE);
        OID_Req_Parameter(OID_NODE_RL, OID_PARAM_TEMPERATURE);
        break;
    case 9:
        OID_Req_Parameter(OID_NODE_FR, OID_PARAM_TEMPERATURE);
        OID_Req_Parameter(OID_NODE_RR, OID_PARAM_TEMPERATURE);
        break;
    }

    req_step++;
    if (req_step > 9)
    {
        req_step = 0;
    }

#endif
}

#if MOTOR_DRIVER_USED == DRIVER_TYPE_OID
static void chassis_control_req_oid_actual_speed(void)
{
    static uint32_t last_req_tick = 0;
    uint32_t now_tick = HAL_GetTick();

    if ((now_tick - last_req_tick) < 50U)
        return;
    last_req_tick = now_tick;

    OID_Req_Actual_Speed(OID_NODE_FL);
    OID_Req_Actual_Speed(OID_NODE_FR);
    OID_Req_Actual_Speed(OID_NODE_RL);
    OID_Req_Actual_Speed(OID_NODE_RR);
}

static int16_t chassis_erpm_to_speed_mmps(int32_t erpm, int8_t sign)
{
    float speed_mps = ((float)erpm * (float)sign) * PI * WHEEL_DIAMETER /
                      ((float)OID_POLE_PAIRS * GEAR_RATIO * 60.0f);
    float speed_mmps = speed_mps * 1000.0f;
    int32_t value = (speed_mmps >= 0.0f) ? (int32_t)(speed_mmps + 0.5f) : (int32_t)(speed_mmps - 0.5f);

    if (value > INT16_MAX)
        return INT16_MAX;
    if (value < INT16_MIN)
        return INT16_MIN;
    return (int16_t)value;
}

static void chassis_pack_i16_be(uint8_t *buf, uint8_t offset, int16_t value)
{
    buf[offset] = (uint8_t)((uint16_t)value >> 8);
    buf[offset + 1U] = (uint8_t)((uint16_t)value & 0xFFU);
}

static void chassis_control_send_oid_wheel_speed_feedback(void)
{
    static uint32_t last_speed_tick = 0;
    uint32_t now_tick = HAL_GetTick();

    if ((now_tick - last_speed_tick) < 50U)
        return;
    last_speed_tick = now_tick;

    uint8_t tx_buf[8];
    int16_t speed_fl = chassis_erpm_to_speed_mmps(g_oid_drives[OID_NODE_FL].actual_speed_erpm, 1);
    int16_t speed_fr = chassis_erpm_to_speed_mmps(g_oid_drives[OID_NODE_FR].actual_speed_erpm, -1);
    int16_t speed_rl = chassis_erpm_to_speed_mmps(g_oid_drives[OID_NODE_RL].actual_speed_erpm, 1);
    int16_t speed_rr = chassis_erpm_to_speed_mmps(g_oid_drives[OID_NODE_RR].actual_speed_erpm, -1);

    chassis_pack_i16_be(tx_buf, 0, speed_fl);
    chassis_pack_i16_be(tx_buf, 2, speed_fr);
    chassis_pack_i16_be(tx_buf, 4, speed_rl);
    chassis_pack_i16_be(tx_buf, 6, speed_rr);

    odrive_can_send_message(&hcan1, CHASSIS_WHEEL_SPEED_CAN_ID, tx_buf, 8, 0);
}
#endif

/**
 * @brief 紧急切断底盘动力
 */
void chassis_stop(void)
{
#if MOTOR_DRIVER_USED == DRIVER_TYPE_ODRIVE
    Send_Motor_speed(1, 0.0f, 0.0f, 0);
    Send_Motor_speed(2, 0.0f, 0.0f, 0);
    Send_Motor_speed(3, 0.0f, 0.0f, 0);
    Send_Motor_speed(4, 0.0f, 0.0f, 0);
#elif MOTOR_DRIVER_USED == DRIVER_TYPE_MSSD
    MSSD_Stop_Motor(MSSD_NODE_RIGHT, MSSD_CH_RIGHT);
    MSSD_Stop_Motor(MSSD_NODE_RIGHT, MSSD_CH_LEFT);
    MSSD_Stop_Motor(MSSD_NODE_LEFT, MSSD_CH_RIGHT);
    MSSD_Stop_Motor(MSSD_NODE_LEFT, MSSD_CH_LEFT);
#elif MOTOR_DRIVER_USED == DRIVER_TYPE_OID
    set_chassis_velocity(0, 0, 0);
#endif
}

/**
 * @brief 计算真实里程计并发送 (线速度、角速度、编码器累计位置)
 * @note 频率限制为 20ms (50Hz) 发送一次，防止挤占 CAN 总线带宽
 */
void chassis_control_send_odom_feedback(void)
{
    static uint32_t last_odom_tick = 0;
    uint32_t now_tick = HAL_GetTick();

    if ((now_tick - last_odom_tick) < 50U)
        return;
    last_odom_tick = now_tick;

    float tx_vx = 0.0f;
    float tx_omega = 0.0f;

    // ============================================================
    // 【核心修改】：根据当前模式选择反馈的数据源
    // ============================================================
    if (Ctrl_Mode == 1 || Ctrl_Mode == 2)
    {
        // API 模式下，直接反馈上位机发来的设定值 (Echo)
        tx_vx = host_vx;
        tx_omega = host_omega;
    }

    // -------------------------------------------------------------
    // 3. 打包发送 CAN 数据帧 (ID: 0x507)
    // -------------------------------------------------------------
    uint8_t tx_buf[8];

    // Byte 0-3: tx_vx (float)
    // Byte 4-7: tx_omega (float)
    memcpy(&tx_buf[0], &tx_vx, 4);
    memcpy(&tx_buf[4], &tx_omega, 4);

    odrive_can_send_message(&hcan1, CHASSIS_ODOM_CAN_ID, tx_buf, 8, 0);
    // --- 帧 0x08: 右侧编码器绝对位置 (Turns) ---
    // float pos_fr = g_odrive_motors[1].encoder_pos;
    // float pos_rr = g_odrive_motors[2].encoder_pos;
    // memcpy(&tx_buf[0], &pos_fr, 4);
    // memcpy(&tx_buf[4], &pos_rr, 4);
    // odrive_can_send_message(&hcan1, CHASSIS_ENC_RIGHT_CAN_ID, tx_buf, 8, 0);

    // // --- 帧 0x09: 左侧编码器绝对位置 (Turns) ---
    // // 左侧取反，保证前进时所有编码器位置都是单调递增的，方便上位机做里程计
    // float pos_fl = -g_odrive_motors[3].encoder_pos;
    // float pos_rl = -g_odrive_motors[4].encoder_pos;
    // memcpy(&tx_buf[0], &pos_fl, 4);
    // memcpy(&tx_buf[4], &pos_rl, 4);
    // odrive_can_send_message(&hcan1, CHASSIS_ENC_LEFT_CAN_ID, tx_buf, 8, 0);
}

void Send_Motor_speed(uint8_t axis_id, float speed_rpm, float torque_ff, uint8_t reset_integrator)
{
    // 1. 单位转换: Turns/s
    float vel_rps = speed_rpm;
    odrive_set_input_vel(&hcan1, axis_id, vel_rps, torque_ff);
}

void set_chassis_velocity(float vx, float vy, float omega)
{
    vy = 0.0f; // 强制不支持侧向移动

    float R = TRACK_WIDTH / 2.0f;

    // 计算各轮子线速度 (m/s)
    float v_fr = vx - vy - omega * R;
    float v_fl = vx + vy + omega * R;
    float v_rr = vx + vy - omega * R;
    float v_rl = vx - vy + omega * R;

    // 转换为RPM (电机输出轴 RPM = 线速度 * 60 / 轮周长)
    float rpm_fl = (v_fl * 60.0f) / (PI * WHEEL_DIAMETER);
    float rpm_fr = (v_fr * 60.0f) / (PI * WHEEL_DIAMETER);
    float rpm_rl = (v_rl * 60.0f) / (PI * WHEEL_DIAMETER);
    float rpm_rr = (v_rr * 60.0f) / (PI * WHEEL_DIAMETER);

    // 乘以减速比，得到电机本体的真实 RPM
    float motor_rpm_fl = rpm_fl * GEAR_RATIO;
    float motor_rpm_fr = rpm_fr * GEAR_RATIO;
    float motor_rpm_rl = rpm_rl * GEAR_RATIO;
    float motor_rpm_rr = rpm_rr * GEAR_RATIO;

#if MOTOR_DRIVER_USED == DRIVER_TYPE_ODRIVE
    // ODrive 接收的是 Turns/s (转/秒)，需要除以 60
    Send_Motor_speed(1, motor_rpm_fr / 60.0f, 0.0f, 0);  // 前右
    Send_Motor_speed(2, motor_rpm_rr / 60.0f, 0.0f, 0);  // 后右
    Send_Motor_speed(3, -motor_rpm_fl / 60.0f, 0.0f, 0); // 前左，物理取反
    Send_Motor_speed(4, -motor_rpm_rl / 60.0f, 0.0f, 0); // 后左，物理取反

#elif MOTOR_DRIVER_USED == DRIVER_TYPE_MSSD
    // MSSD 直接接收 RPM (整型)
    int32_t send_rpm_fr = (int32_t)(-motor_rpm_fr);
    int32_t send_rpm_rr = (int32_t)(-motor_rpm_rr);
    int32_t send_rpm_fl = (int32_t)(motor_rpm_fl);
    int32_t send_rpm_rl = (int32_t)(motor_rpm_rl);
    // Serial_Printf(SERIAL_PORT_2, "Set Vel FR: %ld RPM, RR: %ld RPM, FL: %ld RPM, RL: %ld RPM\r\n", send_rpm_fr, send_rpm_rr, send_rpm_fl, send_rpm_rl);
    //  驱动器 1 (Node 1) 控制左侧前后电机
    MSSD_Set_Velocity(MSSD_NODE_LEFT, MSSD_CH_RIGHT, send_rpm_fl);
    osDelay(1);
    MSSD_Set_Velocity(MSSD_NODE_LEFT, MSSD_CH_LEFT, send_rpm_rl);
    osDelay(1);
    MSSD_Set_Velocity(MSSD_NODE_RIGHT, MSSD_CH_RIGHT, send_rpm_fr);
    osDelay(1); // 适当延时，避免 CAN 总线过载
    MSSD_Set_Velocity(MSSD_NODE_RIGHT, MSSD_CH_LEFT, send_rpm_rr);
    osDelay(1);

#elif MOTOR_DRIVER_USED == DRIVER_TYPE_OID
    // OID 使用 erpm，公式: erpm = rpm * 极对数
    // 全部统一使正 erpm = 前进方向
    int32_t oid_erpm_fr = (int32_t)(-motor_rpm_fr * OID_POLE_PAIRS);
    int32_t oid_erpm_rr = (int32_t)(-motor_rpm_rr * OID_POLE_PAIRS);
    int32_t oid_erpm_fl = (int32_t)(motor_rpm_fl * OID_POLE_PAIRS);
    int32_t oid_erpm_rl = (int32_t)(motor_rpm_rl * OID_POLE_PAIRS);

    OID_Set_Speed(OID_NODE_FR, oid_erpm_fr);
    OID_Set_Speed(OID_NODE_RR, oid_erpm_rr);
    OID_Set_Speed(OID_NODE_FL, oid_erpm_fl);
    OID_Set_Speed(OID_NODE_RL, oid_erpm_rl);

#endif
}

/**
 * @brief 使用遥控器输入控制底盘运动（遥控模式）
 * @param ly 左摇杆Y轴值(0-4095)
 * @param rx 右摇杆X轴值(0-4095)
 */
void set_chassis_rc_control(uint16_t ly, uint16_t rx)
{
    // 1. 归一化输入(-1.0 ~ 1.0)
    float forward_in = ((float)ly - 2048.0f) / 2048.0f;  // 霍尔摇杆
    float turn_in = ((float)2048 - (float)rx) / 2048.0f; // 减速器轮子
    // 2. 死区处理
    if (fabsf(forward_in) < RC_DEADZONE)
    {
        forward_in = 0.0f;
    }
    // ============================================================
    // <<<---  转向起步跃变补偿 (克服草地静摩擦) --->>>
    // ============================================================
    // 设定最小转向响应力度 (0.35 表示只要轻推，瞬间输出 35% 的转向力度)

    if (fabsf(turn_in) <= RC_DEADZONE)
    {
        turn_in = 0.0f; // 在死区内，绝对静止
    }
    else
    {
        // 提取方向符号 (1.0 为右，-1.0 为左)
        float sign = (turn_in > 0.0f) ? 1.0f : -1.0f;

        // 计算超出死区后的有效行程比例 (0.0 ~ 1.0)
        float active_ratio = (fabsf(turn_in) - RC_DEADZONE) / (1.0f - RC_DEADZONE);

        // 重新映射：将 0.0~1.0 的行程，映射到响应区间
        turn_in = sign * (RC_MIN_TURN_RESPONSE + active_ratio * (1.0f - RC_MIN_TURN_RESPONSE));
        if (turn_in > 1.0f)
            turn_in = 1.0f;
        if (turn_in < -1.0f)
            turn_in = -1.0f;
    }

    // ============================================================
    // <<<--- 护草模式运动学限幅 --->>>
    // 原理：禁止“无油门打方向”(原地转圈)，并限制最大转弯比例
    // ============================================================
    if (g_app.cfg_lawn_protect_en == 1)
    {
        // 1. 如果完全没有前进/后退的油门输入，严禁任何转向操作！
        if (forward_in == 0.0f)
        {
            turn_in = 0.0f;
        }
        else
        {
            // 2. 比例限幅保护 (防止大油门极限死打方向，导致内侧轮反转刨坑)
            // 逻辑：允许的最大转向输入，绝不能超过当前油门输入的绝对值
            // 比如：你只推了 30% 的油门，那就算你方向盘打满，系统也只给你 30% 的转向度
            float abs_forward = fabsf(forward_in);
            float abs_turn = fabsf(turn_in);

            if (abs_turn > abs_forward)
            {
                // 裁切 turn_in，但保留原有的正负号方向
                turn_in = (turn_in > 0.0f) ? abs_forward : -abs_forward;
            }
        }
    }

    // ============================================================
    // <<<---  后退限速动态补偿系数 --->>>
    // ============================================================
    float reverse_factor = 1.0f; // 默认不减速
    float turn_in_dir; // 后退方向
    // forward_in < 0.0f 说明操作员正在将摇杆往后拉（向后倒车）
    if (forward_in < 0.0f && g_app.cfg_reverse_limit_en == 1)
    {
        reverse_factor = 0.8f; // 如果遥控器设置中的“后退限速”开启，速度裁切为 80%
        
    }

    if (forward_in < 0.0f)
    {
        turn_in_dir = -turn_in;
    }
    else if (forward_in > 0.0f)
    {
        turn_in_dir = turn_in;
    }

    float vx = forward_in * g_app.speed * reverse_factor;
    float vy = 0.0f; // 差速底盘不支持侧向移动
    float omega = turn_in_dir * g_app.speed * g_app.turn_speed;

    set_chassis_velocity(vx, vy, omega);
}

static void get_rc_control_inputs(uint16_t *throttle, uint16_t *steering)
{

    if (!CheckCommunicationStatus())
    {
        *throttle = 2048;
        *steering = 2048;
        return;
    }
    uint16_t raw_lx = g_app.Joystick[JOYSTICK_INDEX_LEFT_X];
    uint16_t raw_ly = g_app.Joystick[JOYSTICK_INDEX_THROTTLE];
    uint16_t raw_rx = g_app.Joystick[JOYSTICK_INDEX_STEERING];
    uint16_t raw_ry = g_app.Joystick[JOYSTICK_INDEX_RIGHT_Y];

    switch (g_app.cfg_op_mode)
    {
    case 0: // 双手1: 左手控前后，右手控左右
        *throttle = raw_ly;
        *steering = raw_rx;
        break;
    case 1: // 双手2: 左手控左右(LX)，右手控前后(RY)
        *throttle = raw_ry;
        *steering = raw_lx;
        break;
    case 2: // 单手1: 左手全包，控前后(LY)和左右(LX)
        *throttle = raw_ly;
        *steering = raw_lx;
        break;
    case 3: // 单手2: 右手全包，控前后(RY)和左右(RX)
        *throttle = raw_ry;
        *steering = raw_rx;
        break;
    default:
        *throttle = raw_ly;
        *steering = raw_rx;
        break;
    }
}

void Control_Update(void)
{

#if MOTOR_DRIVER_USED == DRIVER_TYPE_OID
    chassis_control_req_oid_actual_speed();
    chassis_control_send_oid_wheel_speed_feedback();
#endif

    // =========================================================
    //  无论什么模式，底层物理安全必须置顶！
    // =========================================================
    if (g_app.g_collision_latched)
    {
        chassis_stop();
        return;
    }

    // --- 状态 0：纯遥控 ---
    if (Ctrl_Mode == 0)
    {

        get_rc_control_inputs(&mapped_throttle, &mapped_steering);
        set_chassis_rc_control(mapped_throttle, mapped_steering);
    }
    // --- 状态 1：API 待机 (已挂载但禁止动力) ---
    else if (Ctrl_Mode == 1)
    {
        chassis_stop(); // 动力切断

        // 安全检测：如果此时遥控器动了，虽然已经是状态1，但我们还是设为api_lockout
        // 这样可以防止上位机在状态1时，人去动摇杆，上位机却突然发指令跳到状态2
        uint16_t lt, ls;
        get_rc_control_inputs(&lt, &ls);
        if (abs((int)lt - 2048) > JOYSTICK_DEADZONE || abs((int)ls - 2048) > JOYSTICK_DEADZONE)
        {
            Ctrl_Mode = 0;
            api_lockout = 1;
            // 【核心新增】：退回遥控后，立刻锁死底盘，必须双摇杆解锁
            g_app.lock = 1;
            g_app.g_collision_latched = 1;
            g_app.g_system_report_state = 7; // 弹窗提示：请手动解锁
            g_app.receiver_status[STATUS_INDEX_LOCK] = 0;
            g_app.receiver_status[STATUS_INDEX_BLADE_SPEED] = 0;
        }
    }
    // --- 状态 2：API 运行 (动力开启) ---
    else if (Ctrl_Mode == 2)
    {
        // 1. API 心跳超时检查
        if (HAL_GetTick() - last_host_cmd_tick > API_TIMEOUT_MS)
        {
            Ctrl_Mode = 0;
            g_app.lock = 1;
            g_app.g_collision_latched = 1;
            g_app.g_system_report_state = 7; // 弹窗提示：请手动解锁
            chassis_stop();
            g_app.receiver_status[STATUS_INDEX_LOCK] = 0;
            g_app.receiver_status[STATUS_INDEX_BLADE_SPEED] = 0;
            return;
        }

        // 2. 遥控器最高优先级拦截
        uint16_t lt, ls;
        get_rc_control_inputs(&lt, &ls);
        if (abs((int)lt - 2048) > JOYSTICK_DEADZONE || abs((int)ls - 2048) > JOYSTICK_DEADZONE)
        {
            Ctrl_Mode = 0;
            api_lockout = 1;
            chassis_stop();

            // ：遥控器接管后，必须先双摇杆解锁才能开

            g_app.is_on_slope = 1;
            BladeMotor_SetSpeed(BLADE_SPEED_OFF);
            Cruise_Stop();

            g_app.lock = 1;                  // 顺便把割刀也锁上，保证绝对安全
            g_app.g_collision_latched = 1;   // 锁底盘
            g_app.g_system_report_state = 7; // 弹窗提示解锁
            g_app.receiver_status[STATUS_INDEX_LOCK] = 0;
            g_app.receiver_status[STATUS_INDEX_BLADE_SPEED] = 0;
            return;
        }

        // 3. 正常执行 API 运动指令
        set_chassis_velocity(host_vx, host_vy, host_omega);
    }

    chassis_control_req_motor_data();
    osDelay(1);
    // 发送线速度、角速度与编码器数据
}
