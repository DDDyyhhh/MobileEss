/**
 ******************************************************************************
 * @file           : cruise_control.c
 * @author         : [Your Name/Team]
 * @version        : V1.1
 * @date           : 2026-03-11
 * @brief          : 定向巡航控制逻辑，负责开环直线保持与智能避障降速
 *
 * 主要功能：
 * 1. 摇杆无干预时自动进入巡航模式，保持当前航向(开环直行)；一旦检测到油门/平移摇杆动了，立即退出。
 * 2. 手动介入无缝切换：在巡航期间依然允许通过右摇杆原地修正航向。
 * 3. 融合超声波避障：在直线巡航中，当前方障碍物进入阈值时自动降速。
 *
 * 注意事项：
 * - 一旦触碰油门/平移摇杆 (左摇杆)，将无条件退出巡航模式返回纯手动控制。
 ******************************************************************************
 */

#include "cruise_control.h"
#include "car_Control.h" // 需要控制电机
#include "buzzer.h"      // 需要蜂鸣器
#include "app_context.h"
#include <math.h>
#include <stdlib.h>
#include "tricolor_led.h"
#include "chassis_control.h"
// --- 内部变量 ---
static CruiseState_t g_cruise_state = CRUISE_OFF;
static float g_cruise_target_yaw = 0.0f; // 锁定航向
static float g_u_turn_start_yaw = 0.0f;  // 掉头起始航向

// 定义滤波计数器 (静态变量，保持状态)
static uint8_t obstacle_filter_count = 0;

// 外部变量引用 (根据你的项目实际情况调整)
extern IMU_Data_t imu_data;

// --- 内部辅助函数 ---
/**
 * @brief 计算归一化的目标角度 (-180~180 的跳变)
 */
static float Normalize_Angle(float angle)
{
    // 如果你的 IMU 输出是 0~360
    /*
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f)    angle += 360.0f;
    */

    // 如果你的 IMU 输出是 -180~180 (通常是这个)
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle <= -180.0f)
        angle += 360.0f;

    return angle;
}

/**
 * @brief 计算角度差 (绝对值)
 */
static float Get_Angle_Diff(float start_angle, float current_angle)
{
    float diff = fabsf(current_angle - start_angle);
    if (diff > 180.0f)
        diff = 360.0f - diff;
    return diff;
}

/**
 * @brief 航向角 PID 计算
 */
static float Yaw_PID_Compute(float target, float current)
{
    float error = target - current;
    if (error > 180.0f)
        error -= 360.0f;
    if (error < -180.0f)
        error += 360.0f;

    float output = error * YAW_PID_KP;

    // 限幅
    if (output > 0.3f)
        output = 0.3f;
    if (output < -0.3f)
        output = -0.3f;

    return output;
}

// --- 接口函数实现 ---

void Cruise_Init(void)
{
    g_cruise_state = CRUISE_OFF;
}

void Cruise_Start(void)
{
    if (g_cruise_state == CRUISE_OFF)
    {
        g_cruise_state = CRUISE_STRAIGHT;
        g_app.g_system_report_state = 10;   // 直线巡航 = 10
        g_cruise_target_yaw = imu_data.yaw; // 锁定当前朝向
    }
}

void Cruise_Stop(void)
{
    if (g_cruise_state != CRUISE_OFF)
    {
        g_app.g_system_report_state = 0; // 恢复正常手动模式
        g_cruise_state = CRUISE_OFF;
    }
}

CruiseState_t Cruise_GetState(void)
{
    return g_cruise_state;
}

/**
 * @brief 巡航核心逻辑 (需在 50ms 任务中调用，直行时不再使用 PID，改为纯开环控制)
 */
void Cruise_Process(uint16_t mapped_throttle, uint16_t mapped_steering)
{
    // 1. 摇杆介入检查 (退出巡航逻辑)

    // 【核心优化】：无论你是单手还是双手模式，只要你拨动了“前进/后退”通道，
    // 就意味着你要人工接管，立刻退出巡航！
    if (abs((int)mapped_throttle - 2048) > JOYSTICK_DEADZONE)
    {
        Cruise_Stop();
        return;
    }

    // 如果未开启，直接返回
    if (g_cruise_state == CRUISE_OFF)
        return;

    // =================================================================
    //            <<<--- 智能避障与速度决策 ---<<<
    // =================================================================

    float current_cruise_speed = 0.0f;

    // 根据全局旋钮变量，选择对应的巡航基础速度
    switch (g_app.knob_gear)
    {
    case 1:
        current_cruise_speed = CRUISE_GEAR_1_SPEED;
        break;
    case 2:
        current_cruise_speed = CRUISE_GEAR_2_SPEED;
        break;
    case 3:
        current_cruise_speed = CRUISE_GEAR_3_SPEED;
        break;
    default:
        current_cruise_speed = CRUISE_GEAR_1_SPEED; // 默认或异常时用1档最安全
        break;
    }
    // 找出 4 个探头中的最小距离
    float min_dist = g_app.d1;
    if (g_app.d2 < min_dist)
        min_dist = g_app.d2;
    if (g_app.d3 < min_dist)
        min_dist = g_app.d3;
    if (g_app.d4 < min_dist)
        min_dist = g_app.d4;

    // 只有在超声波使能开启时，才执行避障干预
    if (g_app.cfg_ultrasonic_en == 1)
    {
        // 1. 【极度危险区】 <= 50cm
        if (min_dist <= 50.0f)
        {
            // 立即退出巡航模式，将控制权交还给人工！
            Cruise_Stop();

            // 打印或发出蜂鸣器提示 (可选)
            // Serial_Printf(SERIAL_PORT_2, "[Cruise] Auto-Exit: Obstacle too close (%.1f cm)!\r\n", min_dist);
            return; // 必须 return，放弃本周期的后续巡航接管
        }

        // 2. 【逼近危险区】 50 ~ 100cm
        else if (min_dist <= 100.0f)
        {
            if (current_cruise_speed > MOWER_UC_2_MAX_SPEED)
            {
                current_cruise_speed = MOWER_UC_2_MAX_SPEED; // 压低巡航速度
            }
        }

        // 3. 【预警区】 100 ~ 150cm
        else if (min_dist <= 150.0f)
        {
            if (current_cruise_speed > MOWER_UC_3_MAX_SPEED)
            {
                current_cruise_speed = MOWER_UC_3_MAX_SPEED; // 压低巡航速度
            }
        }
    }

    // =================================================================
    //            <<<--- 巡航控制逻辑 ---<<<
    // =================================================================

    // 计算转向输入 (-1.0 ~ 1.0)
    // float turn_in = ((float)rx - 2048) / 2048.0f;//山地轮子和充气轮子
    // float turn_in = (2048.0f - (float)rx) / 2048.0f;//减速器轮子
    // float turn_in = ((float)2048 - (float)mapped_steering) / 2048.0f;

    // 计算原始转向输入 (-1.0 ~ 1.0)
    float raw_turn_in = ((float)2048 - (float)mapped_steering) / 2048.0f;
    float turn_in = 0.0f;

    // ============================================================
    // <<<--- 【终极优化】 转向起步跃变补偿 (克服草地静摩擦) --->>>
    // ============================================================
    const float DEADZONE = 0.1f;
    const float MIN_TURN_RESPONSE = 0.45f; // 与手动模式保持一致的起步爆发力

    if (fabsf(raw_turn_in) <= DEADZONE)
    {
        turn_in = 0.0f;
    }
    else
    {
        float sign = (raw_turn_in > 0.0f) ? 1.0f : -1.0f;
        float active_ratio = (fabsf(raw_turn_in) - DEADZONE) / (1.0f - DEADZONE);
        turn_in = sign * (MIN_TURN_RESPONSE + active_ratio * (1.0f - MIN_TURN_RESPONSE));
    }

    // --- 分支 A: 用户正在手动转向 (右摇杆拨动) ---
    // =================================================================
    //            <<<--- 巡航运动学输出解算 --->>>
    // =================================================================

    float target_vx = 0.0f;
    float target_omega = 0.0f;

    // --- 分支 A: 用户正在手动转向 (右摇杆拨动) ---
    if (turn_in != 0.0f)
    {
        // 1. 实时更新锁定航向，为松手后继续直行做准备
        g_cruise_target_yaw = imu_data.yaw;

        float abs_turn = fabsf(turn_in);

        // 设定触发原地自转的摇杆阈值 (0.75 表示推到 75% 行程以外)
        const float PIVOT_TURN_THRESHOLD = 0.65f;

        if (abs_turn >= PIVOT_TURN_THRESHOLD)
        {
            // -----------------------------------------------------
            // 模式 1: 极度打杆 -> 原地自转 (Tank Turn)
            // -----------------------------------------------------
            // 取消直行速度，完全转化为旋转角速度
            target_vx = 0.0f;
            // 乘以 1.2 让掉头更干脆 (此处映射为角速度 omega)
            target_omega = turn_in * current_cruise_speed * 1.2f;
        }
        else
        {
            // -----------------------------------------------------
            // 模式 2: 轻中度打杆 -> 差速过弯 (Differential Turn)
            // -----------------------------------------------------
            // 车辆保持前进基础速度，同时赋予一定的角速度画弧线
            float diff_sensitivity = 0.8f; // 差速敏感度系数
            target_vx = current_cruise_speed;
            target_omega = turn_in * current_cruise_speed * diff_sensitivity;
        }
    }
    // --- 分支 B: 用户未操作，保持直线巡航 (纯开环直行) ---
    else
    {
        // -----------------------------------------------------
        // 模式 3: 纯开环直行
        // -----------------------------------------------------
        target_vx = current_cruise_speed;
        target_omega = 0.0f;
    }

    // ============================================================
    // 发送统一运动学指令 (底座逆解由该函数在内部自动完成)
    // ============================================================
    // 注：四轮差速底盘不支持 vy (侧向平移)，故强制填 0.0f
    set_chassis_velocity(target_vx, 0.0f, target_omega);
}
