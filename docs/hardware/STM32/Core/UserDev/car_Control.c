/**
  ******************************************************************************
  * @file           : car_Control.c
  * @author         : [Your Name/Team]
  * @version        : V1.3
  * @date           : 2026-03-11
  * @brief          : 核心底盘运动学、动力学前馈补偿及 ODrive 串口通信控制
  * 
  * 主要功能：
  * 1. 差速混合算法：将遥控器的油门和转向摇杆映射为左右轮速度
  * 2. 坡道安全限速：通过 IMU 俯仰角与侧倾角，在坡度 >15° 时强制限制最高车速
  * 3. 动力学前馈补偿 (Feedforward)：重力补偿 (Gravity FF) + 转向补偿 (Turn FF)
  * 4. ODrive 积分防饱和 (Windup) 智能清零：在速度归零、转向剧烈反转时，平滑重置 PID 的 I 项
  *
  * 注意事项：
  * - 发送给 ODrive 的 UART 指令极度依赖 ASCII 格式解析，指令字符串拼接需严谨。
  * - 左侧电机因为机械安装相反，代码中对速度和前馈均进行了显式取反处理。
  ******************************************************************************
  */

#include "car_Control.h"
#include <tim.h>
#include <math.h>        // 需要包含math.h以使用fabsf函数
#include "serial_port.h" // <<< 1. 包含新的串口模块头文件
#include <stdbool.h>
#include "bsp_imu_solver.h"
#include "app_context.h"
#include "odrive_can.h"
// --- 全局变量定义 ---
float g_current_target_speed_rpm = 0.0f;
#define TURN_FF_GAIN 30.0f
extern IMU_Data_t imu_data;
#define GRAVITY_GAIN 0.6f
#define MOTOR_STICTION_CURRENT 4.0f
#define LINEAR_FF_GAIN 8.0f // 直行前馈增益 (满油门时额外给 6A)
// /**
//  * @brief 死区处理函数 - 消除微小抖动
//  * @param value: 输入值指针（会被修改）
//  * @param deadzone: 死区范围，例如0.05表示±5%以内的值会被置零
//  */
void ApplyDeadzone(float *value, float deadzone)
{
    if (value == NULL)
    {
        return;
    }

    // 如果绝对值小于死区范围，则置零
    if (fabsf(*value) < deadzone)
    {
        *value = 0.0f;
    }
}

// 低通滤波器结构体
typedef struct
{
    float output;
    float alpha; // 滤波系数 (0-1), 越小越平滑
} LowPassFilter;

// 初始化滤波器
void LowPassFilter_Init(LowPassFilter *filter, float alpha, float initial_value)
{
    filter->output = initial_value;
    filter->alpha = alpha;
}

// 应用低通滤波器
float LowPassFilter_Update(LowPassFilter *filter, float input)
{
    filter->output = filter->alpha * input + (1.0f - filter->alpha) * filter->output;
    return filter->output;
}

// 或者在声明时使用默认值
static LowPassFilter speed_filter = {0};
static LowPassFilter turn_filter = {0};

// 初始化运动控制滤波器
void MotionControl_Init(void)
{
    LowPassFilter_Init(&speed_filter, 0.2f, 0.0f); // 速度滤波器
    LowPassFilter_Init(&turn_filter, 0.2f, 0.0f);  // 转向滤波器
}

void MotionControl_Update(uint16_t ly, uint16_t rx)
{
    // 1. 归一化输入 (-1.0 ~ 1.0)
    // float forward_in = ((float)2048 - (float)ly) / 2048.0f;
    float forward_in = ((float)ly - 2048.0f) / 2048.0f;//霍尔摇杆
    // float turn_in = ((float)rx - (float)2048) / 2048.0f;//山地轮子和充气轮子 
    float turn_in = ((float)2048 - (float)rx) / 2048.0f;//减速器轮子

    // 2. 死区处理
    const float DEADZONE = 0.1f;
    if (fabsf(forward_in) < DEADZONE)
        forward_in = 0.0f;
    // ============================================================
    // <<<---  转向起步跃变补偿 (克服草地静摩擦) --->>>
    // ============================================================
    // 设定最小转向响应力度 (0.35 表示只要轻推，瞬间输出 35% 的转向力度)
    // 如果觉得起步还是肉，改大到 0.45f；如果起步太窜，减小到 0.25f
    const float MIN_TURN_RESPONSE = 0.45f;

    if (fabsf(turn_in) <= DEADZONE)
    {
        turn_in = 0.0f; // 在死区内，绝对静止
    }
    else
    {
        // 提取方向符号 (1.0 为右，-1.0 为左)
        float sign = (turn_in > 0.0f) ? 1.0f : -1.0f;

        // 计算超出死区后的有效行程比例 (0.0 ~ 1.0)
        float active_ratio = (fabsf(turn_in) - DEADZONE) / (1.0f - DEADZONE);

        // 重新映射：将 0.0~1.0 的行程，映射到 0.35~1.0 的超强响应区间
        turn_in = sign * (MIN_TURN_RESPONSE + active_ratio * (1.0f - MIN_TURN_RESPONSE));
    }

    // 3. 差速混合计算
    // 降低转向权重，实现差速过弯而不是单边刹死
    // 0.4 表示全速转向时，内侧轮还有 60% 的动力
    const float MIX_RATIO = 0.4f;

    float left_mix = forward_in + (turn_in * MIX_RATIO);
    float right_mix = forward_in - (turn_in * MIX_RATIO);

    // =================================================================
    //            <<<--- 【核心修改】 动态速度基准计算 ---<<<
    // =================================================================

    float current_base_speed;

     // 1. 根据遥控器设置，决定后退时的减速系数
    float reverse_factor = 1.0f; // 默认不减速
    if (g_app.cfg_reverse_limit_en == 1) {
        reverse_factor = 0.8f; // 如果开关打开，后退速度只有前进的 80%
    }

    // 判断当前主要意图是 "前进" 还是 "原地转"
    // 我们用 forward_in 的绝对值来判断
    float abs_forward = fabsf(forward_in);

    // 定义过渡区间的阈值 (例如 0.2)
    // 在 0 ~ 0.2 之间进行插值
    const float BLEND_THRESHOLD = 0.2f;

    if (abs_forward >= BLEND_THRESHOLD)
    {
        // --- 场景 A: 明显的前进/后退 ---
        // 完全使用 speed
        if (forward_in < 0.0f)
        {
           current_base_speed = g_app.speed * reverse_factor;// 后退减速
        }
        else
        {
            current_base_speed = g_app.speed; // 前进正常
        }
    }
    else
    {
        // --- 场景 B: 原地转向 或 微速移动 (处于过渡区) ---
        // 计算混合比例 (alpha): 0.0(纯原地) -> 1.0(纯移动)
        float alpha = abs_forward / BLEND_THRESHOLD;

        // 目标移动速度 (考虑后退减速)
        float target_move_speed = (forward_in < 0.0f) ? (g_app.speed * reverse_factor) : g_app.speed;

        // 线性插值公式: Result = (1-alpha)*TurnSpeed + alpha*MoveSpeed
        // 当 alpha=0 时，结果为 turn_speed
        // 当 alpha=1 时，结果为 target_move_speed
        current_base_speed = (1.0f - alpha) * g_app.turn_speed + alpha * target_move_speed;
    }

    // =================================================================

// =================================================================
    //            <<<---  坡道限速逻辑 (集成遥控开关) ---<<<
    // =================================================================
    
    // 【核心修改】 只有当遥控器上的“倾斜传感器”开关是打开时，才执行坡道限速
    if (g_app.cfg_tilt_sensor_en == 1) 
    {
        // 当 Pitch(前后坡) 或 Roll(侧倾) 超过 15度时，强制将基础速度限制
        
        // 注意：你之前的代码只判断了 roll < -15.0f，为了安全，
        // 我帮你改回了更全面的检测 (前后左右倾斜都算)
        // if (fabsf(imu_data.pitch) > 15.0f || fabsf(imu_data.roll) > 15.0f)
        // if (imu_data.roll < -15.0f)
        // {
        //     // 你这里设定的 SLOPE_MAX_SPEED 是 4.0f，请确保这是一个安全的速度值
        //     const float SLOPE_MAX_SPEED = 4.0f;

        //     // 限制正向速度
        //     if (current_base_speed > SLOPE_MAX_SPEED) 
        //     {
        //         current_base_speed = SLOPE_MAX_SPEED;
        //     }
        //     // 限制反向速度
        //     else if (current_base_speed < -SLOPE_MAX_SPEED) 
        //     {
        //         current_base_speed = -SLOPE_MAX_SPEED;
        //     }
        // }
    }
    // 如果开关是关闭的 (g_app.cfg_tilt_sensor_en == 0)，则完全跳过这段限速代码

    // 4. 计算最终 RPM
    float left_rpm = left_mix * current_base_speed;
    float right_rpm = right_mix * current_base_speed;

    // =================================================================
    //            <<<---  重力补偿计算 ---<<<
    // =================================================================

    float slope_ff = 0.0f;

    // 获取当前的俯仰角 (假设上坡 pitch 为正，下坡 pitch 为负)
    // 请务必通过串口打印确认一下你的 IMU 方向定义！
    float current_pitch = imu_data.pitch;

    // 设置一个死区，防止平地上因为传感器噪声导致车子抖动
    if (fabsf(current_pitch) > 2.0f) // 大于 2 度才启动补偿
    {
        // 1. 线性部分 (角度越大，力越大)
        float linear_part = current_pitch * GRAVITY_GAIN;

        // 2. 静摩擦补偿部分 (只要有坡度，先给一个基础力)
        float stiction_part = 0.0f;
        if (current_pitch > 0)
            stiction_part = MOTOR_STICTION_CURRENT; // 上坡给正
        else
            stiction_part = -MOTOR_STICTION_CURRENT; // 下坡给负

        // 3. 总前馈
        slope_ff = linear_part + stiction_part;
    }

    // 限制最大补偿电流 (安全起见，防止传感器出错导致飞车)
    // 例如限制在 +/- 10A 以内
    if (slope_ff > 20.0f)
        slope_ff = 20.0f;
    if (slope_ff < -20.0f)
        slope_ff = -20.0f;

    // =================================================================
    //            <<<---  转向电流前馈计算 (显式方向分配) ---<<<
    // =================================================================

    float left_turn_ff = 0.0f;
    float right_turn_ff = 0.0f;

    if (fabsf(turn_in) > 0.05f)
    {
        // 1. 计算速度衰减系数 (ff_scale)
        float ff_scale = 1.0f;

        if (abs_forward > 0.1f)
        {
            // 线性衰减，但最低保留 0.3 (30%)
            // 速度 0.1 -> Scale 1.0
            // 速度 1.0 -> Scale 0.3
            ff_scale = 1.0f - ((abs_forward - 0.1f) * 0.15f);
        }

        // 2. 计算前馈力度 (取绝对值，方向由下面 if/else 决定)
        // 这样逻辑更清晰，不会因为 turn_in 的正负号搞晕
        float raw_ff_mag = fabsf(turn_in) * TURN_FF_GAIN * ff_scale;

        // 3. 显式分配方向
        if (turn_in > 0)
        {
            // --- 右转 ---
            // 左轮：需要正向推力 (+)
            // 右轮：需要反向拉力 (-)
            left_turn_ff = raw_ff_mag;
            right_turn_ff = -raw_ff_mag;
        }
        else
        {
            // --- 左转 ---
            // 左轮：需要反向拉力 (-)
            // 右轮：需要正向推力 (+)
            left_turn_ff = -raw_ff_mag;
            right_turn_ff = raw_ff_mag;
        }
    }

    // =================================================================
    //            <<<---  直行前馈计算 ---<<<
    // =================================================================
    float linear_ff = 0.0f;

    // 只有在有直行意图时才计算
    if (fabsf(forward_in) > 0.05f)
    {
        // 简单比例前馈：油门越大，电流越大
        // forward_in 范围 -1.0 ~ 1.0
        linear_ff = forward_in * LINEAR_FF_GAIN;

        // 【可选优化】起步增强 (Stiction Kick for Linear)
        // 如果你觉得直行起步还是肉，可以加一个类似重力补偿的"起步一脚"
        /*
        if (forward_in > 0) linear_ff += 2.0f; // 前进起步加 2A
        else                linear_ff -= 2.0f; // 后退起步加 2A
        */
    }

    // 左轮总前馈 = 重力补偿 + 左轮转向前馈 + 直行前馈
    // 右轮总前馈 = 重力补偿 + 右轮转向前馈 + 直行前馈
    float left_total_ff = slope_ff + left_turn_ff + linear_ff;
    float right_total_ff = slope_ff + right_turn_ff + linear_ff;

    // 总限幅保护
    if (left_total_ff > 0.0f)
        left_total_ff = 0.0f;
    if (left_total_ff < -0.0f)
        left_total_ff = -0.0f;
    if (right_total_ff > 0.0f)
        right_total_ff = 0.0f;
    if (right_total_ff < -0.0f)
        right_total_ff = -0.0f;

    // =================================================================
    //            <<<--- 【新增】 转向反转检测 ---<<<
    // =================================================================

    static float last_turn_in = 0.0f;
    uint8_t steering_reversal = 0;

    // 检测逻辑：
    // 1. 当前在向右转 (turn > 0.1) 且 上次在向左转 (last < -0.1)
    // 2. 当前在向左转 (turn < -0.1) 且 上次在向右转 (last > 0.1)
    // 阈值 0.1 是为了防止在中间死区抖动时误触发
    if ((turn_in > 0.1f && last_turn_in < -0.1f) ||
        (turn_in < -0.1f && last_turn_in > 0.1f))
    {
        steering_reversal = 1; // 标记：发生了剧烈转向反转！
    }

    // 更新历史值
    last_turn_in = turn_in;

    // =================================================================
    //            <<<---  判断当前地形 ---<<<
    // =================================================================

    float abs_roll = fabsf(imu_data.roll);
    float abs_pitch = fabsf(imu_data.pitch);

    // 阈值设定：大于 3 度认为是坡道
    // 你可以根据实际测试微调这个值，比如 2.5 或 4.0
    if (abs_roll > 15.0f || abs_pitch > 15.0f)
    {
        g_app.is_on_slope = 1;
    }
    else
    {
        g_app.is_on_slope = 0;
    }

    // 右侧电机 (Axis 0, 1)
    // 速度: right_rpm
    // 前馈: right_ff
    // 只有当 steering_reversal = 1 时，v_ctl_uart 才会强制清零 I 项
    // is_on_slope 参数用于指示当前是否在坡道上 0 是平地 1 是坡道
    // v_ctl_uart(0, right_rpm, right_total_ff, steering_reversal, g_app.is_on_slope);
    // v_ctl_uart(1, right_rpm, right_total_ff, steering_reversal, g_app.is_on_slope);

    // // v_ctl_uart(0, right_rpm, 0);
    // // v_ctl_uart(1, right_rpm, 0);
    // // 左侧电机 (Axis 2, 3)
    // // 你的代码逻辑是：左侧电机物理安装反向，所以速度发负值 (-left_rpm)
    // // 【关键】前馈电流的方向必须和速度方向一致！
    // // 所以前馈也要取反: -left_ff
    // v_ctl_uart(2, -left_rpm, -left_total_ff, steering_reversal, g_app.is_on_slope);
    // v_ctl_uart(3, -left_rpm, -left_total_ff, steering_reversal, g_app.is_on_slope);


     // ============================================================
    // 霍尔摇杆**执行控制：由于前进入参取反了，这里必须重新映射物理方向
    // 经推导，要使车子前进：右侧(0,1)需发负值，左侧(2,3)需发正值
    // ============================================================
    // 右侧电机 (Axis 0, 1) -> 取反 (-)
    // v_ctl_uart(0, -right_rpm, -right_total_ff, steering_reversal, g_app.is_on_slope);
    // v_ctl_uart(1, -right_rpm, -right_total_ff, steering_reversal, g_app.is_on_slope);

    // // 左侧电机 (Axis 2, 3) -> 正常 (+)
    // v_ctl_uart(2, left_rpm, left_total_ff, steering_reversal, g_app.is_on_slope);
    // v_ctl_uart(3, left_rpm, left_total_ff, steering_reversal, g_app.is_on_slope);

        // 右侧电机 (0, 1)
    Send_Motor_Command(1, -right_rpm, -right_total_ff, steering_reversal);
    Send_Motor_Command(2,-right_rpm, -right_total_ff, steering_reversal);

    // 左侧电机 (2, 3) - 物理反向
    Send_Motor_Command(3, left_rpm, left_total_ff, steering_reversal);
    Send_Motor_Command(4, left_rpm, left_total_ff, steering_reversal);
}


void v_ctl_uart(uint8_t axis_id, float speed_rpm, float current_feedforward, uint8_t force_reset, uint8_t is_slope_mode)
{
    SerialPort_ID port_id;
    uint8_t motor_axis_id;

    // 状态记录数组：必须使用 static 关键字，且长度为4，分别记录4个电机的状态
    // 用于记录上一周期的速度
    static float last_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f}; 
    // 用于记录还剩下多少次清零指令需要发送
    static uint8_t reset_counter[4] = {0, 0, 0, 0};      

    // 1. 端口映射
    if (axis_id == 1 || axis_id == 2)
    {
        port_id = SERIAL_PORT_1;
        motor_axis_id = (axis_id == 1) ? 0 : 1;
        
    }
    else if (axis_id == 3 || axis_id == 4)
    {
        port_id = SERIAL_PORT_2;
        motor_axis_id = (axis_id == 3) ? 0 : 1;
    }
    else
    {
        return; // 防止非法的 axis_id 导致数组越界
    }

    // 2. 数据格式化
    int speed_int = (int)(speed_rpm * 10);
    int speed_abs = (speed_int < 0) ? -speed_int : speed_int;
    const char *speed_sign = (speed_int < 0) ? "-" : "";

    int current_int = (int)(current_feedforward * 10);
    int current_abs = (current_int < 0) ? -current_int : current_int;
    const char *current_sign = (current_int < 0) ? "-" : "";

    // 3. 发送速度指令 (正常运动控制)
    Serial_Printf(port_id, "v %d %s%d.%d %s%d.%d\n",
                  motor_axis_id,
                  speed_sign, speed_abs / 10, speed_abs % 10,
                  current_sign, current_abs / 10, current_abs % 10);

    // ============================================================
    //            <<<---  智能积分清零逻辑 (升级版) ---<<<
    // ============================================================

    // 逻辑 A: 强制复位 (用于转向反转)
    // if (force_reset)
    // {
    //     Serial_Printf(port_id, "w axis%d.controller.vel_integrator_torque 0\n", motor_axis_id);
    //     reset_counter[axis_id] = 0; // 发生强制复位时，打断停车清零计数
    // }
    // else
    // {
    //     // 逻辑 B: 停车处理 - 边缘检测 (速度从非0变成0的瞬间)
    //     if (fabsf(last_speed[axis_id]) >= 0.01f && fabsf(speed_rpm) < 0.01f)
    //     {
    //         if (is_slope_mode == 0) {
    //             reset_counter[axis_id] = 20; // 平地：彻底清零，释放电流
    //         } 
    //         // else {
    //         //     reset_counter[axis_id] = 1;  // 坡道：只清零2次(瞬间释放因转向积攒的异常I项)，然后立刻让I项恢复工作对抗重力
    //         // }
    //     }

    //     // 防御性设计：如果在清零期间，遥控器又推动了（速度不再为0），立刻打断清零动作
    //     if (fabsf(speed_rpm) >= 0.01f)
    //     {
    //         reset_counter[axis_id] = 0; 
    //     }

    //     // 如果计数器不为 0，说明当前正处于“停车后的余波”中，需要发送清零指令
    //     if (reset_counter[axis_id] > 0)
    //     {
    //         Serial_Printf(port_id, "w axis%d.controller.vel_integrator_torque 0\n", motor_axis_id);
    //         reset_counter[axis_id]--; // 发送一次，计数器减 1
    //     }
    // }

    // 4. 更新历史速度，供下个控制周期进行边沿判断
    last_speed[axis_id] = speed_rpm;
}

/**
 * @brief 电子手刹/停车保持模式
 * 仅利用 IMU 数据产生抗重力电流，保持机器静止
 */
void MotionControl_Parking(uint8_t is_slope_mode)
{
    // 1. 获取坡度
    float current_pitch = imu_data.pitch;
    float slope_ff = 0.0f;

    // 2. 计算重力前馈 (与 MotionControl_Update 逻辑完全一致)
    if (fabsf(current_pitch) > 2.0f)
    {
        float linear_part = current_pitch * GRAVITY_GAIN;
        float stiction_part = 0.0f;

        if (current_pitch > 0)
            stiction_part = MOTOR_STICTION_CURRENT;
        else
            stiction_part = -MOTOR_STICTION_CURRENT;

        slope_ff = linear_part + stiction_part;
    }

    // 3. 安全限幅 (停车时不需要太大的爆发力，20A足够)
    if (slope_ff > 20.0f)
        slope_ff = 20.0f;
    if (slope_ff < -20.0f)
        slope_ff = -20.0f;

    // 4. 主动抗衡逻辑 (Active Holding)
    // 即使目标是停车，也要给一个微小的速度指令，唤醒 ODrive 的 PID
    float hold_rpm = 0.0f;

    // 5. 发送指令
    // 所有轮子动作一致：左轮物理反向，所以发负

    // 右侧电机
    v_ctl_uart(0, 0, slope_ff, 0, is_slope_mode);
    v_ctl_uart(1, 0, slope_ff, 0, is_slope_mode);

    // 左侧电机 (物理反向)
    v_ctl_uart(2, 0, -slope_ff, 0, is_slope_mode);
    v_ctl_uart(3, 0, -slope_ff, 0, is_slope_mode);
}

/**
 * @brief 统一的电机指令发送函数 (支持 CAN / UART 切换)
 */
void Send_Motor_Command(uint8_t axis_id, float speed_rpm, float torque_ff, uint8_t reset_integrator)
{

    
    // 1. 单位转换: Turns/s
    float vel_rps = speed_rpm ;
    
    odrive_set_input_vel(&hcan1, axis_id, vel_rps, torque_ff);
    
    SerialPort_ID port_id;
    uint8_t motor_axis_id;

    // 1. 端口映射 (保持不变)
    if (axis_id == 1 || axis_id == 2)
    {
        port_id = SERIAL_PORT_1;
        motor_axis_id = (axis_id == 1) ? 0 : 1;
        
    }
    else if (axis_id == 3 || axis_id == 4)
    {
        port_id = SERIAL_PORT_2;
        motor_axis_id = (axis_id == 3) ? 0 : 1;
    }
    else
        return;

    // 2. 数据格式化 (保持不变，使用优化过的无空格格式)
    int speed_int = (int)(speed_rpm * 10);
    int speed_abs = (speed_int < 0) ? -speed_int : speed_int;
    const char *speed_sign = (speed_int < 0) ? "-" : "";

    int current_int = (int)(torque_ff * 10);
    int current_abs = (current_int < 0) ? -current_int : current_int;
    const char *current_sign = (current_int < 0) ? "-" : "";

    // 3. 发送速度指令 (保持不变)
    Serial_Printf(port_id, "v %d %s%d.%d %s%d.%d\n",
                  motor_axis_id,
                  speed_sign, speed_abs / 10, speed_abs % 10,
                  current_sign, current_abs / 10, current_abs % 10);

}