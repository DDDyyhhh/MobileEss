/**
 ******************************************************************************
 * @file           : bsp_imu_solver.c
 * @brief          : 基于 Mahony 互补滤波的 6 轴 IMU 姿态解算引擎
 *
 * 主要功能：
 * 1. `IMU_Solver_Init` : 通过纯加速度向量计算出初始的 Pitch 和 Roll 及四元数
 * 2. `IMU_Solver_Update`: 核心 Mahony 算法实现 (快速平方根倒数 + 一阶龙格库塔积分)
 *    - 利用加速度估算重力向量，与陀螺仪积分的重力向量进行叉乘求误差
 *    - 使用 PI 控制器 (Kp, Ki) 修正陀螺仪零偏，最终输出 Roll, Pitch, Yaw 欧拉角
 * 3. 提供了静止零偏校准接口 (`IMU_Solver_SetOffset`)
 *
 * 注意事项：
 * - 算法假设机体绝大部分时间处于低频运动状态（重力占主导）。
 * - 由于没有磁力计 (9轴)，Yaw 轴 (航向角) 长时间运行会存在缓慢漂移。
 * - `dt` 参数的准确性对积分至关重要，调用频率必须稳定 (建议 100Hz 以上)。
 ******************************************************************************
 */

#include "bsp_imu_solver.h"

/* ================= 参数设置 ================= */
#define Kp 3.0f  // 比例增益 (控制加速度计修正速度)
#define Ki 0.01f // 积分增益 (消除陀螺仪漂移)

/* ================= 变量定义 ================= */
IMU_Data_t imu_data = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f; // 积分误差

// 辅助函数：快速平方根倒数
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/* 【新增】设置零偏函数 */
void IMU_Solver_SetOffset(float gx_off, float gy_off, float gz_off)
{
    imu_data.gyro_offset_x = gx_off;
    imu_data.gyro_offset_y = gy_off;
    imu_data.gyro_offset_z = gz_off;
}

/**
 * @brief  初始化 IMU，利用加速度计计算初始姿态
 * @param  ax, ay, az: 初始加速度数据 (单位任意，如 mg)
 */
void IMU_Solver_Init(float ax, float ay, float az)
{
    // 1. 归一化加速度
    float norm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // 2. 根据加速度计算初始 Pitch 和 Roll
    // 注意：这里假设静止，且 Z 轴向下为正
    float pitch = -asin(ax);
    float roll = atan2(ay, az);
    float yaw = 0.0f; // 6轴无法计算初始 Yaw，设为 0

    // 3. 将欧拉角转换为初始四元数
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);

    imu_data.q0 = cy * cp * cr + sy * sp * sr;
    imu_data.q1 = cy * cp * sr - sy * sp * cr;
    imu_data.q2 = cy * sp * cr + sy * cp * sr;
    imu_data.q3 = sy * cp * cr - cy * sp * sr;

    // 清除积分误差
    exInt = 0;
    eyInt = 0;
    ezInt = 0;
}

/**
 * @brief  Mahony 6轴互补滤波算法
 * @param  gx, gy, gz: 陀螺仪数据 (单位：deg/s)
 * @param  ax, ay, az: 加速度数据 (单位：任意，如 mg)
 * @param  dt: 采样周期 (单位：秒)
 */
void IMU_Solver_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float q0 = imu_data.q0, q1 = imu_data.q1, q2 = imu_data.q2, q3 = imu_data.q3;

    // 1. 如果加速度计数据无效，只进行陀螺仪积分
    if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))
    {
        return;
    }

    // =========================================================
    // <<<--- 【核心修改】 第一步：先减去零偏！ --->>>
    // =========================================================
    gx -= imu_data.gyro_offset_x;
    gy -= imu_data.gyro_offset_y;
    gz -= imu_data.gyro_offset_z;

    // 2. 将陀螺仪单位从 度/秒 转换为 弧度/秒
    gx *= 0.0174533f; // (pi / 180)
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // 3. 加速度归一化
    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // 4. 估计重力方向 (将四元数推算的重力向量转到机体坐标系)
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 5. 计算误差 (加速度测量值 与 重力估计值 的叉积)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // 6. 积分误差 (PI 控制器中的 I)
    exInt += ex * Ki * dt;
    eyInt += ey * Ki * dt;
    ezInt += ez * Ki * dt;

    // 7. 修正陀螺仪数据 (PI 控制器中的 P)
    gx += Kp * ex + exInt;
    gy += Kp * ey + eyInt;
    gz += Kp * ez + ezInt;

    // 8. 四元数微分方程 (一阶龙格库塔法)
    float q0_last = q0;
    float q1_last = q1;
    float q2_last = q2;
    float q3_last = q3;

    q0 += (-q1_last * gx - q2_last * gy - q3_last * gz) * (0.5f * dt);
    q1 += (q0_last * gx + q2_last * gz - q3_last * gy) * (0.5f * dt);
    q2 += (q0_last * gy - q1_last * gz + q3_last * gx) * (0.5f * dt);
    q3 += (q0_last * gz + q1_last * gy - q2_last * gx) * (0.5f * dt);

    // 9. 四元数归一化
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    imu_data.q0 = q0 * norm;
    imu_data.q1 = q1 * norm;
    imu_data.q2 = q2 * norm;
    imu_data.q3 = q3 * norm;

    // 10. 计算欧拉角 (单位：度)
    // Yaw
    imu_data.yaw = atan2f(2.0f * (imu_data.q1 * imu_data.q2 + imu_data.q0 * imu_data.q3),
                          imu_data.q0 * imu_data.q0 + imu_data.q1 * imu_data.q1 - imu_data.q2 * imu_data.q2 - imu_data.q3 * imu_data.q3) *
                   57.29578f;
    // Pitch
    imu_data.pitch = -asinf(2.0f * (imu_data.q1 * imu_data.q3 - imu_data.q0 * imu_data.q2)) * 57.29578f;
    // Roll
    imu_data.roll = atan2f(2.0f * (imu_data.q0 * imu_data.q1 + imu_data.q2 * imu_data.q3),
                           imu_data.q0 * imu_data.q0 - imu_data.q1 * imu_data.q1 - imu_data.q2 * imu_data.q2 + imu_data.q3 * imu_data.q3) *
                    57.29578f;
}