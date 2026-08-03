#ifndef __BSP_IMU_SOLVER_H
#define __BSP_IMU_SOLVER_H

#include <math.h>
#include "main.h" // 为了使用 uint32_t 等类型

typedef struct {
    float q0, q1, q2, q3; // 四元数
    float roll, pitch, yaw; // 欧拉角 (单位：度)

    // 【新增】存储陀螺仪的三轴零偏值
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;
} IMU_Data_t;

// 全局变量声明
extern IMU_Data_t imu_data;

// 初始化函数
void IMU_Solver_Init(float ax, float ay, float az);

// 解算更新函数 (输入单位：加速度任意单位，陀螺仪 deg/s，时间秒)
void IMU_Solver_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

// 【新增】设置零偏的函数声明
void IMU_Solver_SetOffset(float gx_off, float gy_off, float gz_off);

#endif