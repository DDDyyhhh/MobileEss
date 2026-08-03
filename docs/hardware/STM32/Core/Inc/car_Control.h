// 在car_Control.h中定义
// 定义四个电机的ODrive实例
#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H
#include"stdint.h"
// void ApplyDeadzone(float* value, float deadzone);
// // 差速控制函数
// void DifferentialControl_Update(float forward_speed_rpm, float turn_rate);
// void Car_SetMovement(float forward_speed_rpm, float turn_rate);
// void DifferentialControl_Stop();

extern float g_current_target_speed_rpm; // 用于回传的目标速度 (turns/s)
void MotionControl_Update(uint16_t ly, uint16_t rx);

// void MotionControl_Init(void) ;
// void MotionControl_Parking(void);
void MotionControl_Parking(uint8_t is_slope_mode);
// void v_ctl_uart(uint8_t axis_id, float speed_rpm, float current_feedforward);
void v_ctl_uart(uint8_t axis_id, float speed_rpm, float current_feedforward,uint8_t force_reset,uint8_t is_slope_mode);
void Send_Motor_Command(uint8_t axis_id, float speed_rpm, float torque_ff, uint8_t reset_integrator);
#endif // __CAR_CONTROL_H




