#include<tim.h>
HAL_StatusTypeDef PWM_SetDutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, float duty_cycle);
// 刀片电机控制函数
HAL_StatusTypeDef BladeMotor_SetSpeed(float speed_percent);
void BladeMotor_Start(void);
void BladeMotor_Stop(void);