#ifndef BLADE_MOTOR_H
#define BLADE_MOTOR_H

#include "blade_motor_config.h"
#include "stm32f4xx_hal.h"

HAL_StatusTypeDef PWM_SetDutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, float duty_cycle);

#if BLADE_MOTOR_DRIVER_USED == BLADE_MOTOR_DRIVER_PWM
#include "tim.h"
#endif

HAL_StatusTypeDef BladeMotor_SetSpeed(float speed_percent);
void BladeMotor_Start(void);
void BladeMotor_Stop(void);

#endif
