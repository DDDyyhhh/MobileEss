/**
 ******************************************************************************
 * @file           : BladeMotor.c
 * @brief          : 割草机刀盘电机 (Blade Motor) PWM 调速驱动
 * @author         : [Your Name/Team]
 * @version        : V1.3.4
 * @date           : 2026-03-11
 * 主要功能：
 * 1. 将 0.0% ~ 100.0% 的占空比请求映射为底层定时器 (TIM2_CH2) 的 Compare 寄存器值
 * 2. 提供刀盘电机的安全启动 (Start) 与 停止 (Stop) 接口
 *
 * 注意事项：
 * - 采用硬件 PWM 输出，修改寄存器后立即生效，无软件延时
 * - 进行了严格的边界限幅保护，防止占空比异常导致定时器寄存器溢出或死锁
 ******************************************************************************
 */

#include <tim.h>
#include <BladeMotor.h>
#include <math.h>

#define BLADE_SWITCH_GPIO_PORT GPIOE
#define BLADE_SWITCH_GPIO_PIN  GPIO_PIN_5

/* PE5 is an active-low hardware enable for the blade motor. */
static void BladeMotor_SetSwitch(uint8_t on)
{
    HAL_GPIO_WritePin(BLADE_SWITCH_GPIO_PORT,
                      BLADE_SWITCH_GPIO_PIN,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

HAL_StatusTypeDef PWM_SetDutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, float duty_cycle)
{
    // 参数检查
    if (duty_cycle < 0.0f || duty_cycle > 100.0f)
    {
        return HAL_ERROR;
    }

    if (htim == NULL)
    {
        return HAL_ERROR;
    }

    // 计算对应的脉冲值
    uint32_t period = htim->Init.Period + 1;

    // 处理边界情况
    uint32_t pulse;
    if (duty_cycle <= 0.01f)
    {
        pulse = 0; // 0% 占空比
    }
    else if (duty_cycle >= 99.99f)
    {
        pulse = period - 1; // 100% 占空比
    }
    else
    {
        pulse = (uint32_t)(period * duty_cycle / 100.0f);
    }

    // 确保 pulse 在有效范围内
    if (pulse >= period)
    {
        pulse = period - 1;
    }

    // 设置比较值
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);

    return HAL_OK;
}

HAL_StatusTypeDef BladeMotor_SetSpeed(float speed_percent)
{
    
    if (speed_percent < 0.0f || speed_percent > 100.0f)
    {
        BladeMotor_SetSwitch(0);
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = PWM_SetDutyCycle(&htim2, TIM_CHANNEL_2, speed_percent);
    if (status != HAL_OK)
    {
        BladeMotor_SetSwitch(0);
        return status;
    }

    BladeMotor_SetSwitch(speed_percent > 0.01f ? 1 : 0);
    return HAL_OK;
}

// 启动刀片电机
void BladeMotor_Start(void)
{
    BladeMotor_SetSwitch(0);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // 使用HAL库函数
}

// 停止刀片电机
void BladeMotor_Stop(void)
{
    BladeMotor_SetSwitch(0);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); // 使用HAL库函数
}
