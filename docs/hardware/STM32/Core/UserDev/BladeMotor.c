#include "BladeMotor.h"
#include "math.h"
#include "tim.h"

HAL_StatusTypeDef PWM_SetDutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, float duty_cycle)
{
    uint32_t period;
    uint32_t pulse;

    if (duty_cycle < 0.0f || duty_cycle > 100.0f || htim == NULL)
    {
        return HAL_ERROR;
    }

    period = htim->Init.Period + 1U;
    if (duty_cycle <= 0.01f)
    {
        pulse = 0U;
    }
    else if (duty_cycle >= 99.99f)
    {
        pulse = period - 1U;
    }
    else
    {
        pulse = (uint32_t)(period * duty_cycle / 100.0f);
    }

    if (pulse >= period)
    {
        pulse = period - 1U;
    }

    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
    return HAL_OK;
}

#if BLADE_MOTOR_DRIVER_USED == BLADE_MOTOR_DRIVER_PWM

HAL_StatusTypeDef BladeMotor_SetSpeed(float speed_percent)
{
    if (speed_percent < 0.0f || speed_percent > 100.0f)
    {
        return HAL_ERROR;
    }

    return PWM_SetDutyCycle(&htim2, TIM_CHANNEL_2, speed_percent);
}

void BladeMotor_Start(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void BladeMotor_Stop(void)
{
    (void)PWM_SetDutyCycle(&htim2, TIM_CHANNEL_2, 0.0f);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
}
#elif BLADE_MOTOR_DRIVER_USED == BLADE_MOTOR_DRIVER_FLIPSKY_CAN

#include "flipsky_can.h"

static float s_last_can_percent = -1.0f;
static uint32_t s_last_send_tick = 0; // 记录上次发送时间戳

// HAL_StatusTypeDef BladeMotor_SetSpeed(float speed_percent)
// {
//     if (speed_percent < 0.0f || speed_percent > 100.0f || !isfinite(speed_percent))
//     {
//         (void)Flipsky_CAN_Stop();
//         s_last_can_percent = -1.0f;
//         return HAL_ERROR;
//     }

//     uint32_t now = HAL_GetTick();

//     // 【核心修复】：如果是开刀状态(>0)，每隔 200ms 必须强行重发一次进行心跳保活！
//     // 只有在 200ms 间隔内 且 数值没有变化时，才跳过发送
//     if (s_last_can_percent == speed_percent && (now - s_last_send_tick < 200))
//     {
//         return HAL_OK;
//     }

//     // if (Flipsky_CAN_SetCurrentPercent(speed_percent) == 0U)
//     // {
//     //     return HAL_ERROR;
//     // }

//     // 【修改点】：调用的函数改为 Flipsky_CAN_SetDutyCycle
//     if (Flipsky_CAN_SetDutyCycle(speed_percent) == 0U)
//     {
//         return HAL_ERROR;
//     }
    

//     s_last_can_percent = speed_percent;
//     s_last_send_tick = now; // 更新发送时间
//     return HAL_OK;
// }



HAL_StatusTypeDef BladeMotor_SetSpeed(float speed_erpm)
{
    if (speed_erpm < 0.0f || !isfinite(speed_erpm))
    {
        (void)Flipsky_CAN_Stop();
        s_last_can_percent = -1.0f;
        return HAL_ERROR;
    }

    uint32_t now = HAL_GetTick();

    // 200ms 心跳保活重发
    if (s_last_can_percent == speed_erpm && (now - s_last_send_tick < 200))
    {
        return HAL_OK;
    }

    // 【修改点】：调用转速发送函数
    if (Flipsky_CAN_SetSpeedErpm(speed_erpm) == 0U)
    {
        return HAL_ERROR;
    }

    s_last_can_percent = speed_erpm;
    s_last_send_tick = now;
    return HAL_OK;
}


void BladeMotor_Start(void)
{
    Flipsky_CAN_Init();
    (void)BladeMotor_SetSpeed(0.0f);
}

// void BladeMotor_Stop(void)
// {
//     if (s_last_can_percent != 0.0f)
//     {
//         (void)Flipsky_CAN_Stop();
//         s_last_can_percent = 0.0f;
//     }
// }

// void BladeMotor_Stop(void)
// {
//     if (s_last_can_percent != 0.0f)
//     {
//         // 停止时给 0.0% 占空比
//         (void)Flipsky_CAN_SetDutyCycle(0.0f);
//         s_last_can_percent = 0.0f;
//     }
// }

void BladeMotor_Stop(void)
{
    if (s_last_can_percent != 0.0f)
    {
        (void)Flipsky_CAN_SetSpeedErpm(0.0f);
        s_last_can_percent = 0.0f;
    }
}

#else
#error "Unsupported BLADE_MOTOR_DRIVER_USED"
#endif
