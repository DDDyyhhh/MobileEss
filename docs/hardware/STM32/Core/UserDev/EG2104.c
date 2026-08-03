/**
  ******************************************************************************
  * @file           : EG2104.c
  * @brief          : EG2104 栅极驱动器（H桥）底层 PWM 控制驱动
  * 
  * 主要功能：
  * 1. 负责底盘推杆电机 (Lifter) 等大功率有刷直流电机的正反转与速度控制
  * 2. 将 -100.0f ~ 100.0f 的速度指令解析为双路互补或独立的 PWM 占空比输出
  * 3. 封装了安全的停止接口 (`EG2104_HBridge_Stop`)
  *
  * 注意事项：
  * - 当速度为正时，左侧输出 PWM，右侧拉低；为负时则相反。
  * - 死区时间 (Dead-Time) 目前由硬件或 CubeMX 定时器高级配置保证，
  *   切勿在软件层同时拉高两侧的 IN 引脚，否则会造成桥臂直通短路。
  ******************************************************************************
  */

#include "tim.h"
#include "EG2104.h"
#include "math.h"
#include "BladeMotor.h"
#include "serial_port.h"
// EG2104_HBridge_t front_bridge = {
//     &htim3,
//     &htim3,        // 左右都使用TIM3
//     TIM_CHANNEL_3, // 左侧IN (TIM3_CH3)
//     TIM_CHANNEL_4, // 右侧IN (TIM3_CH4)
//     0.0f,          // 当前速度
//     0,             // 方向
//     0,             // 上次移动时间戳
//     0,             // 是否正在移动
// };

// EG2104_HBridge_t behind_bridge = {
//     &htim2,
//     &htim3,        // 左侧用TIM2，右侧用TIM3
//     TIM_CHANNEL_4, // 左侧IN (TIM2_CH4)
//     TIM_CHANNEL_2, // 右侧IN (TIM3_CH2)

//     0.0f, // 当前速度
//     0,    // 方向
//     0,    // 上次移动时间戳
//     0,    // 是否正在移动
// };

// EG2104 H桥初始化
void EG2104_HBridge_Init(EG2104_HBridge_t *bridge)
{
    // 启动PWM输出（用于IN控制）
    HAL_TIM_PWM_Start(bridge->left_timer, bridge->left_in_channel);
    HAL_TIM_PWM_Start(bridge->right_timer, bridge->right_in_channel);

    // 配置SD引脚为输出模式
    // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;

    // // 使能芯片 (SD=1)
    // HAL_GPIO_WritePin(bridge->sd_port, bridge->sd_pin, GPIO_PIN_SET);

    // 初始状态：停止
    EG2104_HBridge_Stop(bridge);
}

// 设置EG2104 H桥速度和方向
HAL_StatusTypeDef EG2104_HBridge_SetSpeed(EG2104_HBridge_t *bridge, float speed)
{
    if (speed < -100.0f || speed > 100.0f)
    {
        return HAL_ERROR;
    }

    bridge->current_speed = speed;
    float duty_cycle = fabsf(speed);

    if (speed > 0)
    {
        // 正转：左侧IN=1(HO高) + 右侧IN=0(LO高)
        bridge->direction = 1;
        PWM_SetDutyCycle(bridge->left_timer, bridge->left_in_channel, duty_cycle); // 左侧PWM
        PWM_SetDutyCycle(bridge->right_timer, bridge->right_in_channel, 0.0f);     // 右侧关闭
    }
    else if (speed < 0)
    {
        // 反转：左侧IN=0(LO高) + 右侧IN=1(HO高)
        bridge->direction = 2;
        PWM_SetDutyCycle(bridge->left_timer, bridge->left_in_channel, 0.0f);         // 左侧关闭
        PWM_SetDutyCycle(bridge->right_timer, bridge->right_in_channel, duty_cycle); // 右侧PWM
    }
    else
    {
        // 停止
        Serial_Printf(SERIAL_PORT_2, "!!! SetSpeed called with speed ~ 0. Stopping. !!!\r\n");
        EG2104_HBridge_Stop(bridge);
    }

    return HAL_OK;
}

// 停止EG2104 H桥
void EG2104_HBridge_Stop(EG2104_HBridge_t *bridge)
{
    bridge->direction = 0;
    bridge->current_speed = 0.0f;

    // 关闭两个IN的PWM输出
    PWM_SetDutyCycle(bridge->left_timer, bridge->left_in_channel, 0.0f);
    PWM_SetDutyCycle(bridge->right_timer, bridge->right_in_channel, 0.0f);
}




// // 紧急停止（使用SD引脚）
// void EG2104_HBridge_EmergencyStop(EG2104_HBridge_t* bridge) {
//     // SD=0 强制停止
//     HAL_GPIO_WritePin(bridge->sd_port, bridge->sd_pin, GPIO_PIN_RESET);
//     bridge->direction = 0;
//     bridge->current_speed = 0.0f;
// }

// // 恢复使能
// void EG2104_HBridge_Enable(EG2104_HBridge_t* bridge) {
//     // SD=1 恢复使能
//     HAL_GPIO_WritePin(bridge->sd_port, bridge->sd_pin, GPIO_PIN_SET);
// }