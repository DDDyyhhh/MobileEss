// buzzer.c

#include "buzzer.h"
#include "tim.h" // 包含由CubeMX生成的tim.h，以获取htim4

// --- 硬件配置 ---
#define BUZZER_USE_PWM      0 // 1: 使用PWM驱动无源蜂鸣器, 0: 使用GPIO驱动有源蜂鸣器

#if BUZZER_USE_PWM
    // PWM配置
    #define BUZZER_TIM_HANDLE   htim4
    #define BUZZER_TIM_CHANNEL  TIM_CHANNEL_1
#else
    // GPIO配置
    #define BUZZER_GPIO_PORT    GPIOD
    #define BUZZER_GPIO_PIN     GPIO_PIN_11
#endif

// --- 内部状态变量 ---
static BuzzerMode_t current_mode = BUZZER_MODE_OFF;
static uint32_t state_timer = 0;
static uint8_t  state_step = 0;
extern TIM_HandleTypeDef htim7; 
// 内部函数，用于控制蜂鸣器的物理开关
static void Buzzer_Switch(uint8_t on)
{
#if BUZZER_USE_PWM
    if (on) {
        HAL_TIM_PWM_Start(&BUZZER_TIM_HANDLE, BUZZER_TIM_CHANNEL);
    } else {
        HAL_TIM_PWM_Stop(&BUZZER_TIM_HANDLE, BUZZER_TIM_CHANNEL);
    }
#else
     if (on) {
        HAL_TIM_Base_Start_IT(&htim7); // 启动定时器中断
    } else {
        HAL_TIM_Base_Stop_IT(&htim7);  // 停止定时器中断
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET); // 确保最后是低电平
    }
#endif
}

void Buzzer_Init(void)
{
    Buzzer_Switch(0); // 默认关闭
}

void Buzzer_SetMode(BuzzerMode_t mode)
{
    current_mode = mode;
    state_timer = HAL_GetTick(); // 重置状态计时器
    state_step = 0;             // 重置状态步骤

    // 对于立即生效的模式，直接操作
    if (mode == BUZZER_MODE_OFF) {
        Buzzer_Switch(0);
    } else if (mode == BUZZER_MODE_ON) {
        Buzzer_Switch(1);
    }
}

void Buzzer_Process(void)
{
    uint32_t current_time = HAL_GetTick();

    switch (current_mode)
    {
        case BUZZER_MODE_OFF:
        case BUZZER_MODE_ON:
            // 这两种模式是持续的，不需要处理
            break;

        case BUZZER_MODE_SHORT: // 短响模式: 响100ms, 然后自动关闭
            if (state_step == 0) {
                Buzzer_Switch(1);
                state_step = 1;
            } else if (state_step == 1 && (current_time - state_timer >= 100)) {
                Buzzer_Switch(0);
                current_mode = BUZZER_MODE_OFF; // 完成后回到OFF状态
            }
            break;

        case BUZZER_MODE_DOUBLE: // 双响模式: 响100ms, 停100ms, 响100ms, 然后关闭
            if (state_step == 0) { // 第一次响
                Buzzer_Switch(1);
                state_step = 1;
            } else if (state_step == 1 && (current_time - state_timer >= 100)) { // 第一次停
                Buzzer_Switch(0);
                state_step = 2;
            } else if (state_step == 2 && (current_time - state_timer >= 200)) { // 第二次响
                Buzzer_Switch(1);
                state_step = 3;
            } else if (state_step == 3 && (current_time - state_timer >= 300)) { // 第二次停 & 结束
                Buzzer_Switch(0);
                current_mode = BUZZER_MODE_OFF;
            }
            break;

        case BUZZER_MODE_LONG: // 长响模式: 响500ms, 然后关闭
            if (state_step == 0) {
                Buzzer_Switch(1);
                state_step = 1;
            } else if (state_step == 1 && (current_time - state_timer >= 500)) {
                Buzzer_Switch(0);
                current_mode = BUZZER_MODE_OFF;
            }
            break;
            
        case BUZZER_MODE_ALARM: // 警报模式: 响200ms, 停200ms, 循环
            if (state_step == 0 && (current_time - state_timer >= 400)) {
                state_timer = current_time; // 重新计时
            }
            if (current_time - state_timer < 200) {
                Buzzer_Switch(1);
            } else {
                Buzzer_Switch(0);
            }
            break;
    }
}