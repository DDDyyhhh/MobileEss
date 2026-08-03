/**
  ******************************************************************************
  * @file           : tricolor_led.c
  * @brief          : RGB 三色状态指示灯 (非阻塞型) 控制模块
  * 
  * 主要功能：
  * 1. 封装了物理 GPIO 引脚，提供各种常规颜色 (红/绿/蓝/黄/紫/青/白) 的快捷设置
  * 2. 实现了软件定时器式的闪烁状态机 (`RGB_LED_ProcessBlink`)
  * 3. 允许独立控制 LED1 和 LED2 以不同的频率和颜色闪烁
  *
  * 注意事项：
  * - 为了实现不阻塞其他任务的闪烁效果，必须确保 `RGB_LED_ProcessBlink()` 
  *   在主循环或特定任务中被高频调用 (例如 10ms ~ 50ms)。
  ******************************************************************************
  */

#include "tricolor_led.h"

// --- GPIO 引脚定义 (根据你的硬件) ---
// LED1: PE2(红), PE4(绿), PE3(蓝)
#define LED1_RED_GPIO_PORT      GPIOE
#define LED1_RED_GPIO_PIN       GPIO_PIN_2
#define LED1_GREEN_GPIO_PORT    GPIOE
#define LED1_GREEN_GPIO_PIN     GPIO_PIN_4
#define LED1_BLUE_GPIO_PORT     GPIOE
#define LED1_BLUE_GPIO_PIN      GPIO_PIN_3

// LED2: PC8(红), PD14(绿), PD13(蓝)
#define LED2_RED_GPIO_PORT      GPIOC
#define LED2_RED_GPIO_PIN       GPIO_PIN_8
#define LED2_GREEN_GPIO_PORT    GPIOD
#define LED2_GREEN_GPIO_PIN     GPIO_PIN_13
#define LED2_BLUE_GPIO_PORT     GPIOD
#define LED2_BLUE_GPIO_PIN      GPIO_PIN_14

// --- 内部宏定义 ---
#define LED_ON  GPIO_PIN_SET
#define LED_OFF GPIO_PIN_RESET

// --- 内部静态变量 ---
// 闪烁控制变量
static uint32_t led1_blink_timer = 0;
static uint32_t led2_blink_timer = 0;
static uint8_t  led1_blink_enabled = 0;
static uint8_t  led2_blink_enabled = 0;
static uint8_t  led1_blink_state_on = 0; // 0=灭, 1=亮
static uint8_t  led2_blink_state_on = 0;
static RGB_Color_t led1_blink_color = RGB_COLOR_OFF;
static RGB_Color_t led2_blink_color = RGB_COLOR_OFF;
static uint32_t led1_blink_interval = 500;
static uint32_t led2_blink_interval = 500;


// 内部辅助函数，用于实际控制GPIO电平，避免代码重复
static void _LED_WritePins(LED_Number_t led_num, GPIO_PinState r_state, GPIO_PinState g_state, GPIO_PinState b_state)
{
    if (led_num == LED_1) {
        HAL_GPIO_WritePin(LED1_RED_GPIO_PORT,   LED1_RED_GPIO_PIN,   r_state);
        HAL_GPIO_WritePin(LED1_GREEN_GPIO_PORT, LED1_GREEN_GPIO_PIN, g_state);
        HAL_GPIO_WritePin(LED1_BLUE_GPIO_PORT,  LED1_BLUE_GPIO_PIN,  b_state);
    } else { // LED_2
        HAL_GPIO_WritePin(LED2_RED_GPIO_PORT,   LED2_RED_GPIO_PIN,   r_state);
        HAL_GPIO_WritePin(LED2_GREEN_GPIO_PORT, LED2_GREEN_GPIO_PIN, g_state);
        HAL_GPIO_WritePin(LED2_BLUE_GPIO_PORT,  LED2_BLUE_GPIO_PIN,  b_state);
    }
}

void RGB_LED_Init(void)
{
   
    // 初始状态：关闭所有LED
    RGB_LED_SetColor(LED_1, RGB_COLOR_OFF);
    RGB_LED_SetColor(LED_2, RGB_COLOR_OFF);
}

void RGB_LED_SetColor(LED_Number_t led_num, RGB_Color_t color)
{
    // 停止可能正在进行的闪烁
    if (led_num == LED_1) led1_blink_enabled = 0;
    else led2_blink_enabled = 0;
    
    switch(color) {
        case RGB_COLOR_OFF:     _LED_WritePins(led_num, LED_OFF, LED_OFF, LED_OFF);  break;
        case RGB_COLOR_RED:     _LED_WritePins(led_num, LED_ON,  LED_OFF, LED_OFF);  break;
        case RGB_COLOR_GREEN:   _LED_WritePins(led_num, LED_OFF, LED_ON,  LED_OFF);  break;
        case RGB_COLOR_BLUE:    _LED_WritePins(led_num, LED_OFF, LED_OFF, LED_ON);   break;
        case RGB_COLOR_CYAN:    _LED_WritePins(led_num, LED_OFF, LED_ON,  LED_ON);   break;
        case RGB_COLOR_MAGENTA: _LED_WritePins(led_num, LED_ON,  LED_OFF, LED_ON);   break;
        case RGB_COLOR_YELLOW:  _LED_WritePins(led_num, LED_ON,  LED_ON,  LED_OFF);  break;
        case RGB_COLOR_WHITE:   _LED_WritePins(led_num, LED_ON,  LED_ON,  LED_ON);   break;
    }
}

void RGB_LED_Blink(LED_Number_t led_num, RGB_Color_t color, uint32_t interval_ms)
{
    if (led_num == LED_1) {
        led1_blink_enabled = 1;
        led1_blink_color = color;
        led1_blink_interval = interval_ms > 0 ? interval_ms : 1; // 避免间隔为0
        led1_blink_timer = HAL_GetTick();
        led1_blink_state_on = 1; // 开始时先点亮
        switch(color) {
        case RGB_COLOR_OFF:     _LED_WritePins(led_num, LED_OFF, LED_OFF, LED_OFF);  break;
        case RGB_COLOR_RED:     _LED_WritePins(led_num, LED_ON,  LED_OFF, LED_OFF);  break;
        case RGB_COLOR_GREEN:   _LED_WritePins(led_num, LED_OFF, LED_ON,  LED_OFF);  break;
        case RGB_COLOR_BLUE:    _LED_WritePins(led_num, LED_OFF, LED_OFF, LED_ON);   break;
        case RGB_COLOR_CYAN:    _LED_WritePins(led_num, LED_OFF, LED_ON,  LED_ON);   break;
        case RGB_COLOR_MAGENTA: _LED_WritePins(led_num, LED_ON,  LED_OFF, LED_ON);   break;
        case RGB_COLOR_YELLOW:  _LED_WritePins(led_num, LED_ON,  LED_ON,  LED_OFF);  break;
        case RGB_COLOR_WHITE:   _LED_WritePins(led_num, LED_ON,  LED_ON,  LED_ON);   break;
    }
    } else { // LED_2
        led2_blink_enabled = 1;
        led2_blink_color = color;
        led2_blink_interval = interval_ms > 0 ? interval_ms : 1;
        led2_blink_timer = HAL_GetTick();
        led2_blink_state_on = 1; // 开始时先点亮
        switch(color) {
        case RGB_COLOR_OFF:     _LED_WritePins(led_num, LED_OFF, LED_OFF, LED_OFF);  break;
        case RGB_COLOR_RED:     _LED_WritePins(led_num, LED_ON,  LED_OFF, LED_OFF);  break;
        case RGB_COLOR_GREEN:   _LED_WritePins(led_num, LED_OFF, LED_ON,  LED_OFF);  break;
        case RGB_COLOR_BLUE:    _LED_WritePins(led_num, LED_OFF, LED_OFF, LED_ON);   break;
        case RGB_COLOR_CYAN:    _LED_WritePins(led_num, LED_OFF, LED_ON,  LED_ON);   break;
        case RGB_COLOR_MAGENTA: _LED_WritePins(led_num, LED_ON,  LED_OFF, LED_ON);   break;
        case RGB_COLOR_YELLOW:  _LED_WritePins(led_num, LED_ON,  LED_ON,  LED_OFF);  break;
        case RGB_COLOR_WHITE:   _LED_WritePins(led_num, LED_ON,  LED_ON,  LED_ON);   break;
    }
    }
}

void RGB_LED_StopBlink(LED_Number_t led_num)
{
    if (led_num == LED_1) {
        led1_blink_enabled = 0;
        RGB_LED_SetColor(LED_1, RGB_COLOR_OFF);
    } else { // LED_2
        led2_blink_enabled = 0;
        RGB_LED_SetColor(LED_2, RGB_COLOR_OFF);
    }
}

void RGB_LED_ProcessBlink(void)
{
    uint32_t current_time = HAL_GetTick();

    // 处理LED1闪烁
    if (led1_blink_enabled && (current_time - led1_blink_timer >= led1_blink_interval)) {
        led1_blink_timer = current_time;
        led1_blink_state_on = !led1_blink_state_on; // 翻转状态
        if (led1_blink_state_on) {
            _LED_WritePins(LED_1, (led1_blink_color==RGB_COLOR_RED||led1_blink_color==RGB_COLOR_MAGENTA||led1_blink_color==RGB_COLOR_YELLOW||led1_blink_color==RGB_COLOR_WHITE)?LED_ON:LED_OFF, (led1_blink_color==RGB_COLOR_GREEN||led1_blink_color==RGB_COLOR_CYAN||led1_blink_color==RGB_COLOR_YELLOW||led1_blink_color==RGB_COLOR_WHITE)?LED_ON:LED_OFF, (led1_blink_color==RGB_COLOR_BLUE||led1_blink_color==RGB_COLOR_CYAN||led1_blink_color==RGB_COLOR_MAGENTA||led1_blink_color==RGB_COLOR_WHITE)?LED_ON:LED_OFF);
        } else {
            _LED_WritePins(LED_1, LED_OFF, LED_OFF, LED_OFF);
        }
    }
    
    // 处理LED2闪烁
    if (led2_blink_enabled && (current_time - led2_blink_timer >= led2_blink_interval)) {
        led2_blink_timer = current_time;
        led2_blink_state_on = !led2_blink_state_on; // 翻转状态
        if (led2_blink_state_on) {
            _LED_WritePins(LED_2, (led2_blink_color==RGB_COLOR_RED||led2_blink_color==RGB_COLOR_MAGENTA||led2_blink_color==RGB_COLOR_YELLOW||led2_blink_color==RGB_COLOR_WHITE)?LED_ON:LED_OFF, (led2_blink_color==RGB_COLOR_GREEN||led2_blink_color==RGB_COLOR_CYAN||led2_blink_color==RGB_COLOR_YELLOW||led2_blink_color==RGB_COLOR_WHITE)?LED_ON:LED_OFF, (led2_blink_color==RGB_COLOR_BLUE||led2_blink_color==RGB_COLOR_CYAN||led2_blink_color==RGB_COLOR_MAGENTA||led2_blink_color==RGB_COLOR_WHITE)?LED_ON:LED_OFF);
        } else {
            _LED_WritePins(LED_2, LED_OFF, LED_OFF, LED_OFF);
        }
    }
}