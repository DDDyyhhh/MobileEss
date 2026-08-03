#ifndef RGB_LED_H
#define RGB_LED_H

#include "stm32f4xx_hal.h" // 请确保这里的头文件与你的工程匹配

// LED 编号枚举
typedef enum {
    LED_1 = 0,
    LED_2 = 1
} LED_Number_t;

// RGB 颜色枚举
typedef enum {
    RGB_COLOR_OFF = 0,    // 关闭
    RGB_COLOR_RED,        // 红色
    RGB_COLOR_GREEN,      // 绿色
    RGB_COLOR_BLUE,       // 蓝色
    RGB_COLOR_CYAN,       // 青色 (绿 + 蓝)
    RGB_COLOR_MAGENTA,    // 品红/洋红 (红 + 蓝)
    RGB_COLOR_YELLOW,     // 黄色 (红 + 绿)
    RGB_COLOR_WHITE       // 白色 (红 + 绿 + 蓝)
} RGB_Color_t;

/**
 * @brief 初始化两路RGB LED所使用的GPIO引脚
 */
void RGB_LED_Init(void);

/**
 * @brief 设置指定LED的颜色
 * @param led_num LED编号 (LED_1 或 LED_2)
 * @param color 要设置的颜色，来自 RGB_Color_t 枚举
 */
void RGB_LED_SetColor(LED_Number_t led_num, RGB_Color_t color);

/**
 * @brief 启动指定LED以特定颜色和频率闪烁
 * @param led_num LED编号 (LED_1 或 LED_2)
 * @param color 闪烁的颜色
 * @param interval_ms 闪烁间隔（亮灭切换时间），单位：毫秒
 */
void RGB_LED_Blink(LED_Number_t led_num, RGB_Color_t color, uint32_t interval_ms);

/**
 * @brief 停止指定LED的闪烁并熄灭它
 * @param led_num LED编号 (LED_1 或 LED_2)
 */
void RGB_LED_StopBlink(LED_Number_t led_num);

/**
 * @brief LED闪烁处理函数，必须在主循环(while(1))中周期性调用
 */
void RGB_LED_ProcessBlink(void);

#endif // RGB_LED_H