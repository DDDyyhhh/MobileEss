// buzzer.h

#ifndef BUZZER_H
#define BUZZER_H

#include "main.h" // 包含 main.h 以获取HAL库和定时器句柄

// 定义蜂鸣器鸣叫模式
typedef enum {
    BUZZER_MODE_OFF,      // 关闭
    BUZZER_MODE_ON,       // 持续响
    BUZZER_MODE_SHORT,    // 短响一声 (滴)
    BUZZER_MODE_DOUBLE,   // 响两声 (滴滴)
    BUZZER_MODE_LONG,     // 长响一声 (嘀---)
    BUZZER_MODE_ALARM     // 警报声 (滴-滴-滴)
} BuzzerMode_t;

/**
 * @brief 初始化蜂鸣器
 * @note  此函数会启动PWM通道（如果使用无源蜂鸣器）
 */
void Buzzer_Init(void);

/**
 * @brief 设置蜂鸣器的工作模式
 * @param mode 要设置的模式，来自 BuzzerMode_t 枚举
 */
void Buzzer_SetMode(BuzzerMode_t mode);

/**
 * @brief 蜂鸣器处理函数，必须在主循环或定时任务中周期性调用
 * @note  此函数用于处理非持续性的鸣叫模式，如短响、双响等
 */
void Buzzer_Process(void);

#endif // BUZZER_H