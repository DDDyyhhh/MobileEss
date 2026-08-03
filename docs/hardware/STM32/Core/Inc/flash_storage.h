// Core/Inc/flash_storage.h

#ifndef __FLASH_STORAGE_H
#define __FLASH_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// 定义一个结构体来存储所有需要掉电保存的参数
typedef struct
{
    float lifter_current_height_cm; // 刀盘当前高度
    uint32_t magic_number;          // 魔法数字，用于验证数据是否有效
    // 未来可以添加更多参数，如累计工作时间等
} AppSettings_t;

// 定义魔法数字，用于检查Flash中是否有有效数据
#define APP_SETTINGS_MAGIC_NUMBER 0xDEADBEEF

// 初始化Flash存储（实际上不需要做什么）
void FlashStorage_Init(void);

// 从Flash加载设置
// @return: 如果成功加载有效数据则返回 true, 否则返回 false
bool FlashStorage_LoadSettings(AppSettings_t *settings);

// 将设置保存到Flash
// @return: 如果成功保存则返回 true, 否则返回 false
bool FlashStorage_SaveSettings(const AppSettings_t *settings);

#endif // __FLASH_STORAGE_H
