// Core/Src/flash_storage.c

#include "flash_storage.h"
#include "main.h" // 包含 STM32 HAL 库

// 选择一个Flash扇区来存储数据。
// STM32F407VET6的Flash地址从 0x08000000 开始。
// 我们选择最后一个扇区，Sector 11 (128KB)，地址为 0x080E0000
// 这样可以避免与程序代码冲突。
#define SETTINGS_FLASH_SECTOR     FLASH_SECTOR_11
#define SETTINGS_FLASH_ADDRESS    0x080E0000

// 定义要存储的数据的实例
static AppSettings_t g_app_settings;

/**
 * @brief 从Flash加载设置
 */
bool FlashStorage_LoadSettings(AppSettings_t *settings)
{
    if (settings == NULL) {
        return false;
    }

    // 直接从Flash地址读取数据
    memcpy(settings, (void*)SETTINGS_FLASH_ADDRESS, sizeof(AppSettings_t));
    
    // 检查魔法数字是否匹配，以判断数据是否有效
    if (settings->magic_number == APP_SETTINGS_MAGIC_NUMBER) {
        return true; // 数据有效
    }
    
    return false; // 数据无效或未初始化
}

/**
 * @brief 将设置保存到Flash
 */
bool FlashStorage_SaveSettings(const AppSettings_t *settings)
{
    if (settings == NULL) {
        return false;
    }
    
    HAL_StatusTypeDef status;

    // 1. 解锁Flash
    HAL_FLASH_Unlock();
    
    // 2. 擦除扇区
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase_init.Sector = SETTINGS_FLASH_SECTOR;
    erase_init.NbSectors = 1;
    
    uint32_t sector_error = 0;
    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return false; // 擦除失败
    }

    // 3. 逐个字 (32-bit) 写入数据
    // Flash编程需要按字、半字或字节进行
    uint32_t* data_ptr = (uint32_t*)settings;
    uint32_t address = SETTINGS_FLASH_ADDRESS;
    uint32_t data_size_in_words = (sizeof(AppSettings_t) + 3) / 4; // 向上取整

    for (uint32_t i = 0; i < data_size_in_words; ++i)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data_ptr[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false; // 写入失败
        }
        address += 4;
    }
    
    // 4. 锁定Flash
    HAL_FLASH_Lock();
    
    return true;
}