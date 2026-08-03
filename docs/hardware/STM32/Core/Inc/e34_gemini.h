/*
 * e34.h
 *
 *  Created on: [Your Date]
 *      Author: [Your Name]
 *
 *  Modular driver for Ebyte E34-2G4Dxxx series wireless modules.
 */

#ifndef INC_E34_GEMINI_H_
#define INC_E34_GEMINI_H_

#include "main.h" // 根据你的MCU型号修改 (例如 stm32f1xx_hal.h)

/*==================================================================================*/
/*                            1. USER CONFIGURATION                                 */
/*==================================================================================*/

// --- 配置与E34模块连接的UART ---
extern UART_HandleTypeDef huart3; // <<<<<<< 修改成你的UART Handle
#define E34_HUART         (&huart3)

// --- 配置 M0 和 M1 控制引脚 ---
#define E34_M0_PORT       GPIOE     // <<<<<<< 修改成 M0 连接的 GPIO Port
#define E34_M0_PIN        GPIO_PIN_0  // <<<<<<< 修改成 M0 连接的 GPIO Pin

#define E34_M1_PORT       GPIOE     // <<<<<<< 修改成 M1 连接的 GPIO Port
#define E34_M1_PIN        GPIO_PIN_1  // <<<<<<< 修改成 M1 连接的 GPIO Pin

// --- 配置工作时的通信波特率 (必须与CubeMX中设置的UART波特率一致) ---
#define E34_WORKING_BAUDRATE      9600    // <<<<<<< 修改成你正常通信时用的波特率

/*==================================================================================*/
/*                           2. DRIVER DEFINITIONS                                  */
/*==================================================================================*/

// E34模块工作模式枚举
typedef enum {
    MODE_NORMAL = 0,        // 正常透传模式 (M0=0, M1=0)
    MODE_FULL_DUPLEX = 1,   // 全双工模式 (M0=1, M1=0)
    MODE_RESERVED = 2,      // 保留模式 (M0=0, M1=1)
    MODE_CONFIG = 3         // 配置/休眠模式 (M0=1, M1=1)
} E34_Mode_t;

// E34模块配置参数结构体
typedef struct {
    uint16_t address;       // 模块地址 (0x0000 - 0xFFFF)
    uint8_t  channel;       // 通信信道 (0x00 - 0x1F, 对应 2400 + CHAN*1 MHz)
    uint8_t  sped_byte;     // 速率参数字节
    uint8_t  option_byte;   // 选项参数字节
} E34_Config_t;

/*==================================================================================*/
/*                             3. PUBLIC FUNCTIONS                                  */
/*==================================================================================*/

/**
 * @brief 初始化E34模块驱动 (初始化控制引脚并设置为正常模式)
 * @retval HAL_StatusTypeDef HAL_OK or HAL_ERROR
 */
HAL_StatusTypeDef E34_Init(void);

/**
 * @brief 设置E34模块的工作模式
 * @param mode 要设置的模式 (E34_Mode_t)
 * @retval HAL_StatusTypeDef HAL_OK or HAL_ERROR
 */
HAL_StatusTypeDef E34_SetMode(E34_Mode_t mode);

/**
 * @brief 从模块读取当前配置参数
 * @param config 指向 E34_Config_t 结构体的指针，用于存放读取到的配置
 * @retval HAL_StatusTypeDef HAL_OK or HAL_ERROR
 */
HAL_StatusTypeDef E34_GetConfig(E34_Config_t* config);

/**
 * @brief 将新的配置参数写入模块并掉电保存
 * @param config 指向包含新配置的 E34_Config_t 结构体的指针
 * @retval HAL_StatusTypeDef HAL_OK on success, HAL_ERROR on failure
 */
HAL_StatusTypeDef E34_SetConfig(const E34_Config_t* config);

/**
 * @brief 发送软件复位指令使模块重启
 * @retval HAL_StatusTypeDef HAL_OK or HAL_ERROR
 */
HAL_StatusTypeDef E34_ResetModule(void);

/**
 * @brief 通过UART发送数据到E34模块
 * @param data 指向要发送数据的指针
 * @param len  要发送的数据长度
 * @param timeout 超时时间 (ms)
 * @retval HAL_StatusTypeDef HAL_OK or HAL_ERROR
 */
HAL_StatusTypeDef E34_SendData(uint8_t* data, uint16_t len, uint32_t timeout);

#endif /* INC_E34_H_ */