/**
  ******************************************************************************
  * @file           : E34.c
  * @brief          : E34 (LoRa/2.4G) 无线透传模块的硬件配置与模式切换驱动
  * 
  * 主要功能：
  * 1. 初始化控制无线模块工作模式的 M0 (PE0) / M1 (PE1) 引脚
  * 2. 提供了切换全双工工作模式、普通透传模式与休眠配置模式的接口
  * 3. 实现了动态修改模块地址、信道及发射功率的 AT/Hex 指令交互 (`E34_Set_Address`)
  *
  * 注意事项：
  * - 根据厂商手册，切换 M0/M1 状态或发送配置命令后，模块需要几十毫秒的内部处理时间，
  *   因此函数中使用了 `osDelay(50)` 避免时序崩塌。
  * - 配置指令使用了阻塞式串口发送 (`HAL_MAX_DELAY`)，只能在系统初始化阶段或安全状态下调用。
  ******************************************************************************
  */

#include "usart.h"
#include "cmsis_os.h"
// 初始化 M0/M1 引脚（PE0/PE1）
void E34_Mode_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}
void E34_Set_FullDuplex_Mode(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);   // M0=1
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); // M1=0
}

void E34_Set_Config_Mode(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET); // M0=1
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET); // M1=1
}
// 设置模块为普通透传模式
void E34_Set_Normal_Mode(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); // M0=0
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); // M1=0
}

// 设置E34模块地址（addr: 0x0000~0xFFFF）
void E34_Set_Address(uint16_t addr)
{
    uint8_t cfg_cmd[6];

    // 进入配置模式
    E34_Set_Config_Mode();
    // 构造写参数命令（以 E34-2G4H11S 通用格式为例，具体格式请参考数据手册）
    cfg_cmd[0] = 0xC0;               // 写配置命令
    cfg_cmd[1] = (addr >> 8) & 0xFF; // 高8位地址
    cfg_cmd[2] = addr & 0xFF;        // 低8位地址
    cfg_cmd[3] = 0x18;               // 通道（默认23，0x17，可根据需要修改）
    cfg_cmd[4] = 0x00;               // 信道带宽（默认250kHz，0x18）
    cfg_cmd[5] = 0x40;               // 功率

    osDelay(50);

    // 发送配置命令

    HAL_UART_Transmit(&huart3, cfg_cmd, 6, HAL_MAX_DELAY);
    osDelay(50);
    // 读取配置以确认
    uint8_t read_cmd[3] = {0xC1, 0xC1, 0xC1};

    HAL_UART_Transmit(&huart3, read_cmd, 3, HAL_MAX_DELAY);
    osDelay(50);
    E34_Set_FullDuplex_Mode(); // 切换回工作模式
}