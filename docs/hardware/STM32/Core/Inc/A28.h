/**
  ******************************************************************************
  * @file           : A28.h
  * @brief          : 泽耀 A28-T2G4A12D1a (SX1281) 无线模块驱动头文件
  ******************************************************************************
  */
#ifndef __A28_H
#define __A28_H

#include "usart.h"
#include <stdint.h>

// ==========================================
// A28 寄存器参数宏定义 (参考手册 6.2)
// ==========================================
// SPEED 寄存器: 波特率 (UART BAUD[2:0])
#define A28_BAUD_9600      (0x03 << 3)
#define A28_BAUD_38400     (0x05 << 3)
#define A28_BAUD_115200    (0x07 << 3) // 推荐用于遥控器

// SPEED 寄存器: 空速 (AIR_SPEED[2:0])
#define A28_AIR_SPEED_0    (0x00) // 默认低空速，距离远
#define A28_AIR_SPEED_1    (0x01)
#define A28_AIR_SPEED_2    (0x02)
#define A28_AIR_SPEED_3    (0x03)
#define A28_AIR_SPEED_7    (0x07) // 最高空速，延迟极低

// OPTION 寄存器: 调制方式 (Bit 7)
#define A28_MOD_FLRC       (0x00 << 7) // 极低延迟模式 (推荐用于割草机运动控制)
#define A28_MOD_LORA       (0x01 << 7) // 超远距离模式 (延迟稍大)

// OPTION 寄存器: 发射功率 (Bit 1:0)
#define A28_POWER_3DBM     (0x00)
#define A28_POWER_6DBM     (0x01)
#define A28_POWER_9DBM     (0x02)
#define A28_POWER_12DBM    (0x03) // 默认 15mW (最高功率)

// ==========================================
// 函数声明
// ==========================================
void A28_Mode_GPIO_Init(void);

// 模式切换
void A28_Set_Config_Mode(void);
void A28_Set_HalfDuplex_Mode(void);
void A28_Set_FullDuplex_Mode(void);

// 参数配置 (影子寄存器机制)
void A28_Apply_Config(void);
void A28_Set_Address(uint16_t addr);
void A28_Set_Channel(uint8_t channel);
void A28_Set_Power_And_Modulation(uint8_t power, uint8_t modulation);

// 信号强度读取
int16_t A28_Read_Current_RSSI(void);
int16_t A28_Read_Environment_RSSI(void);


void A28_Apply_Config(void);
#endif

