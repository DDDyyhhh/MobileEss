#ifndef __AXEBMS_RS485_H
#define __AXEBMS_RS485_H

#include "rs485.h"
#include <stdint.h>
#include <stdbool.h>

// 电池数据结构
typedef struct {
    uint32_t Voltage;      // 电芯总电压(0.01V)
    int16_t CurCadc;       // 实时电流(0.1A)
    uint16_t Temp1;        // 温度1(0.1摄氏度)
    uint16_t Temp2;        // 温度2(0.1摄氏度)
    uint16_t Temp3;        // 温度3(0.1摄氏度)
    uint16_t Temp4;        // 温度4(0.1摄氏度)
    uint16_t Temp5;        // 温度5(0.1摄氏度)
    uint16_t Temp6;        // 温度6(0.1摄氏度)
    uint16_t Tmax;         // 最高温度(0.1摄氏度)
    uint16_t Tmin;         // 最低温度(0.1摄氏度)
    uint16_t Vmax;         // 单体最高电压(1mV)
    uint16_t Vmin;         // 单体最低电压(1mV)
    uint16_t VmaxminNo;    // 单体最高最低电压对应的串数
    uint16_t RSOC;         // 电池包剩余电量百分比(1%)
    uint32_t FCC;          // 系统满充容量(0.01AH)
    uint32_t RC;           // 电池包当前剩余容量(0.01AH)
} AXEBMS_Data;

// 电池管理类
typedef struct {
    uint8_t devAddr;
    
    // 电压保护配置
    uint32_t warmingVoltage;
    uint32_t protectVoltage;
    uint32_t warmingVoltageDelayMs;
    uint32_t protectVoltageDelayMs;
    
    // 状态管理
    int8_t state;
    int8_t validCnt;
    
    // 计时器
    uint32_t warmingVoltageTimer;
    uint32_t protectVoltageTimer;
    uint16_t warmingVoltageCount;
    uint16_t protectVoltageCount;
    
    bool kVoltageWarming;
    bool kVoltageProtected;
    
    // 数据
    AXEBMS_Data data;
    
    // 通信缓冲区
    uint8_t txBuffer[8];
    uint8_t rxBuffer[64];
    uint16_t rxDataLength;
    
} AXEBMS_RS485;

// 函数声明
void AXEBMS_RS485_Init(AXEBMS_RS485* bms, uint8_t devAddr);
void AXEBMS_RS485_Start(AXEBMS_RS485* bms);
void AXEBMS_RS485_Stop(AXEBMS_RS485* bms);
void AXEBMS_RS485_ConfigWarmingVoltage(AXEBMS_RS485* bms, uint32_t vol, uint16_t s);
void AXEBMS_RS485_ConfigProtectVoltage(AXEBMS_RS485* bms, uint32_t vol, uint16_t s);
void AXEBMS_RS485_Process(AXEBMS_RS485* bms);
AXEBMS_Data* AXEBMS_RS485_GetData(AXEBMS_RS485* bms);
int8_t AXEBMS_RS485_GetState(AXEBMS_RS485* bms);
bool AXEBMS_RS485_IsLowVoltage(AXEBMS_RS485* bms);
bool AXEBMS_RS485_IsLossVoltage(AXEBMS_RS485* bms);

#endif

