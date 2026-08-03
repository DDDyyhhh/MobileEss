#ifndef __BMS_COMM_H
#define __BMS_COMM_H

#include <stdint.h>

// 根据 AXEBMS_Data 结构体创建C语言版本
typedef struct {
    float    Voltage;       // 电芯总电压(V)
    float    Current;       // 实时电流(A)
    float    Temp1;         // 温度1(摄氏度)
    float    Temp2;         // 温度2(摄氏度)
    float    Temp3;         // 温度3(摄氏度)
    float    Temp4;         // 温度4(摄氏度)
    float    Temp5;         // 温度5(摄氏度)
    float    Temp6;         // 温度6(摄氏度)
    float    Tmax;          // 最高温度(摄氏度)
    float    Tmin;          // 最低温度(摄氏度)
    uint16_t Vmax;          // 单体最高电压(mV)
    uint16_t Vmin;          // 单体最低电压(mV)
    uint16_t VmaxminNo;     // 单体最高最低电压对应的串数
    uint16_t RSOC;          // 电池包剩余电量百分比(%)
    float    FCC;           // 系统满充容量(AH)
    float    RC;            // 电池包当前剩余容量(AH)
} BatteryData_t;

// 函数声明
void BMS_Init(void);
void BMS_RequestData(uint8_t slave_id);
uint8_t BMS_ProcessData(void); // 返回1表示成功接收并解析，0表示没有新数据或数据错误
const BatteryData_t* BMS_GetData(void);

#endif /* __BMS_COMM_H */