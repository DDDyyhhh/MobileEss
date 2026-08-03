/*
 * 描述: AXEBMS 安轩能量电池
 * 版本: 1.v1.0 lgz 2024/5/17 BASE
 * Copyright (C) 2024, ligz, all right reserved.
 */

#ifndef __AXEBMS_h
#define __AXEBMS_h

#include "util/Type.h"
#include "util/Std.h"
#include "base/BusDeviceBase.h"
#include "protocol/ModbusComm.h"
#include "Job.h"

// 解包后的数据结构
struct AXEBMS_Data {
    u16r2 Voltage;  // 电芯总电压(0.01V)
    s16r1 CurCadc;  // 实时电流(0.1A)
    u16r1 Temp1;    // 温度1(0.1摄氏度)
    u16r1 Temp2;    // 温度2(0.1摄氏度)
    u16r1 Temp3;    // 温度3(0.1摄氏度)
    u16r1 Temp4;    // 温度4(0.1摄氏度)
    u16r1 Temp5;    // 温度5(0.1摄氏度)
    u16r1 Temp6;    // 温度6(0.1摄氏度)
    u16r1 Tmax;     // 最高温度(0.1摄氏度)
    u16r1 Tmin;     // 最低温度(0.1摄氏度)
    u16 Vmax;       // 单体最高电压(1mV)
    u16 Vmin;       // 单体最低电压(1mV)
    u16 VmaxminNo;  // 单体最高最低电压对应的串数
    u16 RSOC;       // 电池包剩余电量百分比(1%)
    u16r2 FCC;      // 系统满充容量(0.01AH)
    u16r2 RC;       // 电池包当前剩余容量(0.01AH)
};

#define AXEBMS_RxDataEvent(sign)                (AXEBMS::RX_DATA|(sign))
#define AXEBMS_VoltageEvent(sign)               (AXEBMS::VOLTAGE|(sign))

// ***总线设备***: 需要挂靠在某个总线上的设备
class AXEBMS : private BusDeviceBase, private Job { 
public:
    AXEBMS(ModbusRtuBus &rtu, u8 devAddr, EventSign evtSign);

    // 事件定义
    enum { 
        RX_DATA = 0x0000,       // 收到数据
        VOLTAGE = 0x0100,       // 电压报警
        COMM = 0x0200,          // 通讯报警
    };

    void turn(Std::Turn on);    // (信息接收)模块启动时 默认开启

    void configWarmingVoltage(u16r2 vol, u16r1 s);      // 0电压表示不启动报警
    void configProtectVoltage(u16r2 vol, u16r1 s);      // 0电压表示不启动报警

public:    
    AXEBMS_Data& read() { 
        return d_; 
    }
    s8 getState() {
        return state_;
    }
    bool isLowVoltage() {
        return state_ == -1;
    }
    bool isLossVoltage() {
        return state_ == -2;
    }
    
    // 事件
    Event RxDataEvent() {
        return AXEBMS_RxDataEvent(evtSign_);
    }
    Event VoltageEvent() {
        return AXEBMS_VoltageEvent(evtSign_);
    }
    Event CommEvent() {
        return AXEBMS_VoltageEvent(evtSign_);
    }
    
private:
    virtual int run();
    virtual Event todo(bool kData);
    void setSendFrame(u8 cmd, u16 start, u16 num);

private:
    ModbusRtuBus &rtu_;

    u8 nodeSn_;
    u8 devAddr_;
    EventSign evtSign_;

    u16r2 warmingVoltage_;
    u16r2 protectVoltage_;

    u32 warmingVoltageDelayMs_;         // ms 为单位
    u32 protectVoltageDelayMs_;         // ms 为单位

    // 电压计时
    u32 warmingVoltageTimer_;
    u32 protectVoltageTimer_;  

    // 电压计数
    u16 warmingVoltageCount_;
    u16 protectVoltageCount_;

    bool kVoltageWarming_;
    bool kVoltageProtected_;

    s8 state_;                          // 状态: 0=正常, -1=低压, -2=缺压, -3=通讯故障, -4=电池故障
    s8 validCnt_;                       // 有效数据 和 无效计数

    u32 normalVoltageCount_;

    struct SendFrame {
        u8 addr_;
        u8 cmd_;
        Be16 startAddr_;
        Be16 num_;
    } sendFrame_;
    
    AXEBMS_Data d_;
};

#endif
