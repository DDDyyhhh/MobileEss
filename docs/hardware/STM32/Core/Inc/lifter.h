// #ifndef __LIFTER_H
// #define __LIFTER_H

// #include "main.h"

// // 状态机枚举
// typedef enum {
//     LIFTER_STATE_IDLE,           // 空闲/停止
//     LIFTER_STATE_STEP_MOVE,      // 步进移动 (保留，用于手动微调)
//     LIFTER_STATE_HOMING_START,   // 开始归位
//     LIFTER_STATE_HOMING_RUNNING, // 归位运动中
//     LIFTER_STATE_HOMING_DONE,    // 归位完成
//     LIFTER_STATE_AUTO_ADJUST     // 【新增】自动调节位置模式
// } Lifter_MachineState;

// // 函数声明
// void Lifter_Init(void);
// void Lifter_Task_Update(void); // 在任务中周期调用
// void Lifter_Stop(void);

// // 手动控制接口 (保留)
// void Lifter_StepUp(void);
// void Lifter_StepDown(void);

// // 【新增】位置控制接口
// void Lifter_SetTargetHeight(float height_cm);

// // 数据获取
// Lifter_MachineState Lifter_GetState(void);
// float Lifter_GetCurrentHeight_cm(void);
// void Lifter_SaveHeightToFlash(void);


// void Lifter_Test_Down_Time(uint32_t ms_duration);
// void Lifter_Test_Up_Time(uint32_t ms_duration);
// #endif


#ifndef __LIFTER_H
#define __LIFTER_H

#include "main.h"

// 状态机枚举
typedef enum {
    LIFTER_STATE_IDLE,
    LIFTER_STATE_HOMING_START,   // 开始归位
    LIFTER_STATE_HOMING_RUNNING, // 归位中
    LIFTER_STATE_HOMING_DONE,    // 归位完成
    LIFTER_STATE_AUTO_ADJUST     // 自动调节
} Lifter_MachineState;

// 函数声明
void Lifter_Init(void);
void Lifter_Task_Update(void); // 周期调用 (建议50ms)
void Lifter_Stop(void);

// 设置目标档位 (1-5)
void Lifter_SetTargetLevel(uint8_t level);

// 获取当前高度 (cm) - 用于发送给遥控器
float Lifter_GetCurrentHeight_cm(void);

// 获取当前状态
Lifter_MachineState Lifter_GetState(void);

//用户自己初始化高度到最高
void Lifter_User_Init(void);


// 设置草袋目标状态：1=打开，0=关闭
void Lifter_SetGrassBagTarget(uint8_t open);
void Lifter_ToggleGrassBag(void); // 触发草袋开合
#endif
