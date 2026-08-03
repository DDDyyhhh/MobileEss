// // #ifndef __SR04_H
// // #define __SR04_H

// // #include "tim.h"

// // //#define DWT_DELAY
// // //#define CYCLE_DELAY

// // //#define IO_MODE
// // //#define IC_DELAY

// // /* 定义所用定时器和通道 */
// // #define SR04_TIM        TIM1
// // #define SR04_TIMER      &htim1
// // #define SR04_CHANNEL    TIM_CHANNEL_1

// // /* Trig 引脚配置 */
// // #ifndef Trig_Pin 
// // #define Trig_Pin        GPIO_PIN_8
// // #define Trig_GPIO_Port  GPIOE
// // #endif

// // /* Echo 引脚配置 */
// // #ifndef Echo_Pin 
// // #define Echo_Pin        GPIO_PIN_9
// // #define Echo_GPIO_Port  GPIOE
// // #endif

// // /**
// //  * @brief 超声波数据结构体
// //  */
// // typedef struct {
// //     uint8_t status;        ///< 捕获状态：0=等待上升沿，1=等待下降沿
// //     uint16_t IC_Time[2];   ///< 捕获的时间戳（上升沿/下降沿）
// //     float distance;        ///< 测量得到的距离（cm）
// // } SR04_Data;

// // /**
// //  * @brief 获取测距结果
// //  * @retval float 距离值（单位：cm）
// //  */
// // float Get_SR04_Distance(void);

// // /**
// //  * @brief 更新测距数据（触发一次测量）
// //  */
// // void SR04_Update(void);



// // #endif


// //测量4个超声波传感器的距离
// // #ifndef __SR04_H
// // #define __SR04_H

// // #include "tim.h"
// // #include "main.h" // 包含main.h以确保GPIO定义被引入

// // // #define DWT_DELAY 
// // // 如果使用DWT延时，请确保在main中初始化了DWT

// // /* 定时器定义 (4个传感器共用 TIM1) */
// // #define SR04_TIM_INSTANCE   TIM1
// // #define SR04_TIMER_HANDLE   &htim1

// // /* --- 传感器 1 定义 --- */
// // #define SR04_1_TRIG_PIN     GPIO_PIN_8
// // #define SR04_1_TRIG_PORT    GPIOE
// // #define SR04_1_ECHO_PIN     GPIO_PIN_9
// // #define SR04_1_ECHO_PORT    GPIOE
// // #define SR04_1_CHANNEL      TIM_CHANNEL_1

// // /* --- 传感器 2 定义 --- */
// // #define SR04_2_TRIG_PIN     GPIO_PIN_10
// // #define SR04_2_TRIG_PORT    GPIOE
// // #define SR04_2_ECHO_PIN     GPIO_PIN_11
// // #define SR04_2_ECHO_PORT    GPIOE
// // #define SR04_2_CHANNEL      TIM_CHANNEL_2

// // /* --- 传感器 3 定义 (TIM1_CH4) --- */
// // #define SR04_3_TRIG_PIN     GPIO_PIN_15
// // #define SR04_3_TRIG_PORT    GPIOE
// // #define SR04_3_ECHO_PIN     GPIO_PIN_14
// // #define SR04_3_ECHO_PORT    GPIOE
// // #define SR04_3_CHANNEL      TIM_CHANNEL_4

// // /* --- 传感器 4 定义 (TIM1_CH3) --- */
// // #define SR04_4_TRIG_PIN     GPIO_PIN_12
// // #define SR04_4_TRIG_PORT    GPIOE
// // #define SR04_4_ECHO_PIN     GPIO_PIN_13
// // #define SR04_4_ECHO_PORT    GPIOE
// // #define SR04_4_CHANNEL      TIM_CHANNEL_3

// // /**
// //  * @brief 超声波数据结构体
// //  */
// // typedef struct {
// //     uint8_t status;        ///< 捕获状态：0=等待上升沿，1=等待下降沿
// //     uint16_t IC_Time[2];   ///< 捕获的时间戳（上升沿/下降沿）
// //     float distance;        ///< 测量得到的距离（cm）
// // } SR04_Data_t;

// // /* --- 触发函数 (分别触发) --- */
// // void SR04_Update_1(void);
// // void SR04_Update_2(void);
// // void SR04_Update_3(void);
// // void SR04_Update_4(void);

// // /* --- 统一更新所有 (可选) --- */
// // void SR04_Update_All(void); // 依次触发所有，中间有延时

// // /* --- 获取距离函数 (分别获取) --- */
// // float Get_SR04_Distance_1(void);
// // float Get_SR04_Distance_2(void);
// // float Get_SR04_Distance_3(void);
// // float Get_SR04_Distance_4(void);

// // /* --- 通用获取函数 (通过索引 0-3) --- */
// // float Ultrasonic_GetDistance_cm(uint8_t sensor_index);

// // #endif


// // // 优化后的超声波测距模块头文件
// #ifndef __SR04_H
// #define __SR04_H

// #include "tim.h"
// #include "main.h" // 包含main.h以确保GPIO定义被引入

// // ==========================================================
// //                  配置参数 (可根据需要调整)
// // ==========================================================
// #define SR04_NUM_SENSORS 4          // 传感器数量
// #define SR04_TIMEOUT_MS 50          // 测量超时时间 (ms)
// #define SR04_FILTER_SIZE 5          // 中值滤波窗口大小 (推荐3或5)
// #define SR04_ERROR_THRESHOLD 5      // 连续错误阈值
// #define SR04_MIN_VALID_DIST 2       // 最小有效距离 (cm)
// #define SR04_MAX_VALID_DIST 400     // 最大有效距离 (cm)

// // ==========================================================
// //                  硬件定义 (保持你原有的定义)
// // ==========================================================
// /* 定时器定义 (4个传感器共用 TIM1) */
// #define SR04_TIM_INSTANCE   TIM1
// #define SR04_TIMER_HANDLE   &htim1

// /* 传感器 1 定义 (TIM1_CH1) */
// #define SR04_1_TRIG_PIN     GPIO_PIN_8
// #define SR04_1_TRIG_PORT    GPIOE
// #define SR04_1_ECHO_PIN     GPIO_PIN_9
// #define SR04_1_ECHO_PORT    GPIOE
// #define SR04_1_CHANNEL      TIM_CHANNEL_1

// /* 传感器 2 定义 (TIM1_CH2) */
// #define SR04_2_TRIG_PIN     GPIO_PIN_10
// #define SR04_2_TRIG_PORT    GPIOE
// #define SR04_2_ECHO_PIN     GPIO_PIN_11
// #define SR04_2_ECHO_PORT    GPIOE
// #define SR04_2_CHANNEL      TIM_CHANNEL_2

// /* 传感器 3 定义 (TIM1_CH4) */
// #define SR04_3_TRIG_PIN     GPIO_PIN_15
// #define SR04_3_TRIG_PORT    GPIOE
// #define SR04_3_ECHO_PIN     GPIO_PIN_14
// #define SR04_3_ECHO_PORT    GPIOE
// #define SR04_3_CHANNEL      TIM_CHANNEL_4

// /* 传感器 4 定义 (TIM1_CH3) */
// #define SR04_4_TRIG_PIN     GPIO_PIN_12
// #define SR04_4_TRIG_PORT    GPIOE
// #define SR04_4_ECHO_PIN     GPIO_PIN_13
// #define SR04_4_ECHO_PORT    GPIOE
// #define SR04_4_CHANNEL      TIM_CHANNEL_3

// // ==========================================================
// //                  数据结构
// // ==========================================================
// /**
//  * @brief 优化的超声波数据结构体
//  */
// typedef struct {
//     volatile uint8_t status;       ///< 捕获状态：0=等待上升沿，1=等待下降沿，2=完成/空闲
//     volatile uint16_t IC_Time[2];  ///< 捕获的时间戳（上升沿/下降沿）
//     volatile float distance;       ///< 最终输出的滤波后距离（cm）
    
//     // --- 新增滤波与状态维护 ---
//     float filter_buffer[SR04_FILTER_SIZE]; ///< 滤波缓冲区
//     uint8_t filter_index;                  ///< 滤波缓冲区当前索引
//     uint32_t last_valid_time;              ///< 最后一次有效测量的时间戳
//     uint8_t error_count;                   ///< 连续错误计数
// } SR04_Data_t;


// // ==========================================================
// //                  函数接口
// // ==========================================================

// // 初始化所有传感器数据结构和定时器
// void SR04_Init(void);

// // 兼容旧代码的接口 (内部调用优化的触发逻辑)
// void SR04_Update_1(void);
// void SR04_Update_2(void);
// void SR04_Update_3(void);
// void SR04_Update_4(void);

// // 兼容旧代码的获取距离接口
// float Get_SR04_Distance_1(void);
// float Get_SR04_Distance_2(void);
// float Get_SR04_Distance_3(void);
// float Get_SR04_Distance_4(void);

// // 通用获取接口 (带有效性检查)
// float Ultrasonic_GetDistance_cm(uint8_t sensor_index);

// // 获取传感器健康状态 (1=正常, 0=故障/超时)
// uint8_t SR04_GetStatus(uint8_t sensor_index);

// #endif // __SR04_H