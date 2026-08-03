/**
 ******************************************************************************
 * @file           : app_BMS_Sensor.c
 * @author         : [Your Name/Team]
 * @version        : V1.0
 * @date           : 2026-03-11
 * @brief          : 传感器数据采集任务，负责处理超声波阵列与 BMS 电池状态
 *
 * @note           任务优先级: 普通 (Normal)
 *
 * 主要功能：
 * 1. 挂起读取超声波消息队列 (Queue_SR04DataHandle)，并进行数据解析处理
 * 2. 轮询调用 AXEBMS_RS485_Process() 维持 Modbus RTU 状态机运转
 * 3. 提取 BMS 回传的真实 SOC、最高温度，并更新到全局上下文 g_app
 * 4. 按 50ms 周期执行推杆 (Lifter) 电机的逻辑状态机
 *
 * 注意事项：
 * - BMS 通信依赖底层的 ModbusRtuBus 异步回调机制，本任务只负责轮询与取值。
 ******************************************************************************
 */

// #include "app_context.h"
// #include "FreeRTOS.h"
// #include "task.h"
// #include "main.h"
// #include "cmsis_os.h"
// #include "serial_port.h"
// #include "sr04_uart.h"
// #include "AXEBMS_RS485.h"
// #include "lifter.h"
// extern uint8_t rx_dma_buffer[RX_BUF_SIZE];
// extern osMessageQueueId_t Queue_SR04DataHandle;
// extern AXEBMS_RS485 my_bms;
// extern osSemaphoreId_t Sem_TxCompleteHandle; // 二值信号量
// /* USER CODE BEGIN Header_StartTask_BMS_Sensor */
// /**
//  * @brief Function implementing the Task_BMS_Sensor thread.
//  * @param argument: Not used
//  * @retval None
//  */
// /* USER CODE END Header_StartTask_BMS_Sensor */
// void StartTask_BMS_Sensor(void *argument)
// {
//   /* USER CODE BEGIN StartTask_BMS_Sensor */
//   UartRxMsg_t sr04_msg;
//   // 启动 超声波

//   SR04_UART_Init();
//   /* Infinite loop */
//   for (;;)
//   {
//     // 1. 检查超声波队列 (非阻塞，或者短超时)
//     if (osMessageQueueGet(Queue_SR04DataHandle, &sr04_msg, NULL, 0) == osOK)
//     {
//       SR04_Process_Data(sr04_msg.data, sr04_msg.len);
//     }

//     // 等待 RS485 数据 (或者超时 100ms 去做其他检查)
//     AXEBMS_RS485_Process(&my_bms);
//     // =================================================================
//     //      <<<---更新电池电量全局变量 ---<<<
//     // =================================================================
//     // 检查BMS通信是否正常
//     if (AXEBMS_RS485_GetState(&my_bms) == 0)
//     {
//       // 通信正常，从my_bms结构体中获取最新数据
//       AXEBMS_Data *bms_data = AXEBMS_RS485_GetData(&my_bms);
//       // 确保 voltage_int 存储的是 SOC (0-100)
//       g_app.voltage_int = (uint8_t)bms_data->RSOC;

//       // ============================================================
//       // <<<--- 读取 BMS 最高温度 --->>>
//       // ============================================================
//       // bms_data->Tmax 单位是 0.1度 (例如 285 = 28.5度)
//       // 我们除以 10 取整，并限制范围防溢出
//       float temp_val = (float)bms_data->Tmax / 10.0f;

//       // 简单限幅 (0 ~ 100度)
//       if (temp_val < 0)
//         temp_val = 0;
//       if (temp_val > 100)
//         temp_val = 100;

//       g_app.bms_temp = (uint8_t)temp_val;
//     }
//     else
//     {
//       // 通信故障，发送特殊值 (例如 0xFF 表示未知/故障)
//       g_app.voltage_int = 0xFF;
//       g_app.bms_temp = 0xFF; // 温度也设为 0xFF 表示未知
//     }
//     // 3. 处理推杆状态机 (每 50ms 跑一次足够了)
//     Lifter_Task_Update();
//     osDelay(50);
//   }
//   /* USER CODE END StartTask_BMS_Sensor */
// }

/**
 ******************************************************************************
 * @file           : app_BMS_Sensor.c
 * @brief          : 传感器采集与上位机遥测任务
 ******************************************************************************
 */

#include "app_context.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "serial_port.h"
#include "sr04_uart.h"
#include "AXEBMS_RS485.h"
#include "lifter.h"
#include "odrive_can.h" // 必须包含此头文件以使用 CAN_Enqueue_Message
#include "chassis_control.h"
#include <string.h>
#include <math.h>

// 引用外部对象
extern osMessageQueueId_t Queue_SR04DataHandle;
extern AXEBMS_RS485 my_bms;
extern AppContext_t g_app;
extern uint8_t Ctrl_Mode;
extern float host_vx;
extern float host_omega;
extern uint16_t mapped_throttle;
extern uint16_t mapped_steering;
extern volatile ODrive_Data_t g_odrive_motors[ODRIVE_MAX_MOTORS];

// 内部私有函数：打包并发送 0x507 核心 UI 帧
static void Send_Telemetry_Core_Frame(void);
// 内部私有函数：打包并发送 0x508/0x509 里程计帧
static void Send_Telemetry_Odom_Frames(void);

/**
 * @brief 传感器与反馈任务主循环
 */
void StartTask_BMS_Sensor(void *argument)
{
    UartRxMsg_t sr04_msg;
    uint32_t last_ui_tick = 0;
    uint32_t last_odom_tick = 0;
    uint32_t last_bms_ok_tick = 0;
    uint8_t last_bms_soc = 0xFF;
    uint8_t last_bms_temp = 0xFF;

    // 初始化超声波硬件
    SR04_UART_Init();

    for (;;)
    {
        // 1. 处理超声波串口队列 (解析雷达数据)
        if (osMessageQueueGet(Queue_SR04DataHandle, &sr04_msg, NULL, 0) == osOK)
        {
            SR04_Process_Data(sr04_msg.data, sr04_msg.len);
        }

        // 2. 维护 Modbus RTU 状态机 (解析 BMS 数据)
        AXEBMS_RS485_Process(&my_bms);

        // 3. 同步 BMS 数据到全局变量
        AXEBMS_Data *bms_data = AXEBMS_RS485_GetData(&my_bms);
        if (AXEBMS_RS485_GetState(&my_bms) != -3 &&
            bms_data->Voltage > 0U &&
            bms_data->RSOC <= 100U &&
            bms_data->Tmax <= 1000U)
        {
            last_bms_ok_tick = HAL_GetTick();
            last_bms_soc = (uint8_t)bms_data->RSOC;

            float temp_val = (float)bms_data->Tmax / 10.0f;
            if (temp_val < 0)
                temp_val = 0;
            if (temp_val > 100)
                temp_val = 100;
            last_bms_temp = (uint8_t)temp_val;

            g_app.voltage_int = last_bms_soc;
            g_app.bms_temp = last_bms_temp;
        }
        else
        {
            g_app.voltage_int = 0x01; // BMS 通信故障标志
            g_app.bms_temp = 0xFF;
            if (last_bms_ok_tick != 0U && (HAL_GetTick() - last_bms_ok_tick) < 3000U)
            {
                g_app.voltage_int = last_bms_soc;
                g_app.bms_temp = last_bms_temp;
            }
        }

        // 4. 处理推杆高度逻辑状态机 (50ms 级别逻辑)
        Lifter_Task_Update();

        // =========================================================
        // 5. 错峰发送上位机反馈 (通过 CAN 队列)
        // =========================================================
        uint32_t now = HAL_GetTick();

        // // --- A. 发送核心 UI 帧 (0x507): 周期 40ms ---
        if (now - last_ui_tick >= 40)
        {
            last_ui_tick = now;
            Send_Telemetry_Core_Frame();
        }

        // --- B. 发送里程计帧 (0x508/0x509): 周期 100ms ---
        if (now - last_odom_tick >= 100)
        {
            last_odom_tick = now;
            Send_Telemetry_Odom_Frames();
        }

        // 任务节拍 20ms
        osDelay(20);
    }
}

/**
 * @brief 打包发送 0x507 帧 (线速度, 角速度, 模式, 电量, 温度, 草袋)
 * 使用 Int16 缩放方案，避开浮点数传输风险
 */
static void Send_Telemetry_Core_Frame(void)
{
    uint8_t tx_buf[8];
    float cur_vx = 0.0f;
    float cur_w = 0.0f;

    // 根据模式选择反馈源 (指令回显 vs 遥控实时值)
    if (Ctrl_Mode >= 1)
    {
        cur_vx = host_vx;
        cur_w = host_omega;
    }
    else
    {
        float f_in = ((float)mapped_throttle - 2048.0f) / 2048.0f;
        float t_in = ((float)2048 - (float)mapped_steering) / 2048.0f;
        if (fabsf(f_in) < 0.1f)
            f_in = 0.0f;
        if (fabsf(t_in) < 0.1f)
            t_in = 0.0f;

        cur_vx = f_in * g_app.user_set_speed;
        cur_w = t_in * g_app.user_set_speed * g_app.turn_speed;
    }

    // 缩放：1.0 m/s -> 1000
    int16_t scaled_vx = (int16_t)(cur_vx * 1000.0f);
    int16_t scaled_w = (int16_t)(cur_w * 1000.0f);

    // 字节填充 (大端序，方便上位机查看原始报文)
    tx_buf[0] = (uint8_t)(scaled_vx >> 8);
    tx_buf[1] = (uint8_t)(scaled_vx & 0xFF);
    tx_buf[2] = (uint8_t)(scaled_w >> 8);
    tx_buf[3] = (uint8_t)(scaled_w & 0xFF);

    tx_buf[4] = Ctrl_Mode;
    tx_buf[5] = g_app.voltage_int;
    tx_buf[6] = g_app.bms_temp;

    //合并草袋状态与割刀状态
    uint8_t status_flags = 0;
    if (g_app.grass_bag_status)
        status_flags |= (1 << 0); // Bit 0
    if (g_app.is_bag_connected)
        status_flags |= (1 << 1); // Bit 1
    if (g_app.is_bag_full)
        status_flags |= (1 << 2); // Bit 2

    // 将割刀档位 (0~3) 移位到 Bit 4 和 Bit 5
    // 注意：如果是 API 模式，反馈 API 的设定值；如果是遥控模式，反馈遥控器的设定值
    status_flags |= ((g_app.knob_blade & 0x03) << 4);

    tx_buf[7] = status_flags;

    // 入队待发
    CAN_Enqueue_Message(0x507, tx_buf, 8, 0);
}

/**
 * @brief 打包发送 0x508 & 0x509 里程计位置帧
 */
static void Send_Telemetry_Odom_Frames(void)
{
    uint8_t tx_buf[8];

    float pos_fr = 0, pos_rr = 0, pos_fl = 0, pos_rl = 0;

#if MOTOR_DRIVER_USED == DRIVER_TYPE_ODRIVE
    // ODrive 直接提供绝对位置
    pos_fr = g_odrive_motors[1].encoder_pos;
    pos_rr = g_odrive_motors[2].encoder_pos;
    pos_fl = -g_odrive_motors[3].encoder_pos;
    pos_rl = -g_odrive_motors[4].encoder_pos;

#elif MOTOR_DRIVER_USED == DRIVER_TYPE_MSSD
    // MSSD 采用软件积分计算绝对位置 (在 100ms 周期内积分)
    static float soft_pos_fr = 0, soft_pos_rr = 0, soft_pos_fl = 0, soft_pos_rl = 0;
    float dt = 0.1f; // 此函数每 100ms 调用一次，故 dt=0.1秒

    // 提取 RPM 并转为 转/秒 (Turns/s)
    float rps_fr = (float)g_mssd_drives[MSSD_NODE_RIGHT].actual_rpm_r / 60.0f;
    float rps_rr = (float)g_mssd_drives[MSSD_NODE_RIGHT].actual_rpm_l / 60.0f;
    // 左侧读回来的速度要反向补偿
    float rps_fl = -(float)g_mssd_drives[MSSD_NODE_LEFT].actual_rpm_r / 60.0f;
    float rps_rl = -(float)g_mssd_drives[MSSD_NODE_LEFT].actual_rpm_l / 60.0f;

    // 梯形积分
    soft_pos_fr += rps_fr * dt;
    soft_pos_rr += rps_rr * dt;
    soft_pos_fl += rps_fl * dt;
    soft_pos_rl += rps_rl * dt;

    pos_fr = soft_pos_fr;
    pos_rr = soft_pos_rr;
    pos_fl = soft_pos_fl;
    pos_rl = soft_pos_rl;
#endif

    // --- 0x508: 右侧轮位置 ---
    memcpy(&tx_buf[0], &pos_fr, 4);
    memcpy(&tx_buf[4], &pos_rr, 4);
    CAN_Enqueue_Message(0x508, tx_buf, 8, 0);

    // --- 0x509: 左侧轮位置 ---
    memcpy(&tx_buf[0], &pos_fl, 4);
    memcpy(&tx_buf[4], &pos_rl, 4);
    CAN_Enqueue_Message(0x509, tx_buf, 8, 0);
}
