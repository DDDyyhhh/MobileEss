// pairing_receiver.c
/**
 ******************************************************************************
 * @file           : pairing_receiver.c
 * @brief          : 遥控器无线配对 (对码) 状态机与底层遥测数据包组装
 *
 * 主要功能：
 * 1. 配对状态机：响应请求 (REQ) -> 接收新信道地址 (ADDR) -> 切换并测试连通性 (TEST)
 * 2. 动态信道切换：在确认对码后，调用 E34 驱动更改通信频率和地址，防止多机干扰
 * 3. 遥测回传 (`receiver_resport`)：将割刀状态、车速、电池 SOC、推杆高度、超声波避障距离
 *    以及异常报错码打包，按自定义协议 (0xCC ... 0xDD) 回传给遥控器。
 *
 * 注意事项：
 * - E34 模块在切换地址或模式时需要响应时间，必须使用 RTOS 的 `osDelay` 等待其内部复位完成。
 * - 遥测数据回传采用了无符号 8 位截断策略 (超声波限幅 255cm)，确保数据包体积轻量化。
 ******************************************************************************
 */
#include "pairing_receiver.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>
#include <pairing_common.h>
#include <math.h>
#include "car_Control.h" // <<<--- 需要包含这个头文件来使用 g_current_target_speed_rpm
#include "lifter.h"      // <<<--- 1. 包含 lifter.h 头文件
#include "serial_port.h"
#include "app_context.h"
#include "cmsis_os.h"        // 引入 RTOS 头文件
#include "mssd_can.h"        // 引入 MSSD CAN 驱动头文件
#include "chassis_control.h" // 引入底盘控制头文件以访问底盘状态
// ============================================================
// <<<--- 【硬件平台切换宏】 无线通信模块选择 --->>>
// ============================================================
// 0: 使用老款 E34 模块 (半双工)
// 1: 使用新款 A28 模块 (全双工，推荐)
#define WIRELESS_MODULE_TYPE 1

#if WIRELESS_MODULE_TYPE == 0
#include "E34.h"
#elif WIRELESS_MODULE_TYPE == 1
#include "A28.h"
#else
#error "Please select a valid WIRELESS_MODULE_TYPE (0 or 1)"
#endif
// ============================================================

// 内部状态
static pairing_state_t pairing_state = PAIRING_IDLE;
extern volatile uint16_t new_address1;
static uint32_t pairing_timer = 0;
volatile uint8_t REQ = 0;
volatile uint8_t ADDR = 0;
extern volatile uint8_t TESTok;
extern uint16_t RPM;
static uint8_t buf[32];
extern AppContext_t g_app;

// 适配函数：动态调用对应模块的地址设置指令
static void RF_Module_Set_Address(uint16_t addr)
{
#if WIRELESS_MODULE_TYPE == 0
    E34_Set_Address(addr);
#elif WIRELESS_MODULE_TYPE == 1
    A28_Set_Address(addr);
#endif
}

// 适配函数：让模块进入最适合通信的透传模式
void RF_Module_Set_Trans_Mode(void)
{
#if WIRELESS_MODULE_TYPE == 0
    // E34 透传必须使用全双工模式引脚配置
    E34_Set_FullDuplex_Mode();
#elif WIRELESS_MODULE_TYPE == 1
    // A28 强烈推荐全双工模式
    A28_Set_FullDuplex_Mode();
#endif
}

pairing_state_t pairing_receiver_get_state(void)
{
    return pairing_state;
}

static void Send_Pairing_Packet(uint8_t cmd, uint8_t *payload, uint8_t len)
{
    uint8_t buf[10];
    buf[0] = 0xAA;
    buf[1] = cmd;
    if (len > 0 && payload != NULL)
    {
        memcpy(&buf[2], payload, len);
    }
    buf[2 + len] = 0xBB;

    // 使用 HAL_UART_Transmit (阻塞发送没问题，因为配对时不需要高并发)
    HAL_UART_Transmit(&huart3, buf, 3 + len, 100);
}
void pairing_receiver_init(void)
{
    pairing_state = PAIRING_IDLE;
    new_address1 = 0;
    pairing_timer = 0;
    TESTok = 0;
    REQ = 0;
    ADDR = 0;

    Serial_Printf(SERIAL_PORT_2, "[DEBUG] Setting RF Module to 0000...\r\n");

    // 强制终止当前的 UART 活动
    HAL_UART_AbortTransmit(&huart3);
    HAL_UART_AbortReceive(&huart3);
    osDelay(20);

    // 调用抽象接口设置默认地址
    RF_Module_Set_Address(DEFAULT_ADDRESS);
    osDelay(20);

    // 调用抽象接口进入透传模式
    RF_Module_Set_Trans_Mode();
    osDelay(20);

    // 清除错误标志
    __HAL_UART_CLEAR_OREFLAG(&huart3);
}

void pairing_receiver_handle_message(uint8_t *data, uint16_t length)
{

    uint8_t cmd = data[1]; // 命令字节在第二位

    switch (cmd)
    {
    case PAIRING_REQ:
        // 【修改】不仅 IDLE 可以接收，STARTED 也可以接收（说明对方没收到ACK，在重试）
        if (pairing_state == PAIRING_IDLE || pairing_state == PAIRING_STARTED)
        {
            REQ = 1;
        }

        break;

    case PAIRING_ADDR:
        if (pairing_state == PAIRING_STARTED)
        {
            // 地址在 data[2] 和 data[3]
            new_address1 = (data[2] << 8) | data[3];
            ADDR = 1;
        }

        break;

    case PAIRING_TEST:
        if (pairing_state == PAIRING_ADDR_RECEIVED)
        {

            TESTok = 1;
        }
        break;
    }
}
void pairing_receiver_task(void)
{
    // ============================================================
    // 1. 处理 REQ 请求 (响应配对)
    // ============================================================
    if (REQ == 1)
    {
        // 连发 3 次 ACK 保证遥控器能收到
        for (int i = 0; i < 3; i++)
        {
            Send_Pairing_Packet(PAIRING_ACK, NULL, 0);
            osDelay(10);
        }

        REQ = 0; // 清除标志

        // 无论是第一次收到，还是遥控器没听到而重发的 REQ，都刷新状态和定时器
        pairing_state = PAIRING_STARTED;
        pairing_timer = HAL_GetTick(); // 【新增】重置超时定时器

        // Serial_Printf(SERIAL_PORT_2, "[Pairing] REQ Recv, Sent ACK...\r\n");
    }

    // ============================================================
    // 2. 状态: 已启动，等待 ADDR 包
    // ============================================================
    if (pairing_state == PAIRING_STARTED)
    {
        if (ADDR == 1)
        {
            // Serial_Printf(SERIAL_PORT_2, "[Pairing] Got Addr: %04X\r\n", new_address1);

            // 1. 必须在原地址 (0000) 上连发 CONFIRM，让遥控器听到
            for (int i = 0; i < 4; i++)
            {
                Send_Pairing_Packet(PAIRING_CONFIRM, NULL, 0);
                osDelay(10);
            }

            // 2. 确认发完后，自己再切换到新地址
            RF_Module_Set_Address(new_address1);
            osDelay(50); // 【对齐遥控器】 给模块充足的生效时间

            ADDR = 0; // 清除标志

            // 3. 进入下一阶段
            pairing_state = PAIRING_ADDR_RECEIVED;
            pairing_timer = HAL_GetTick(); // 【重置】刷新超时定时器
            // Serial_Printf(SERIAL_PORT_2, "[Pairing] Waiting for TEST...\r\n");
        }
        // 【新增】 超时保护 1：如果在 STARTED 状态停留超 3秒没收到 ADDR
        else if (HAL_GetTick() - pairing_timer > 1000)
        {
            // Serial_Printf(SERIAL_PORT_2, "[Timeout] No ADDR received. Resetting...\r\n");
            pairing_state = PAIRING_IDLE; // 直接回退到空闲
            REQ = 0;
            ADDR = 0;
        }
    }

    // ============================================================
    // 3. 状态: 已切新地址，等待 TEST 测试包
    // ============================================================
    if (pairing_state == PAIRING_ADDR_RECEIVED)
    {
        if (TESTok == 1)
        {
            // 正常收到 TEST，连发 TEST_OK
            for (int i = 0; i < 4; i++)
            {
                Send_Pairing_Packet(PAIRING_TEST_OK, NULL, 0);
                osDelay(10);
            }

            pairing_state = PAIRING_SUCCESS; // 彻底成功
            TESTok = 0;

            Serial_Printf(SERIAL_PORT_2, "PAIRING_SUCCESS\r\n");
        }
        // 【优化】 超时保护 2：如果在 ADDR_RECEIVED 状态停留超 3秒没收到 TEST
        else if (HAL_GetTick() - pairing_timer > 1000)
        {
            Serial_Printf(SERIAL_PORT_2, "[Timeout] No TEST received. Resetting to 0000...\r\n");

            // 1. 既然已经切了新地址，必须切回默认地址 (0000)
            RF_Module_Set_Address(DEFAULT_ADDRESS);
            osDelay(50); // 【对齐遥控器】

            // 2. 状态彻底回退到 IDLE
            pairing_state = PAIRING_IDLE;
            REQ = 0;
            ADDR = 0;
            TESTok = 0;
        }
    }
}

void receiver_resport(void)
{
    // --- 新的转速处理逻辑 ---

    // ============================================================
    // <<<--- 从 MSSD 驱动器提取温度并计算功率 --->>>
    // ============================================================
#if MOTOR_DRIVER_USED == DRIVER_TYPE_MSSD
    extern volatile MSSD_Data_t g_mssd_drives[]; // 引用驱动器数据

    // 右前轮 (FR) = Node Right, 假设为 Right 通道
    g_app.odrive_temps[0] = (uint8_t)g_mssd_drives[MSSD_NODE_RIGHT].temp_r;
    // float pwr_fr = fabsf(g_mssd_drives[MSSD_NODE_RIGHT].bus_voltage * g_mssd_drives[MSSD_NODE_RIGHT].bus_current_r);
    float pwr_fr = fabsf(g_mssd_drives[MSSD_NODE_RIGHT].bus_current_r);

    // 右后轮 (RR) = Node Right, 假设为 Left 通道
    g_app.odrive_temps[1] = (uint8_t)g_mssd_drives[MSSD_NODE_RIGHT].temp_l;
    // float pwr_rr = fabsf(g_mssd_drives[MSSD_NODE_RIGHT].bus_voltage * g_mssd_drives[MSSD_NODE_RIGHT].bus_current_l);
    float pwr_rr = fabsf(g_mssd_drives[MSSD_NODE_RIGHT].bus_current_l);

    // 左前轮 (FL) = Node Left, Right 通道
    g_app.odrive_temps[2] = (uint8_t)g_mssd_drives[MSSD_NODE_LEFT].temp_r;
    // float pwr_fl = fabsf(g_mssd_drives[MSSD_NODE_LEFT].bus_voltage * g_mssd_drives[MSSD_NODE_LEFT].bus_current_r);
    float pwr_fl = fabsf(g_mssd_drives[MSSD_NODE_LEFT].bus_current_r);

    // 左后轮 (RL) = Node Left, Left 通道
    g_app.odrive_temps[3] = (uint8_t)g_mssd_drives[MSSD_NODE_LEFT].temp_l;
    // float pwr_rl = fabsf(g_mssd_drives[MSSD_NODE_LEFT].bus_voltage * g_mssd_drives[MSSD_NODE_LEFT].bus_current_l);
    float pwr_rl = fabsf(g_mssd_drives[MSSD_NODE_LEFT].bus_current_l);

    float temp_pwr_fr = pwr_fr * 10.0f + 0.5f;
    float temp_pwr_rr = pwr_rr * 10.0f + 0.5f;
    float temp_pwr_fl = pwr_fl * 10.0f + 0.5f;
    float temp_pwr_rl = pwr_rl * 10.0f + 0.5f;

    if (temp_pwr_fr > 255.0f)
        temp_pwr_fr = 255.0f; // 限制在 uint8_t 最大值
    if (temp_pwr_rr > 255.0f)
        temp_pwr_rr = 255.0f; // 限制在 uint8_t 最大值
    if (temp_pwr_fl > 255.0f)
        temp_pwr_fl = 255.0f; // 限制在 uint8_t 最大值
    if (temp_pwr_rl > 255.0f)
        temp_pwr_rl = 255.0f; // 限制在 uint8_t 最大值

    // 将功率缩小 10 倍存入 uint8_t (例如 150W 存为 15，最大可表示 2550W)
    g_app.motor_powers[0] = (uint8_t)(temp_pwr_fr);
    g_app.motor_powers[1] = (uint8_t)(temp_pwr_rr);
    g_app.motor_powers[2] = (uint8_t)(temp_pwr_fl);
    g_app.motor_powers[3] = (uint8_t)(temp_pwr_rl);

    // g_app.motor_powers[0] = (uint8_t)(pwr_fr / 10.0f);
    // g_app.motor_powers[1] = (uint8_t)(pwr_rr / 10.0f);
    // g_app.motor_powers[2] = (uint8_t)(pwr_fl / 10.0f);
    // g_app.motor_powers[3] = (uint8_t)(pwr_rl / 10.0f);
#elif MOTOR_DRIVER_USED == DRIVER_TYPE_OID
    extern volatile OID_Data_t g_oid_drives[];

    // [0]前右 FR = Node 1, [1]后右 RR = Node 3
    // [2]前左 FL = Node 2, [3]后左 RL = Node 4
    g_app.odrive_temps[0] = (uint8_t)g_oid_drives[OID_NODE_FR].motor_temp;
    g_app.odrive_temps[1] = (uint8_t)g_oid_drives[OID_NODE_RR].motor_temp;
    g_app.odrive_temps[2] = (uint8_t)g_oid_drives[OID_NODE_FL].motor_temp;
    g_app.odrive_temps[3] = (uint8_t)g_oid_drives[OID_NODE_RL].motor_temp;

    // OID 直接读取相电流 (A)，放大 10 倍存入 uint8_t (单位 0.1A)
    float cur_fr = fabsf(g_oid_drives[OID_NODE_FR].phase_current);
    float cur_rr = fabsf(g_oid_drives[OID_NODE_RR].phase_current);
    float cur_fl = fabsf(g_oid_drives[OID_NODE_FL].phase_current);
    float cur_rl = fabsf(g_oid_drives[OID_NODE_RL].phase_current);

    float temp_cur_fr = cur_fr * 10.0f + 0.5f;
    float temp_cur_rr = cur_rr * 10.0f + 0.5f;
    float temp_cur_fl = cur_fl * 10.0f + 0.5f;
    float temp_cur_rl = cur_rl * 10.0f + 0.5f;

    if (temp_cur_fr > 255.0f) temp_cur_fr = 255.0f;
    if (temp_cur_rr > 255.0f) temp_cur_rr = 255.0f;
    if (temp_cur_fl > 255.0f) temp_cur_fl = 255.0f;
    if (temp_cur_rl > 255.0f) temp_cur_rl = 255.0f;

    g_app.motor_powers[0] = (uint8_t)(temp_cur_fr);
    g_app.motor_powers[1] = (uint8_t)(temp_cur_rr);
    g_app.motor_powers[2] = (uint8_t)(temp_cur_fl);
    g_app.motor_powers[3] = (uint8_t)(temp_cur_rl);
#endif
    // 1. 获取 float 类型的目标速度
    // 2. 将其缩放到一个 uint8_t 范围内。例如，乘以10。
    //    确保值不会超过255。
    uint8_t speed_to_send = (uint8_t)(g_current_target_speed_rpm * 10.0f);
    if (speed_to_send > 255)
    {
        speed_to_send = 255;
    }

    // --- 2. 处理刀盘高度 ---
    float current_height = Lifter_GetCurrentHeight_cm();        // a. 获取 float 类型的当前高度
    uint8_t height_to_send = (uint8_t)(current_height * 10.0f); // b. 乘以10，转换为 0.1cm 为单位的整数

    // float distance = Ultrasonic_GetDistance_cm(1); // 获取第一个超声波传感器的距离
    buf[0] = 0xCC;                     // 帧头
    buf[1] = g_app.receiver_status[0]; // 割刀状态
    buf[2] = g_app.receiver_status[1]; // 车速
    buf[3] = g_app.receiver_status[2]; //
    buf[4] = g_app.voltage_int;        // 电池
    // buf[5] = Get_SR04_Distance();
    // buf[5] = distance;

    buf[5] = RPM; // 割刀电机电流
    // buf[5] = voltage_int;

    buf[6] = height_to_send; // 高度

    // ============================================================
    // <<<---  独立打包草袋的物理状态 --->>>
    // ============================================================
    uint8_t bag_flags = 0;

    // Bit 0: 门开合状态 (1=开, 0=关)
    if (g_app.grass_bag_status)
        bag_flags |= (1 << 0);

    // Bit 1: 满载状态 (1=满, 0=未满)
    if (g_app.is_bag_full)
        bag_flags |= (1 << 1);

    // Bit 2: 连接状态 (1=未连接, 0=已连接)  <-- 注意这里1代表未连接，方便判断
    if (!g_app.is_bag_connected)
        bag_flags |= (1 << 2);

    buf[7] = bag_flags; // 写入独立通道

    // --- 4. 超声波数据限幅处理 (新增) ---
    // 定义一个临时的浮点数组方便循环处理，或者直接写4次判断
    float dists[4] = {g_app.d1, g_app.d2, g_app.d3, g_app.d4};

    for (int i = 0; i < 4; i++)
    {
        if (dists[i] > 255.0f)
        {
            buf[8 + i] = 255;
        }
        else if (dists[i] < 0.0f) // 同时也建议过滤负数
        {
            buf[8 + i] = 0;
        }
        else
        {
            buf[8 + i] = (uint8_t)dists[i];
        }
    }

    // 逻辑：
    // 1. 如果有物理故障锁存 (碰撞/急停)，必须发送状态 (1~6)
    // 2. 如果没有锁存，但有系统提示消息 (初始化/倒草 7~13)，也要发送
    // 3. 否则发送 0

    if (g_app.g_collision_latched)
    {
        buf[12] = (uint8_t)g_app.g_system_report_state;
    }
    else if (g_app.g_system_report_state >= REPORT_INIT_SUCCESS) // 状态码 >= 7
    {
        // 发送非锁存类的提示信息 (7, 8, 9, 13 等)
        buf[12] = (uint8_t)g_app.g_system_report_state;
    }
    else
    {
        buf[12] = 0;
    }

    buf[13] = g_app.Unlock_tips;

    buf[14] = g_app.odrive_temps[0]; // 右前 驱动 温度
    buf[15] = g_app.odrive_temps[1]; // 右后 驱动 温度
    buf[16] = g_app.odrive_temps[2]; // 左前 驱动 温度
    buf[17] = g_app.odrive_temps[3]; // 左后 驱动 温度

    buf[18] = g_app.bms_temp; // BMS 最高温度

    // --- 装填功率数据 ---
    buf[19] = g_app.motor_powers[0]; // 右前 功率
    buf[20] = g_app.motor_powers[1]; // 右后 功率
    buf[21] = g_app.motor_powers[2]; // 左前 功率
    buf[22] = g_app.motor_powers[3]; // 左后 功率

    buf[23] = 0xDD; // 帧尾

    Serial_SendData(SERIAL_PORT_3, buf, 24);
}
