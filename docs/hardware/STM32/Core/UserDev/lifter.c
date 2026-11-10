
/**
 ******************************************************************************
 * @file           : lifter.c
 * @author         : [Your Name/Team]
 * @version        : V1.2
 * @date           : 2026-03-11
 * @brief          : 割草机刀盘高度调节与草袋开合控制状态机
 *
 * 主要功能：
 * 1. 刀盘高度：通过时间积分估算电机位置，具备开机归位与掉电保存功能。
 * 2. 草袋开合：单键切换 (Toggle)，采用超时保护机制 (无物理限位) 开合。
 * 3. 延时持久化 (Lazy Save)：高度调节停止后 10 秒才写入 Flash，极大延长寿命。
 ******************************************************************************
 */

#include "lifter.h"
#include "EG2104.h"
#include "tim.h"
#include "serial_port.h"
#include "flash_storage.h"
#include "connet_check.h"
#include "math.h"
#include "app_context.h"
// =================================================================
// 1. 硬件定义
// =================================================================
EG2104_HBridge_t blade_height_bridge = {
    .left_timer = &htim3,
    .right_timer = &htim3,
    .left_in_channel = TIM_CHANNEL_3,
    .right_in_channel = TIM_CHANNEL_4,
};
EG2104_HBridge_t grass_bag_bridge = {
    .left_timer = &htim2,
    .right_timer = &htim2,
    .left_in_channel = TIM_CHANNEL_4,
    .right_in_channel = TIM_CHANNEL_3,
};

// =================================================================
// 2. 核心参数与数据表
// =================================================================
// --- 刀盘高度参数 ---
#define LIFTER_SPEED 20.0f      // 你的设定速度
#define HOMING_TIMEOUT_MS 10000 // 归位超时 (最长行程约6秒，设8秒足够)
#define DEAD_BAND_MS 100        // 死区：误差小于100ms(约2mm)认为到位

// 上升速度补偿系数 (0.0 ~ 1.0)
// 假设上升比下降慢 (克服重力)。如果下降用6s，上升用8s，系数约为 0.75
// 如果你发现上升总是“不到位”，请调小这个值；如果上升“过头”，调大这个值。
#define UP_SPEED_FACTOR 0.80f

// --- 草袋开合参数 ---
#define GRASS_BAG_SPEED 60.0f      // 草袋电机速度 (根据实际机械阻力调整)
#define GRASS_BAG_TIMEOUT_MS 20000 // 草袋完全打开或关闭所需的时间 (毫秒)，请实际测试后修改

// --- 【核心】实测数据点 (时间 ms -> 高度 cm) ---
// 基于你提供的数据：0s=11.3, 1s=10.0, 2s=7.2, 3s=5.7, 4s=4.4, 5s=3.2, 6s=2.1
typedef struct
{
    uint32_t time_ms;
    float height_cm;
} HeightMap_t;

static const HeightMap_t HEIGHT_MAP[] = {
    {0, 11.0f},   // 0ms = 5档 = 11cm
    {2800, 9.0f}, // 2800 = 4档 = 9cm
    {4500, 7.0f}, // 4500 = 3档 = 7cm
    {6500, 5.0f}, // 6500ms = 2档 = 5cm
    {9500, 2.0f}, // 9500 = 1档 = 2cm
    {10000, 1.0f} // 预留一个极限值，防止计算越界
};
#define MAP_SIZE (sizeof(HEIGHT_MAP) / sizeof(HeightMap_t))

// --- 【核心】档位目标时间表 (根据要求计算) ---
// 5档(11cm) -> 约 230ms
// 4档(9cm)  -> 约 1360ms (在10.0和7.2之间插值)
// 3档(7cm)  -> 约 2130ms (在7.2和5.7之间插值)
// 2档(5cm)  -> 约 3540ms (在5.7和4.4之间插值)
// 1档(2cm)  -> 约 6100ms (在2.1附近外推)
static const uint32_t LEVEL_TARGET_MS[6] = {
    0,    // 0档占位
    9500, // 1档: 目标 2cm (最低) - 大幅增加时间
    6500, // 2档: 目标 5cm
    4500, // 3档: 目标 7cm
    2800, // 4档: 目标 9cm
    0     // 5档: 目标 11cm (最高，基准点)
};

// =================================================================
// 3. 内部变量
// =================================================================
static Lifter_MachineState g_state = LIFTER_STATE_IDLE;
static uint32_t g_last_tick = 0;
static uint32_t g_homing_start_ts = 0;

static float g_current_pos_ms = 0.0f;     // 当前虚拟位置 (ms)
static float g_target_pos_ms = 0.0f;      // 目标位置 (ms)
static float g_current_height_cm = 11.0f; // 对外显示的CM值

static int8_t g_dir = 0; // 1:Up(向0ms), -1:Down(向6000ms), 0:Stop

// === 延时保存 (Lazy Save) 变量 ===
#define FLASH_SAVE_DELAY_MS 10000        // 停稳后 10秒 (10000ms) 才保存 Flash
static uint8_t g_flash_save_pending = 0; // 0:无需保存, 1:等待保存中
static uint32_t g_flash_save_timer = 0;  // 记录停稳时的时间戳
static uint32_t g_dump_done_timer = 0;   // 用于倒草完成显示的计时

// --- 草袋状态变量 (【新增】) ---
typedef enum
{
    BAG_STATE_CLOSED = 0,
    BAG_STATE_OPENING,
    BAG_STATE_OPEN,
    BAG_STATE_CLOSING
} GrassBag_MachineState;

static GrassBag_MachineState g_bag_state = BAG_STATE_CLOSED;
static uint32_t g_bag_timer = 0;

// =================================================================
// 4. 辅助函数
// =================================================================

void EG2104_HBridge_Stop(EG2104_HBridge_t *bridge);
HAL_StatusTypeDef EG2104_HBridge_SetSpeed(EG2104_HBridge_t *bridge, float speed);

static void Motor_Up(void)
{ // 物理向上 (高度增加，时间减小)
    EG2104_HBridge_SetSpeed(&blade_height_bridge, LIFTER_SPEED);
    g_dir = 1;
    g_flash_save_pending = 0; // 只要电机动了，就取消未执行的保存计划
}

static void Motor_Down(void)
{ // 物理向下 (高度减小，时间增加)
    EG2104_HBridge_SetSpeed(&blade_height_bridge, -LIFTER_SPEED);
    g_dir = -1;
    g_flash_save_pending = 0; // 只要电机动了，就取消未执行的保存计划
}

void Lifter_Stop(void)
{
    EG2104_HBridge_Stop(&blade_height_bridge);
    g_dir = 0;
    // 停止时不改变 g_state，由调用者决定
}

// --- 草袋控制驱动 (【新增】) ---
static void GrassBag_Motor_Open(void)
{
    // 假设正转是打开草袋，如果实际相反，改为 -GRASS_BAG_SPEED 即可
    EG2104_HBridge_SetSpeed(&grass_bag_bridge, GRASS_BAG_SPEED);
}

static void GrassBag_Motor_Close(void)
{
    EG2104_HBridge_SetSpeed(&grass_bag_bridge, -GRASS_BAG_SPEED);
}

static void GrassBag_Motor_Stop(void)
{
    EG2104_HBridge_Stop(&grass_bag_bridge);
}

/**
 * @brief 根据当前毫秒数，反向插值计算厘米高度
 */
static void Update_Height_CM_From_MS(float current_ms)
{
    // 1. 边界处理
    if (current_ms <= 0)
    {
        g_current_height_cm = HEIGHT_MAP[0].height_cm;
        return;
    }
    if (current_ms >= HEIGHT_MAP[MAP_SIZE - 1].time_ms)
    {
        g_current_height_cm = HEIGHT_MAP[MAP_SIZE - 1].height_cm;
        return;
    }

    // 2. 线性插值
    for (int i = 0; i < MAP_SIZE - 1; i++)
    {
        if (current_ms >= HEIGHT_MAP[i].time_ms && current_ms < HEIGHT_MAP[i + 1].time_ms)
        {
            float t1 = (float)HEIGHT_MAP[i].time_ms;
            float t2 = (float)HEIGHT_MAP[i + 1].time_ms;
            float h1 = HEIGHT_MAP[i].height_cm;
            float h2 = HEIGHT_MAP[i + 1].height_cm;

            // 线性公式: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
            float ratio = (current_ms - t1) / (t2 - t1);
            g_current_height_cm = h1 + ratio * (h2 - h1);
            return;
        }
    }
}

/**
 * @brief 反向插值：根据 CM 计算 MS (用于开机恢复位置)
 */
static float Get_MS_From_Height_CM(float height_cm)
{
    // 保护范围
    if (height_cm >= HEIGHT_MAP[0].height_cm)
        return 0.0f; // >11cm -> 0ms
    if (height_cm <= HEIGHT_MAP[MAP_SIZE - 1].height_cm)
        return (float)HEIGHT_MAP[MAP_SIZE - 1].time_ms;

    // 遍历查找区间 (注意高度是递减的)
    for (int i = 0; i < MAP_SIZE - 1; i++)
    {
        float h_high = HEIGHT_MAP[i].height_cm;    // 如 11.0
        float h_low = HEIGHT_MAP[i + 1].height_cm; // 如 9.0

        // 如果目标高度在这个区间内
        if (height_cm <= h_high && height_cm > h_low)
        {
            float t1 = (float)HEIGHT_MAP[i].time_ms;
            float t2 = (float)HEIGHT_MAP[i + 1].time_ms;

            // 线性插值
            float ratio = (h_high - height_cm) / (h_high - h_low);
            return t1 + ratio * (t2 - t1);
        }
    }
    return 0.0f;
}

// =================================================================
// 5. 接口实现
// =================================================================

void Lifter_Init(void)
{
    HAL_TIM_PWM_Start(blade_height_bridge.left_timer, blade_height_bridge.left_in_channel);
    HAL_TIM_PWM_Start(blade_height_bridge.right_timer, blade_height_bridge.right_in_channel);

    HAL_TIM_PWM_Start(grass_bag_bridge.left_timer, grass_bag_bridge.left_in_channel);
    HAL_TIM_PWM_Start(grass_bag_bridge.right_timer, grass_bag_bridge.right_in_channel);

    // 确保电机初始处于停止状态
    Lifter_Stop();
    GrassBag_Motor_Stop();

    // 初始化草袋状态为关闭
    g_bag_state = BAG_STATE_CLOSED;
    g_app.grass_bag_status = 0; // 全局标志位

    // 2. 尝试读取 Flash
    AppSettings_t settings;
    bool load_ok = FlashStorage_LoadSettings(&settings);

    // 3. 判断是否有效
    if (load_ok && settings.lifter_current_height_cm >= 1.0f && settings.lifter_current_height_cm <= 13.0f)
    {
        // === 成功读取：恢复状态，跳过归位 ===
        g_current_height_cm = settings.lifter_current_height_cm;
        // 反算当前的时间位置 (ms)
        g_current_pos_ms = Get_MS_From_Height_CM(g_current_height_cm);
        // 目标设为当前，保持静止
        g_target_pos_ms = g_current_pos_ms;

        g_state = LIFTER_STATE_IDLE; // 直接进入空闲
        // Serial_Printf(SERIAL_PORT_2, "[LIFTER] Loaded from Flash: %.1f cm (%.0f ms). Skip Homing.\r\n",
        //               g_current_height_cm, g_current_pos_ms);
    }
    else
    {
        // === 读取失败或非法：强制归位 ===
        g_state = LIFTER_STATE_HOMING_START;
        // Serial_Printf(SERIAL_PORT_2, "[LIFTER] No saved data. Force Homing...\r\n");
    }

    g_last_tick = HAL_GetTick();
}

/**
 * @brief 暴露给用户的强制重新初始化接口 (长按触发)
 * @note  无视 Flash 历史记录，强制推杆上升到最高点进行物理归零
 */
void Lifter_User_Init(void)
{
    // 强制把状态机切到归位起点
    g_state = LIFTER_STATE_HOMING_START;

    // 可选：重置一下目标位置，防止归位完成后乱跑
    g_target_pos_ms = 0.0f;

    //  设置系统状态，让遥控器弹窗
    g_app.g_system_report_state = REPORT_BLADE_INITIALIZING;

    // Serial_Printf(SERIAL_PORT_2, "[LIFTER] User forced homing to MAX height!\r\n");
}

void Lifter_SetTargetLevel(uint8_t level)
{
    if (level < 1)
        level = 1;
    if (level > 5)
        level = 5;

    // 查表获取目标时间位置
    g_target_pos_ms = (float)LEVEL_TARGET_MS[level];

    // 如果不在归位中，启动自动调节
    if (g_state != LIFTER_STATE_HOMING_START && g_state != LIFTER_STATE_HOMING_RUNNING)
    {
        g_state = LIFTER_STATE_AUTO_ADJUST;
    }
}

void Lifter_SaveHeightToFlash(void)
{
    Suspend_Heartbeat_Check(1);

    // 稍微延时，确保电机完全停稳
    HAL_Delay(50);

    AppSettings_t settings;
    settings.lifter_current_height_cm = g_current_height_cm;
    settings.magic_number = APP_SETTINGS_MAGIC_NUMBER;

    if (FlashStorage_SaveSettings(&settings))
    {
        // Serial_Printf(SERIAL_PORT_2, "[LIFTER] Saved %.1f cm.\r\n", g_current_height_cm);
    }

    g_app.last_receive_timestamp = HAL_GetTick(); // 续命
    extern void receiver_resport(void);
    receiver_resport(); // 补发数据

    Suspend_Heartbeat_Check(0);
}

float Lifter_GetCurrentHeight_cm(void) { return g_current_height_cm; }
Lifter_MachineState Lifter_GetState(void) { return g_state; }

/**
 * @brief 设置草袋目标状态
 * @param open 1=打开，0=关闭
 */
void Lifter_SetGrassBagTarget(uint8_t open)
{
    if (open)
    {
        // 已关闭或正在关闭时，切换为打开
        if (g_bag_state == BAG_STATE_CLOSED || g_bag_state == BAG_STATE_CLOSING)
        {
            g_bag_state = BAG_STATE_OPENING;
            g_bag_timer = HAL_GetTick();
            GrassBag_Motor_Open();
        }
    }
    else
    {
        // 已打开或正在打开时，切换为关闭
        if (g_bag_state == BAG_STATE_OPEN || g_bag_state == BAG_STATE_OPENING)
        {
            g_bag_state = BAG_STATE_CLOSING;
            g_bag_timer = HAL_GetTick();
            GrassBag_Motor_Close();
        }
    }
}

/**
 * @brief 触发草袋开合 (暴露给 App 层的接口)
 */
void Lifter_ToggleGrassBag(void)
{
    // 如果当前是关闭，或正在关闭中 -> 切换为打开
    if (g_bag_state == BAG_STATE_CLOSED || g_bag_state == BAG_STATE_CLOSING)
    {
        Lifter_SetGrassBagTarget(1);
    }
    // 如果当前是打开，或正在打开中 -> 切换为关闭
    else if (g_bag_state == BAG_STATE_OPEN || g_bag_state == BAG_STATE_OPENING)
    {
        Lifter_SetGrassBagTarget(0);
    }
}

/**
 * @brief 周期任务
 */
void Lifter_Task_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt = now - g_last_tick;
    g_last_tick = now;
    // =================================================================
    //  A. 刀盘高度逻辑
    // =================================================================

    // --- 1. 位置积分 ---
    if (g_dir != 0 && g_state != LIFTER_STATE_HOMING_RUNNING)
    {
        float change = (float)dt;
        if (g_dir == 1)
        {
            g_current_pos_ms -= (change * UP_SPEED_FACTOR);
        }
        else
        {
            g_current_pos_ms += change;
        }
        if (g_current_pos_ms < 0)
            g_current_pos_ms = 0;
        if (g_current_pos_ms > 15000.0f)
            g_current_pos_ms = 15000.0f;
    }

    Update_Height_CM_From_MS(g_current_pos_ms);

    // --- 2. 状态机 ---
    switch (g_state)
    {
    case LIFTER_STATE_HOMING_START:
        Motor_Up();
        g_homing_start_ts = now;
        g_state = LIFTER_STATE_HOMING_RUNNING;
        break;

    case LIFTER_STATE_HOMING_RUNNING:
        if (now - g_homing_start_ts > HOMING_TIMEOUT_MS)
        {
            Lifter_Stop();
            g_current_pos_ms = 0.0f;
            g_target_pos_ms = 0.0f;
            g_state = LIFTER_STATE_HOMING_DONE;

            //  归位时间到（约10-11s），清除弹窗状态
            if (g_app.g_system_report_state == REPORT_BLADE_INITIALIZING)
            {
                g_app.g_system_report_state = REPORT_NONE;
            }

            // 【修改】归位完成，不立刻保存，而是触发 10 秒倒计时
            g_flash_save_pending = 1;
            g_flash_save_timer = now;
        }
        break;

    case LIFTER_STATE_HOMING_DONE:
        g_state = LIFTER_STATE_AUTO_ADJUST;
        break;

    case LIFTER_STATE_AUTO_ADJUST:
    {
        float err = g_target_pos_ms - g_current_pos_ms;

        // 死区判断 (到位)
        if (fabsf(err) <= DEAD_BAND_MS)
        {
            if (g_dir != 0)
            {
                Lifter_Stop();
                // 到位后不立刻保存，而是触发 10 秒倒计时
                g_flash_save_pending = 1;
                g_flash_save_timer = now;
            }
        }
        else if (err > 0)
        {
            if (g_dir != -1)
                Motor_Down();
        }
        else
        {
            if (g_dir != 1)
                Motor_Up();
        }
        break;
    }
    default:
        break;
    }

    // =================================================================
    //  <<<---  3. 延时保存 Flash (Lazy Save) 机制 ---<<<
    // =================================================================

    // 只有当有保存任务，且电机确实停稳了，才进行倒计时判定
    if (g_flash_save_pending && g_dir == 0)
    {
        // 检查是否已经停稳超过 10 秒 (FLASH_SAVE_DELAY_MS)
        if (now - g_flash_save_timer >= FLASH_SAVE_DELAY_MS)
        {
            Lifter_SaveHeightToFlash(); // 真正执行 Flash 写入操作

            g_flash_save_pending = 0; // 写入完毕，清除标志位

            // 可以通过串口打印确认一下保存成功
            // Serial_Printf(SERIAL_PORT_2, "[LIFTER] Lazy Save Executed after 10s stop.\r\n");
        }
    }

    // =================================================================
    //  B. 草袋开合状态机 (被动等待接口触发，处理超时)
    // =================================================================
    switch (g_bag_state)
    {
    case BAG_STATE_OPENING:
        // 正在打开 -> 设置状态为 8
        g_app.g_system_report_state = REPORT_DUMPING_START;
        if (now - g_bag_timer >= GRASS_BAG_TIMEOUT_MS)
        {
            GrassBag_Motor_Stop();
            g_bag_state = BAG_STATE_OPEN;
            g_app.grass_bag_status = true; // 报告遥控器：已打开
        }
        break;
    case BAG_STATE_OPEN:
        // 保持打开 -> 维持状态 8
        g_app.g_system_report_state = REPORT_DUMPING_START;
        break;

    case BAG_STATE_CLOSING:
        // 正在关闭 -> 维持状态 8 (或者你可以定义一个正在关闭的状态)
        g_app.g_system_report_state = REPORT_DUMPING_CLOSING;

        if (now - g_bag_timer >= GRASS_BAG_TIMEOUT_MS)
        {
            GrassBag_Motor_Stop();
            g_bag_state = BAG_STATE_CLOSED;
            g_app.grass_bag_status = false; // 报告遥控器：已关闭

            // 关闭完成瞬间 -> 切换到状态 9，并启动计时
            g_app.g_system_report_state = REPORT_DUMPING_DONE;
            g_dump_done_timer = now;
        }
        break;
    case BAG_STATE_CLOSED:
        //  倒草完成状态保持 2 秒
        if (g_app.g_system_report_state == REPORT_DUMPING_DONE)
        {
            if (now - g_dump_done_timer >= 2000) // 2000ms = 2秒
            {
                // 时间到，清除系统状态，弹窗消失
                g_app.g_system_report_state = REPORT_NONE;
            }
        }
        break;
    default:
        break;
    }
}
