/**
 ******************************************************************************
 * @file           : app_Motion.c
 * @author         : [Your Name/Team]
 * @version        : V1.2
 * @date           : 2026-03-11
 * @brief          : 核心运动控制任务，负责整车状态机、姿态解算及底盘/刀盘驱动
 *
 * @note           任务优先级: 高 (High / Real-Time)
 *
 * 主要功能：
 * 1. 严格控制 10ms (100Hz) 的绝对执行周期 (vTaskDelayUntil)
 * 2. 执行最高优先级的硬件安全检测 (碰撞杆、急停开关)
 * 3. 运行 IMU 姿态解算 (100Hz)，判断设备是否处于坡道倾斜状态
 * 4. 降频 (50Hz) 执行运动控制逻辑：读取摇杆输入、处理超声波避障限速
 * 5. 处理软件状态机：通信防失联保护、组合解锁逻辑
 * 6. 控制巡航模式、推杆高度 (Lifter)、割草刀盘转速与底盘速度平滑 (Ramp)
 *
 * 注意事项：
 * - 本任务是系统的“心脏”，耗时操作必须严格控制，切勿在此处使用阻塞型延时。
 ******************************************************************************
 */
#include "app_context.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_imu_solver.h"
#include "serial_port.h"
#include "cruise_control.h"
#include "tricolor_led.h"
#include "lifter.h"
#include "BladeMotor.h"
// #include "car_Control.h"
#include "sr04_uart.h"
#include "connet_check.h"
#include "AXEBMS_RS485.h"
#include "icm42688.h"
#include "E34.h"
#include "A28.h"
#include "pairing_receiver.h"

#define USE_CHASSIS_CONTROL 1 // 1: 使用chassis_control.c, 0: 使用car_Control.c
#if USE_CHASSIS_CONTROL
#include "chassis_control.h"
#else
#include "car_Control.h"
#endif
#include "tricolor_led.h"
// #include "chassis_control.h"
extern AppContext_t g_app;
extern AXEBMS_RS485 my_bms;
extern uint8_t rx_dma_buffer[RX_BUF_SIZE];
extern uint8_t rx_dma_buffer_port1[64];
extern uint8_t rx_dma_buffer_port2[64];

extern uint8_t api_lockout;
extern uint32_t last_host_cmd_tick;
/* ============================================================ */
/*   功能开关配置                                                */
/* ============================================================ */
// 1: 开启极限压力测试模式 (长按 Gear1 原地自转+开刀)

void Check_Unlock_Sequence();
void App_Init(void);
void Modules_Init(void);
void System_Start_Comm(void);
/* USER CODE BEGIN Header_StartTask_Motion */
/**
 * @brief Function implementing the Task_Motion thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_Motion */
void StartTask_Motion(void *argument)
{
  /* USER CODE BEGIN StartTask_Motion */

  // 1. 启动通信接口 (开启中断接收)
  // 此时队列已就绪，中断产生的数据可以安全入队
  System_Start_Comm();

  // 2. 初始化各功能模块
  // 此时可以使用 osDelay，串口打印也能正常发出
  Modules_Init();

  // 3. 初始化应用层参数
  App_Init();

  // 4. 打印初始化完成
  Serial_Printf(SERIAL_PORT_2, "[INIT] System Ready. Entering Control Loop.\r\n");

  // 1. 初始化时序变量
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms 周期

  // IMU 时间戳
  g_app.last_imu_tick = HAL_GetTick();

  // 分频计数器
  uint8_t control_divider = 0;

  // 按键状态记录 (用于边沿检测)
  static bool last_key_state[8] = {0};

  // 旋钮状态记录
  static uint8_t last_blade_knob = 0xFF;
  static uint8_t last_height_knob = 0xFF;

#if ENABLE_STRESS_TEST_MODE
  // ==========================================
  // <<<--- 测试自转模式标志位 --->>>
  // ==========================================
  static bool test_spin_mode = false;
  static uint8_t test_mode_key_lock = 0;

#endif
  // 定义静态变量防止重复触发
  static uint8_t us_toggle_lock = 0;

  static uint8_t cruise_key_lock = 0;
  // 【新增】 巡航切换冷却计时器
  static uint32_t cruise_toggle_tick = 0;
  /* Infinite loop */
  for (;;)
  {
    // Serial_Printf(SERIAL_PORT_2, "Ctrl_Mode=%u\r\n", Ctrl_Mode);
    // Control_Update();
    // Serial_Printf(SERIAL_PORT_2, "knob_gear=%d\r\n", g_app.knob_gear);
    // Serial_Printf(SERIAL_PORT_2, "user_set_speed: %f\r\n", g_app.user_set_speed);
    // === 1. 绝对延时 (确保严格 10ms 周期) ===
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

#if MOTOR_DRIVER_USED == DRIVER_TYPE_OID
    {
      static uint32_t last_oid_heartbeat = 0;
      if (HAL_GetTick() - last_oid_heartbeat >= 100)
      {
        last_oid_heartbeat = HAL_GetTick();
        OID_Heartbeat(OID_NODE_FL);
        OID_Heartbeat(OID_NODE_FR);
        OID_Heartbeat(OID_NODE_RL);
        OID_Heartbeat(OID_NODE_RR);
      }
    }
#endif

    // === 2. 安全检测 (最高优先级) ===
    if (g_app.g_collision_latched == 0)
    {
      // A. 碰撞杆检测 (PD0)
      if (g_app.cfg_collision_en == 1)
      {
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
          g_app.g_collision_latched = 1;
          g_app.g_system_report_state = REPORT_FAULT_COLLISION;

#if USE_CHASSIS_CONTROL
          chassis_stop();
#else
          MotionControl_Parking(g_app.is_on_slope);
#endif
          BladeMotor_SetSpeed(BLADE_SPEED_OFF);
        }
      }

      // B. 急停按钮检测 (PD1)
      if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_RESET)
      {
        g_app.g_collision_latched = 1;
        g_app.g_system_report_state = REPORT_FAULT_EMERGENCY_BTN;

#if USE_CHASSIS_CONTROL
        chassis_stop();
#else
        MotionControl_Parking(g_app.is_on_slope);
#endif
        BladeMotor_SetSpeed(BLADE_SPEED_OFF);
      }

      // ============================================================
      // <<<---  C.  倾翻/侧翻保护检测 (100Hz 实时检测) --->>>
      // ============================================================
      if (g_app.cfg_tilt_sensor_en == 1)
      {
        // 当 Pitch(前后坡度) 或 Roll(侧倾角度) 的绝对值超过 60 度时触发
        if (fabsf(imu_data.pitch) > 15.0f || fabsf(imu_data.roll) > 40.0f)
        {
          g_app.g_collision_latched = 1;                   // 触发安全锁存，锁定底盘
          g_app.g_system_report_state = REPORT_FAULT_TILT; // 报告侧翻故障 (状态码 3)

#if USE_CHASSIS_CONTROL
          chassis_stop(); // 紧急切断底盘动力
#else
          MotionControl_Parking(g_app.is_on_slope);
#endif
          BladeMotor_SetSpeed(BLADE_SPEED_OFF); // 紧急切断割刀动力 (瞬时关刀)

          Serial_Printf(SERIAL_PORT_2, "[SAFETY] Rollover Detected! Pitch:%.1f, Roll:%.1f. System Latched!\r\n",
                        imu_data.pitch, imu_data.roll);
        }
      }
    }
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == 0)
    {
      // 重新开始配对模式
      pairing_receiver_init();
      osDelay(100);
      uint32_t start_tick = HAL_GetTick();
      Serial_Printf(SERIAL_PORT_2, "Start Pairing Receiver...\r\n");

      // 3. 进入等待循环 (条件：未成功 且 未超时10秒)
      while (pairing_receiver_get_state() != PAIRING_SUCCESS)
      {
        g_app.g_system_report_state = REPORT_FAULT_COMM_ING;
        pairing_receiver_task();
        osDelay(10);
        Serial_Printf(SERIAL_PORT_2, "State: %d\r\n", pairing_receiver_get_state());
        // 必须有延时，让 StartTask_Comm 接收数据
        osDelay(10);

        // 超时判断 (10秒 = 10000ms)
        if (HAL_GetTick() - start_tick > 10000)
        {
          pairing_receiver_init();
          osDelay(100);
          start_tick = HAL_GetTick();
        }
      }

      if (pairing_receiver_get_state() == PAIRING_SUCCESS)
      {
        g_app.g_system_report_state = REPORT_FAULT_COMM_SUCCESS;
        Serial_Printf(SERIAL_PORT_2, "[Auto-Pair] Success!\r\n");
      }
    }

    // === 3. IMU 姿态解算 (100Hz) ===
    uint32_t now_tick = HAL_GetTick();
    float dt = (float)(now_tick - g_app.last_imu_tick) / 1000.0f;
    g_app.last_imu_tick = now_tick;

    if (dt < 0.001f)
      dt = 0.001f;
    if (dt > 0.1f)
      dt = 0.01f;

    icm42688RealData_t acc, gyro;
    bsp_IcmGetRawData(&acc, &gyro);
    IMU_Solver_Update(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, dt);

    if (fabsf(imu_data.roll) > 4.0f || fabsf(imu_data.pitch) > 4.0f)
      g_app.is_on_slope = 1;
    else
      g_app.is_on_slope = 0;

    // === 4. 运动控制逻辑 (50Hz - 每 50ms 执行一次) ===
    control_divider++;
    if (control_divider >= 5)
    {
      control_divider = 0;

      // ============================================================
      //      <<<--- A. 读取输入与操作模式通道映射 --->>>
      // ============================================================
      // 1. 获取四个物理通道的原始 ADC 数据
      uint16_t raw_lx = g_app.Joystick[0]; // 左摇杆 X 轴
      uint16_t raw_ly = g_app.Joystick[1]; // 左摇杆 Y 轴
      uint16_t raw_rx = g_app.Joystick[2]; // 右摇杆 X 轴
      uint16_t raw_ry = g_app.Joystick[3]; // 右摇杆 Y 轴

      // 2. 定义映射后的抽象控制量 (默认中位 2048)
      uint16_t mapped_throttle = 2048; // 负责前进/后退
      uint16_t mapped_steering = 2048; // 负责左转/右转

      // 3. 根据遥控器下发的操作模式进行通道路由
      switch (g_app.cfg_op_mode)
      {
      case 0: // 双手1 (默认): 左手控前后(LY)，右手控左右(RX)
        mapped_throttle = raw_ly;
        mapped_steering = raw_rx;
        break;
      case 1: // 双手2: 左手控左右(LX)，右手控前后(RY)
        mapped_throttle = raw_ry;
        mapped_steering = raw_lx;
        break;
      case 2: // 单手1: 左手全包，控前后(LY)和左右(LX)
        mapped_throttle = raw_ly;
        mapped_steering = raw_lx;
        break;
      case 3: // 单手2: 右手全包，控前后(RY)和左右(RX)
        mapped_throttle = raw_ry;
        mapped_steering = raw_rx;
        break;
      default:
        mapped_throttle = raw_ly;
        mapped_steering = raw_rx;
        break;
      }

      // 4. 计算摇杆是否被拨动 (用于打断测试模式等)
      // 注意：现在只判断“有效映射”后的摇杆，没用到的摇杆乱动也不会误触发
      bool is_joystick_active = (abs((int)mapped_throttle - 2048) > 200) ||
                                (abs((int)mapped_steering - 2048) > 200);

#if ENABLE_STRESS_TEST_MODE
      // ============================================================
      //      <<<--- 【测试模式】 摇杆行为检测 (防抖死区 200) --->>>
      // ============================================================
      // 如果摇杆偏离中心 2048 超过 200，说明用户动手了
      bool is_joystick_active = (abs((int)local_ly - 2048) > 200) ||
                                (abs((int)local_rx - 2048) > 200);

      // ============================================================
      //      <<<--- 【测试模式】 一键自转测试模式状态机 --->>>
      // ============================================================
      // 只有在解锁状态下，才允许进入测试模式
      if (g_app.g_collision_latched == 0)
      {
        // 1. 检测长按进入/退出
        if (g_app.key[KEY_INDEX_GEAR_2] == 2)
        {
          if (test_mode_key_lock == 0)
          {
            test_mode_key_lock = 1; // 锁住防抖

            if (!test_spin_mode && !is_joystick_active)
            {
              test_spin_mode = true;
              Serial_Printf(SERIAL_PORT_2, "[TEST MODE] ON: Spinning + Blade L1\r\n");
            }
            else
            {
              test_spin_mode = false;
              Serial_Printf(SERIAL_PORT_2, "[TEST MODE] OFF: Exited by Key\r\n");
            }
          }
        }
        else
        {
          test_mode_key_lock = 0; // 松手复位
        }

        // 2. 摇杆打断机制：测试中只要碰到摇杆，立即退出！
        if (test_spin_mode && is_joystick_active)
        {
          test_spin_mode = false;
          Serial_Printf(SERIAL_PORT_2, "[TEST MODE] OFF: Interrupted by Joystick\r\n");
        }
      }
      else
      {
        // 一旦上锁/急停/通信丢失，无条件切断测试模式
        test_spin_mode = false;
      }
#endif
      //      <<<--- B. 更新超声波数据 (集成遥控器开关拦截) --->>>
      // ============================================================
      // 【核心修改】：使用从遥控器发来的配置标志 cfg_ultrasonic_en
      if (g_app.cfg_ultrasonic_en == 1)
      {
        // 开关打开时，读取真实的传感器距离
        g_app.d1 = SR04_GetDistance(0);
        g_app.d2 = SR04_GetDistance(1);
        g_app.d3 = SR04_GetDistance(2);
        g_app.d4 = SR04_GetDistance(3);
      }
      else
      {
        // 开关关闭时，强制把所有距离覆写为 250.0f
        // 这样不仅底层不会触发避障，通过 receiver_resport 发给遥控器的数据也是 250，
        // 遥控器端的 UI 发现距离 > 200，就会自动把雷达波纹隐藏掉！
        g_app.d1 = 250.0f;
        g_app.d2 = 250.0f;
        g_app.d3 = 250.0f;
        g_app.d4 = 250.0f;
      }

      // ============================================================
      //      <<<--- 【智能版】 草袋满载检测与清理状态机 --->>>
      // ============================================================
      static uint8_t bag_fsm_state = 0;
      static uint32_t grass_present_tick = 0;
      static uint32_t grass_absent_tick = 0;

      // 🚨 新增：用于记录物理推杆开始关门动作的绝对时间戳
      static uint32_t bag_close_start_tick = 0;

// 💡 架构师提示：请根据你物理推杆的实际关门耗时（秒）调整此参数。
// 如果推杆 8 秒关完，设为 10000 即可；如果 12 秒关完，设为 14000。给机械留出 2 秒余量。
#define BAG_CLOSE_BLANKING_MS 14000 // 12秒关门机械屏蔽期

      bool is_grass_detected = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == GPIO_PIN_SET);
      bool is_grass_connected = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) == GPIO_PIN_RESET);

      // 无论发生什么报警，绝对真实地记录物理连接状态
      g_app.is_bag_connected = is_grass_connected ? 1 : 0;

      // 2. 独立的防抖定时器 (双向防抖)
      if (is_grass_detected)
      {
        grass_absent_tick = 0; // 没草计时器清零
        if (grass_present_tick == 0)
          grass_present_tick = HAL_GetTick();
      }
      else
      {
        grass_present_tick = 0; // 有草计时器清零
        if (grass_absent_tick == 0)
          grass_absent_tick = HAL_GetTick();
      }

      // 计算是否达到 10 秒
      bool is_10s_grass = (grass_present_tick != 0 && (HAL_GetTick() - grass_present_tick >= 10000));
      bool is_10s_clear = (grass_absent_tick != 0 && (HAL_GetTick() - grass_absent_tick >= 10000));

      // 3. 清理流程状态机与连接检测
      if (is_grass_connected)
      {
        switch (bag_fsm_state)
        {
        case 0: // 【正常监控】
          if (is_10s_grass)
          {
            bag_fsm_state = 1; // 满载，触发报警
          }
          break;

        case 1: // 【满载报警中】
          if (g_app.g_collision_latched == 0 &&
              g_app.g_system_report_state < REPORT_DUMPING_START)
          {
            // g_app.g_system_report_state = REPORT_FAULT_BAG_FULL;
          }

          if (g_app.grass_bag_status == true)
          {
            bag_fsm_state = 2; // 用户确认开盖倒草
            grass_present_tick = 0;
            grass_absent_tick = 0;

            if (g_app.g_system_report_state == REPORT_FAULT_BAG_FULL)
            {
              g_app.g_system_report_state = REPORT_NONE;
            }
          }
          break;

        case 2: // 【倒草开盖中】
          if (g_app.grass_bag_status == false)
          {
            bag_fsm_state = 3; // 收到关盖指令，进入验证期
            grass_present_tick = 0;
            grass_absent_tick = 0;

            // 🚨【关键修复】关门瞬间，记录下当前绝对时间，开始启动机械屏蔽保护
            bag_close_start_tick = HAL_GetTick();
          }
          break;

        case 3: // 【关闭后验证阶段】
          if (is_10s_clear)
          {
            bag_fsm_state = 0; // 连续 10 秒无草，安全返回正常监控
          }
          else if (is_10s_grass)
          {
            // 🚨【核心安全屏蔽阻断】
            // 只有当推杆关门动作彻底结束（超过 12 秒屏蔽期）之后，
            // 传感器依然检测到草，才被允许重新判定为满载。
            // 这样完美避免了推杆缓慢运动、门板遮挡、机械震动瞬态电平引发的假满载误判！
            if (HAL_GetTick() - bag_close_start_tick >= BAG_CLOSE_BLANKING_MS)
            {
              bag_fsm_state = 1;
            }
          }
          break;
        }

        g_app.is_bag_full = (bag_fsm_state == 1) ? 1 : 0;
      }
      else
      {
        bag_fsm_state = 0;
        grass_present_tick = 0;
        g_app.is_bag_full = 0;
      }
      // --- C. 状态判断与执行 ---

      // 独立计算两路心跳
      bool rc_online = CheckCommunicationStatus();
      bool api_online = (HAL_GetTick() - last_host_cmd_tick < API_TIMEOUT_MS);

      // 优先级 1.遥控器和上位机都没在线！
      if (!rc_online && !api_online)
      {
        g_app.is_on_slope = 1;
        g_app.g_system_report_state = REPORT_FAULT_COMM_LOST;
        BladeMotor_SetSpeed(BLADE_SPEED_OFF);
        Cruise_Stop();
        g_app.g_collision_latched = 1;
        if (g_app.lock != 1)
        {
          Serial_Printf(SERIAL_PORT_2, "[LOCK] reason=comm_lost -> lock=1\r\n");
        }
        g_app.lock = 1; // 通信丢失自动上锁

        // ============================================================
        // 强制同步 UI 状态，防止遥控器图标卡死
        // ============================================================
        g_app.receiver_status[STATUS_INDEX_LOCK] = 0;        // 告诉遥控器已上锁
        g_app.receiver_status[STATUS_INDEX_BLADE_SPEED] = 0; // 告诉遥控器刀盘已停
      }
      // 优先级 2: 物理故障 (锁存状态)
      else if (g_app.g_collision_latched)
      {
        g_app.is_on_slope = 1;
        BladeMotor_SetSpeed(BLADE_SPEED_OFF);
        Cruise_Stop();

        if (g_app.lock != 1)
        {
          Serial_Printf(SERIAL_PORT_2, "[LOCK] reason=fault_latched -> lock=1\r\n");
        }
        g_app.lock = 1; // 故障自动上锁

        // 只有遥控器能通过摇杆解开物理锁；API 模式则通过发 0xAA 密匙解开
        if (Ctrl_Mode == 0)
        {
          Check_Unlock_Sequence();
        }
        // ============================================================
        // 强制同步 UI 状态，防止遥控器图标卡死
        // ============================================================
        g_app.receiver_status[STATUS_INDEX_LOCK] = 0;        // 告诉遥控器已上锁
        g_app.receiver_status[STATUS_INDEX_BLADE_SPEED] = 0; // 告诉遥控器刀盘已停
      }
      // 优先级 3: 正常运行 (至少有一方在线，且未撞车)
      else
      {
        if (g_app.g_system_report_state == REPORT_FAULT_COMM_LOST)
          g_app.g_system_report_state = REPORT_NONE;

        // 【预留给遥控器屏幕】以后你修改遥控器LVGL代码时，可以发一个命令清零
        // 比如：如果遥控器发来一个标志位 g_app.cfg_api_enable == 1
        // if (g_app.cfg_api_enable == 1) { api_lockout = 0; }

        // ============================================================
        //      <<<--- 软件锁 (Lock) 逻辑 ---<<<(仅限遥控模式，API由上位机接管)
        // ============================================================
        // 双键解锁/上锁
        if (Ctrl_Mode == 0)
        {

          // 2. 判断按键状态 (两个解锁键是否都按下了)
          bool is_auto_mode = (Cruise_GetState() != CRUISE_OFF);
#if ENABLE_STRESS_TEST_MODE
          if (test_spin_mode)
            is_auto_mode = true; // 测试模式也算自动状态
#endif
          bool keys_pressed = g_app.key[KEY_INDEX_UNLOCK_A] && g_app.key[KEY_INDEX_UNLOCK_B];
          bool keys_released = !g_app.key[KEY_INDEX_UNLOCK_A] && !g_app.key[KEY_INDEX_UNLOCK_B];
          uint8_t unlock_confirm = (g_app.key[KEY_INDEX_BLADE_SPEED_3] == 1) ? 1 : 0;
          uint8_t unlock_hit = (keys_pressed && g_app.lock == 1 && unlock_confirm) ? 1 : 0;
          static uint8_t last_unlock_trace = 0xFF;
          uint8_t unlock_trace = (keys_pressed ? 0x01 : 0x00) |
                                 (unlock_confirm ? 0x02 : 0x00) |
                                 (unlock_hit ? 0x04 : 0x00);
          if (unlock_trace != last_unlock_trace)
          {
            last_unlock_trace = unlock_trace;
            // Serial_Printf(SERIAL_PORT_2,
            //               "[UNLOCK] keys=%u confirm=%u hit=%u k0=%u k1=%u k7=%u lock=%u\r\n",
            //               keys_pressed ? 1 : 0, unlock_confirm, unlock_hit,
            //               g_app.key[KEY_INDEX_UNLOCK_A], g_app.key[KEY_INDEX_UNLOCK_B],
            //               g_app.key[KEY_INDEX_BLADE_SPEED_3], g_app.lock);
          }

          if (keys_pressed)
          {

            if (g_app.lock == 1)
            {
              // 开锁提示弹窗 0关闭 1开启
              g_app.Unlock_tips = 1;
              // 按下确认键 (Gear 3) 执行解锁
              if (g_app.key[KEY_INDEX_BLADE_SPEED_3] == 1)
              {
                g_app.Unlock_tips = 0;
                g_app.lock = 0;                               // 解锁
                g_app.receiver_status[STATUS_INDEX_LOCK] = 1; // 反馈给遥控器
              }
            }
          }
          else if (keys_released && is_auto_mode == false)
          {
            g_app.Unlock_tips = 0;
            if (g_app.lock == 0)
            {

              g_app.lock = 1; // 上锁
              g_app.receiver_status[STATUS_INDEX_LOCK] = 0;
              g_app.receiver_status[STATUS_INDEX_BLADE_SPEED] = 0;
              BladeMotor_SetSpeed(BLADE_SPEED_OFF);
            }
          }
          else
            g_app.Unlock_tips = 0;
        }
        // ============================================================
        //      <<<--- 巡航控制触发逻辑 ---<<<
        // ============================================================

        // 2. 处理巡航请求触发
        if (g_app.trigger_cruise_req == 1)
        {
          // 二次安全校验：确保没撞车、没急停、没报错
          if (g_app.g_collision_latched == 0 && CheckCommunicationStatus())
          {
            Cruise_Start(); // 启动巡航

            // Serial_Printf(SERIAL_PORT_2, "[Motion] Executed Cruise Start from Context\r\n");
          }
          g_app.trigger_cruise_req = 0; // 【关键】不论是否成功开启，都清零
        }

        // ============================================================
        //      <<<--- 按键处理 (档位与刀盘) ---<<<
        // ============================================================

        switch (g_app.knob_gear)
        {
        case 1:
          g_app.user_set_speed = MOWER_GEAR_1_MAX_SPEED;
          g_app.turn_speed = MOWER_GEAR_1_MAX_TURN_SPEED;
          break;
        case 2:
          g_app.user_set_speed = MOWER_GEAR_2_MAX_SPEED;
          g_app.turn_speed = MOWER_GEAR_2_MAX_TURN_SPEED;
          break;
        case 3:
          g_app.user_set_speed = MOWER_GEAR_3_MAX_SPEED;
          g_app.turn_speed = MOWER_GEAR_3_MAX_TURN_SPEED;
          break;
        default:
          g_app.user_set_speed = MOWER_GEAR_1_MAX_SPEED;
          g_app.turn_speed = MOWER_GEAR_1_MAX_TURN_SPEED;
          break; // 默认或者0档
        }
        // 更新状态回传
        g_app.receiver_status[STATUS_INDEX_GEAR] = g_app.knob_gear;

        // 2. 处理割刀 (Blade) -> 必须解锁才生效
        if (Ctrl_Mode == 2 || (Ctrl_Mode == 0 && g_app.lock == 0)) // 已解锁
        {
          // 只有当旋钮位置发生变化时，才下发指令 (防止一直发CAN/PWM指令占用总线)
          // 或者你的 BladeMotor_SetSpeed 内部有判断也可以

          switch (g_app.knob_blade)
          {
          case 1:
            g_app.receiver_status[STATUS_INDEX_BLADE_SPEED] = 1;
            BladeMotor_SetSpeed(BLADE_SPEED_LEVEL_1);
            break;
          case 2:
            g_app.receiver_status[STATUS_INDEX_BLADE_SPEED] = 2;
            BladeMotor_SetSpeed(BLADE_SPEED_LEVEL_2);
            break;
          case 3:
            g_app.receiver_status[STATUS_INDEX_BLADE_SPEED] = 3;
            BladeMotor_SetSpeed(BLADE_SPEED_LEVEL_3);
            break;
          default:
            BladeMotor_SetSpeed(0.0f);
            break; // 0档关闭
          }
        }
        else
        {
          // RC 模式且未解锁，强制关刀
          BladeMotor_SetSpeed(BLADE_SPEED_OFF);
        }
        // ============================================================
        //      <<<--- 草袋开合切换逻辑 (新增) ---<<<
        // ============================================================
        // 静态变量用于边沿检测，防止长按导致反复翻转
        static uint8_t bag_toggle_lock = 0;

        if (g_app.key[KEY_INDEX_GEAR_1] == 1 && is_grass_connected == 1) // 假设返回1表示按键按下
        {
          if (bag_toggle_lock == 0)
          {
            bag_toggle_lock = 1; // 上锁

            // 触发底层草袋翻转动作
            Lifter_ToggleGrassBag();

            // 可选：打印提示音或日志
            // Serial_Printf(SERIAL_PORT_2, "[App] Toggle Grass Bag Triggered.\r\n");
          }
        }
        else
        {
          bag_toggle_lock = 0; // 松开后解锁，为下一次触发做准备
        }

        // 1. 处理刀盘初始化触发
        if (g_app.trigger_blade_init == 1)
        {
          Lifter_User_Init();           // 执行归位动作
          g_app.trigger_blade_init = 0; // 【关键】执行完立即清零，等待下次脉冲
                                        // Serial_Printf(SERIAL_PORT_2, "[Motion] Executed Blade Init from Context\r\n");
        }
        // ============================================================
        //      <<<---  推杆高度控制 (旋钮位置控制版) ---<<<
        // ============================================================
        // 只有当旋钮数值发生变化时，才更新目标高度
        if (g_app.knob_height != last_height_knob)
        {
          Lifter_SetTargetLevel(g_app.knob_height);
          last_height_knob = g_app.knob_height;
        }

        // 更新按键历史状态
        for (int i = 0; i < 8; i++)
          last_key_state[i] = g_app.key[i];

        // ============================================================
        //      <<<--- 超声波避障计算 ---<<<
        // ============================================================
        float target_speed = g_app.user_set_speed;

        // 只有在使能开启时，才执行避障逻辑
        if (g_app.cfg_ultrasonic_en == 1)
        {
          // 找出 4 个探头中的最小距离
          float front_min = (g_app.d2 < g_app.d3) ? g_app.d2 : g_app.d3;
          float side_min = (g_app.d1 < g_app.d4) ? g_app.d1 : g_app.d4;
          float min_dist = (front_min < side_min) ? front_min : side_min;

          float us_speed_limit = target_speed; // 默认不限速

          // --- 阶梯式限速逻辑 ---
          if (min_dist <= 50.0f)
          {
            us_speed_limit = MOWER_UC_1_MAX_SPEED; // 极近距离：强制蠕行
          }
          else if (min_dist <= 100.0f)
          {
            us_speed_limit = MOWER_UC_2_MAX_SPEED; // 逼近距离：中速限制
          }
          else if (min_dist <= 150.0f)
          {
            us_speed_limit = MOWER_UC_3_MAX_SPEED; // 预警距离：高速限制
          }

          // 如果当前用户的推杆目标速度大于雷达允许的上限，则强行压低
          if (target_speed > us_speed_limit)
          {
            target_speed = us_speed_limit;
          }
        }

        // ============================================================
        //      <<<---  速度平滑 (Ramp) ---<<<
        // ============================================================
        // 直接对 target_speed 进行平滑追踪即可
        if (g_app.current_ramped_speed < target_speed)
        {
          g_app.current_ramped_speed += SPEED_RAMP_UP_STEP;
          if (g_app.current_ramped_speed > target_speed)
            g_app.current_ramped_speed = target_speed;
        }
        else if (g_app.current_ramped_speed > target_speed)
        {
          g_app.current_ramped_speed -= SPEED_RAMP_DOWN_STEP;
          if (g_app.current_ramped_speed < target_speed)
            g_app.current_ramped_speed = target_speed;
        }

        // 更新全局 speed 变量，供 MotionControl_Update 使用
        g_app.speed = g_app.current_ramped_speed;
      }

      // ============================================================
      //      <<<--- B. 执行控制 (使用映射后的变量) ---<<<
      // ============================================================

      if (Cruise_GetState() != CRUISE_OFF)
      {
        // 【修改】将映射后的摇杆值传给巡航模块
        Cruise_Process(mapped_throttle, mapped_steering);
      }
      else
      {
        // 【修改】将映射后的摇杆值传给手动底盘控制模块
#if USE_CHASSIS_CONTROL
        Control_Update();

#else
        MotionControl_Update(mapped_throttle, mapped_steering);
#endif
      }
    }
  }
  /* USER CODE END StartTask_Motion */
}

// 辅助函数：启动通信接口
void System_Start_Comm(void)
{
  // 1. 打印启动信息 (此时队列已创建，可以安全打印)
  Serial_Printf(SERIAL_PORT_2, "\r\n--- System Boot (RTOS Running) ---\r\n");

  // 2. 启动无线模块接收 (DMA + IDLE)
  // 必须在任务里启动，因为一旦启动，中断回调就会尝试往队列塞数据
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_dma_buffer, RX_BUF_SIZE);
  RF_Module_Set_Trans_Mode();

  // 启动 ODrive 串口的 DMA 接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buffer_port1, 64);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buffer_port2, 64);

  // 5. 启用 DWT 计数器 (用于微秒延时，如果有用到)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// 辅助函数：应用初始化
void App_Init(void)
{
  Serial_Printf(SERIAL_PORT_2, "[INIT] Application initialization...\r\n");

  // 3. 启动BMS处理逻辑
  AXEBMS_RS485_Start(&my_bms);

  // 设置刀盘电机初始速度为0
  BladeMotor_SetSpeed(0.0f);
}

// 辅助函数：模块初始化
void Modules_Init(void)
{
  Serial_Printf(SERIAL_PORT_2, "[INIT] Modules initialization...\r\n");

  // 1. 电机初始化
  BladeMotor_Start();

  // 2. 无线模块配置
  A28_Set_FullDuplex_Mode();

  // 4. IMU 初始化与校准
  if (bsp_Icm42688Init() == 0)
  {
    Serial_Printf(SERIAL_PORT_2, "IMU Init Success\n");
  }
  else
  {
    Serial_Printf(SERIAL_PORT_2, "IMU Init Failed\n");
  }

  // 【关键】在任务中使用 osDelay 替代 HAL_Delay
  // osDelay 会挂起当前任务，让出 CPU 给其他任务（如串口发送任务），
  // 这样你的 Serial_Printf 才能发出去！
  osDelay(100);

  // ... IMU 校准逻辑 ...
  Serial_Printf(SERIAL_PORT_2, "[IMU] Calibrating... DO NOT MOVE!\r\n");

  // 亮黄灯提示校准中
  RGB_LED_SetColor(LED_1, RGB_COLOR_YELLOW);
  RGB_LED_SetColor(LED_1, RGB_COLOR_YELLOW);

  float gx_sum = 0.0f;
  float gy_sum = 0.0f;
  float gz_sum = 0.0f;
  int sample_count = 200; // 采样200次 (200 * 5ms = 1秒)

  for (int i = 0; i < sample_count; i++)
  {
    bsp_IcmGetRawData(&g_app.accval, &g_app.gyroval);

    gx_sum += g_app.gyroval.x;
    gy_sum += g_app.gyroval.y;
    gz_sum += g_app.gyroval.z;

    osDelay(5); // 稍微延时，覆盖一段时间
  }

  // 计算平均值并应用
  IMU_Solver_SetOffset(gx_sum / sample_count,
                       gy_sum / sample_count,
                       gz_sum / sample_count);

  Serial_Printf(SERIAL_PORT_2, "[IMU] Done. Offset X:%.2f Y:%.2f Z:%.2f\r\n",
                imu_data.gyro_offset_x,
                imu_data.gyro_offset_y,
                imu_data.gyro_offset_z);

  // 恢复红灯
  RGB_LED_SetColor(LED_1, RGB_COLOR_RED);
  RGB_LED_SetColor(LED_1, RGB_COLOR_RED);
  /* ============================================================ */

  bsp_IcmGetRawData(&g_app.accval, &g_app.gyroval);
  IMU_Solver_Init(g_app.accval.x, g_app.accval.y, g_app.accval.z); // 用加速度立即计算当前倾角

  // 记录当前时间
  g_app.last_imu_tick = HAL_GetTick();
  // ...

  // 5. 其他模块

  Lifter_Init();
  AXEBMS_RS485_Init(&my_bms, 0x01);
  RGB_LED_Init();
  RGB_LED_SetColor(LED_1, RGB_COLOR_RED);
  RGB_LED_SetColor(LED_2, RGB_COLOR_RED);
  Cruise_Init();
}

// 辅助函数：解锁逻辑封装
void Check_Unlock_Sequence()
{
// 1. 定义阈值参数
// 触发阈值：摇杆推到极限 (接近0)
#define JOYSTICK_TRIGGER_LIMIT 3000
// 回中阈值：摇杆回到中间范围 (12位ADC中间是2048，设置一个宽泛的中间带)
#define JOYSTICK_CENTER_MIN 1500
#define JOYSTICK_CENTER_MAX 2600

  // 2. 获取当前按键和摇杆状态
  bool key1_pressed = g_app.key[KEY_INDEX_UNLOCK_A];
  bool key2_pressed = g_app.key[KEY_INDEX_UNLOCK_B];

  // 判断摇杆是否推到了极限位置 (大于3800)
  bool ly_at_limit = g_app.Joystick[JOYSTICK_INDEX_THROTTLE] > JOYSTICK_TRIGGER_LIMIT;
  bool ry_at_limit = g_app.Joystick[JOYSTICK_INDEX_RIGHT_Y] > JOYSTICK_TRIGGER_LIMIT;
  // Serial_Printf(SERIAL_PORT_2, "%d,%d,%d,%d\r\n", Joystick[JOYSTICK_INDEX_THROTTLE], Joystick[JOYSTICK_INDEX_RIGHT_Y],key1_pressed, key2_pressed);

  // 3. 判断是否满足【第一阶段】：双键按下 + 双摇杆推到底
  if (key1_pressed && key2_pressed && ly_at_limit && ry_at_limit)
  {
    // 如果第一阶段尚未完成，现在标记为完成
    if (!g_app.unlock_stage1_complete)
    {
      g_app.unlock_stage1_complete = true;
      // Serial_Printf(SERIAL_PORT_2, "[UNLOCK] Stage 1 OK. Please release joysticks to center.\r\n");

      // 亮黄灯提示 (等待回中)
      // RGB_LED_SetColor(LED_1, RGB_COLOR_YELLOW);
      // RGB_LED_SetColor(LED_2, RGB_COLOR_YELLOW);
    }
  }
  // 4. 判断是否满足【第二阶段】：已完成第一阶段 + 摇杆回到中间
  else if (g_app.unlock_stage1_complete)
  {
    // 获取当前摇杆数值
    uint16_t current_ly = g_app.Joystick[JOYSTICK_INDEX_THROTTLE];
    uint16_t current_ry = g_app.Joystick[JOYSTICK_INDEX_RIGHT_Y];
    // 检查摇杆是否真的回到了中间区域 (防止在边缘抖动就触发解锁)
    bool ly_centered = (current_ly > JOYSTICK_CENTER_MIN) && (current_ly < JOYSTICK_CENTER_MAX);
    bool ry_centered = (current_ry > JOYSTICK_CENTER_MIN) && (current_ry < JOYSTICK_CENTER_MAX);

    if (ly_centered && ry_centered)
    {
      g_app.g_collision_latched = 0; // 解除锁存
      g_app.g_system_report_state = REPORT_NONE;
      g_app.unlock_stage1_complete = false; // 重置标志位

      // --- 解锁后的一次性动作 ---
      // Serial_Printf(SERIAL_PORT_2, "[INFO] Collision lock RESET! System Active.\r\n");

      // 恢复绿灯，表示正常
      RGB_LED_SetColor(LED_1, RGB_COLOR_GREEN);
      RGB_LED_SetColor(LED_2, RGB_COLOR_GREEN);
    }
  }
}
