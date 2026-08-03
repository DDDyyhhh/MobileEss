#include "odrive_can.h"
#include "chassis_control.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "serial_port.h" // 用于调试打印
#include "cmsis_os.h"
#include "chassis_control.h"
// 全局变量定义
volatile ODrive_Data_t g_odrive_motors[ODRIVE_MAX_MOTORS];

volatile uint32_t g_can_rx_irq_total = 0;
volatile uint32_t g_can_rx_ok = 0;
volatile uint32_t g_can_rx_err = 0;
volatile uint32_t g_can_rx_std = 0;
volatile uint32_t g_can_rx_ext = 0;
volatile uint32_t g_can_rx_rtr = 0;
volatile uint32_t g_can_rx_host_cmd = 0;
volatile uint32_t g_can_rx_host_bad_dlc = 0;
volatile uint32_t g_can_rx_node_oob = 0;
volatile uint32_t g_can_rx_queue_full = 0;
volatile uint8_t g_host_cmd_pending = 0;
volatile uint8_t g_host_cmd_len = 0;
volatile uint8_t g_host_cmd_data[8] = {0};
extern CAN_HandleTypeDef hcan1; // 声明外部 CAN 句柄，实际定义在 main.c 中
extern osMessageQueueId_t Queue_CanTxHandle;
extern osMessageQueueId_t Queue_CanRxHandle;
uint8_t CAN_Enqueue_Message(uint32_t id, uint8_t *data, uint8_t len, uint8_t is_remote)
{
    CAN_TxMsg_t msg;
    msg.stdId = id;
    msg.rtr = is_remote; // 记录帧类型
    
    // 对于 RTR 帧，DLC 可以是你要请求的数据长度，且 data 可以为空
    msg.dlc = (len > 8) ? 8 : len;
    
    if (data != NULL && !is_remote) {
        memcpy(msg.data, data, msg.dlc);
    } else {
        memset(msg.data, 0, 8); // 清零以防有随机值
    }

    uint32_t free_space = osMessageQueueGetSpace(Queue_CanTxHandle);

    // ============================================================
    // QoS 流量清洗机制 (保底设计)
    // ============================================================
    if (free_space < 10) 
    {
        // 队列快满时，丢弃不重要的状态反馈帧 (>= 0x500)
        // 保证电机的控制指令 (0x000 ~ 0x3FF) 能进队列
        if (id >= 0x500) 
        {
            return 0; 
        }
    }

    if (osMessageQueuePut(Queue_CanTxHandle, &msg, 0, 0) == osOK) {
        return 1; 
    }
    
    return 0;
}
// =================================================================
// 1. 底层发送函�?
// =================================================================

// uint8_t odrive_can_send_message(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t length, uint8_t is_remote)
// {
//     CAN_TxHeaderTypeDef tx_header;
//     uint32_t mailbox;

//     tx_header.StdId = id & 0x7FF;
//     tx_header.ExtId = 0;
//     tx_header.IDE = CAN_ID_STD;
//     tx_header.RTR = is_remote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
//     tx_header.DLC = is_remote ? 0 : ((length > 8) ? 8 : length);
//     tx_header.TransmitGlobalTime = DISABLE;

//     //      <<<--- 【核心修改�?邮箱满等待逻辑 ---<<<

//     // 等待直到至少有一个邮箱空�?(超时设为 2ms 足够�?
//     // Non-blocking path: if TX mailbox is full, drop this frame immediately.
//     // {
//         uint32_t start_tick = HAL_GetTick();
//         while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
//         {
//             if ((HAL_GetTick() - start_tick) >= 2U)
//             {
//                 return 0;
//             }
//         }
//     // }

    
//     // ============================================================

//     // 根据帧类型选择数据
//     uint8_t *send_data;
//     if (is_remote)
//     {
//         static uint8_t dummy_data[1] = {0};
//         send_data = dummy_data;
//     }
//     else
//     {
//         send_data = data;
//     }

//     if (HAL_CAN_AddTxMessage(hcan, &tx_header, send_data, &mailbox) != HAL_OK)
//     {
//         return 0;
//     }
//     return 1;
// }
// =================================================================
// 1. 底层发送函? (现在已经彻底蜕变为“入队封装器”)
// =================================================================
uint8_t odrive_can_send_message(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t length, uint8_t is_remote)
{
    // 不管是什么消息，全部无脑塞进队列，绝不在原地等待邮箱！
    return CAN_Enqueue_Message(id, data, length, is_remote);
}

// 便捷封装
/**
 * @brief 发送数据帧的便捷函�?
 * @param hcan: CAN句柄
 * @param id: CAN ID
 * @param data: 数据指针
 * @param length: 数据长度
 * @return 1:成功 0:失败
 */
static uint8_t odrive_send_data(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t len)
{
    return odrive_can_send_message(hcan, id, data, len, 0);
}
/**
 * @brief 发送远程帧（请求数据）的便捷函�?
 * @param hcan: CAN句柄
 * @param id: CAN ID
 * @return 1:成功 0:失败
 */
static uint8_t odrive_send_remote(CAN_HandleTypeDef *hcan, uint32_t id)
{
    return odrive_can_send_message(hcan, id, NULL, 0, 1);
}

// =================================================================
// 2. 控制命令实现
// =================================================================

uint8_t odrive_estop(CAN_HandleTypeDef* hcan, uint8_t node_id) {
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_ESTOP;
    uint8_t data[0] = {};
    return odrive_send_data(hcan, can_id, data, 0);
}

uint8_t odrive_set_axis_node_id(CAN_HandleTypeDef* hcan, uint8_t node_id, uint32_t new_node_id) {
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_SET_AXIS_NODE_ID;
    uint8_t data[4] = {0};
    memcpy(data, &new_node_id, 4);
    return odrive_send_data(hcan, can_id, data, 4);
}

uint8_t odrive_set_axis_state(CAN_HandleTypeDef *hcan, uint8_t node_id, odrive_state_t state)
{
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_SET_AXIS_REQUESTED_STATE;
    uint32_t state_val = (uint32_t)state;
    return odrive_send_data(hcan, can_id, (uint8_t *)&state_val, 4);
}

uint8_t odrive_reboot(CAN_HandleTypeDef* hcan, uint8_t node_id) {
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_REBOOT;
    uint8_t data[0] = {};
    return odrive_send_data(hcan, can_id, data, 0);
}

uint8_t odrive_clear_errors(CAN_HandleTypeDef *hcan, uint8_t node_id)
{
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_CLEAR_ERRORS;
    uint8_t dummy = 0;
    return odrive_send_data(hcan, can_id, &dummy, 0); // 长度0的数据帧
}



// ========== 获取错误信息 ==========

uint8_t odrive_req_motor_error(CAN_HandleTypeDef* hcan, uint8_t node_id) {
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_GET_MOTOR_ERROR;
    return odrive_send_remote(hcan, can_id);
}

uint8_t odrive_req_encoder_error(CAN_HandleTypeDef* hcan, uint8_t node_id) {
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_GET_ENCODER_ERROR;
    return odrive_send_remote(hcan, can_id);
}
uint8_t odrive_req_sensorless_error(CAN_HandleTypeDef* hcan, uint8_t node_id) {
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_GET_SENSORLESS_ERROR;
    return odrive_send_remote(hcan, can_id);
}

// ========== 控制器配�?==========

uint8_t odrive_set_controller_modes(CAN_HandleTypeDef *hcan, uint8_t node_id,
                                    odrive_control_mode_t control_mode,
                                    odrive_input_mode_t input_mode)
{
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_SET_CONTROLLER_MODES;
    uint8_t data[8];
    int32_t c = (int32_t)control_mode;
    int32_t i = (int32_t)input_mode;
    memcpy(data, &c, 4);
    memcpy(data + 4, &i, 4);
    return odrive_send_data(hcan, can_id, data, 8);
}


// ========== 运动控制命令 ==========
uint8_t odrive_set_input_vel(CAN_HandleTypeDef *hcan, uint8_t node_id, float velocity, float torque_ff)
{
    // 1. 计算 CAN ID (NodeID << 5 | CommandID)
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_SET_INPUT_VEL;
    
    // 2. 准备数据负载 (8字节)
    uint8_t data[8];
    memcpy(data, &velocity, 4);      // 速度 (float)
    memcpy(data + 4, &torque_ff, 4);  // 前馈扭矩 (float)

    // 3. 【核心修改】：不再调用 odrive_send_data，直接进入软件队列
    // 这样这个函数会瞬间返回，不会因为 CAN 邮箱满而阻塞
    return CAN_Enqueue_Message(can_id, data, 8, 0);
}

uint8_t odrive_set_input_pos(CAN_HandleTypeDef *hcan, uint8_t node_id, float position, float velocity_ff, float torque_ff)
{
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_SET_INPUT_POS;
    uint8_t data[8];
    memcpy(data, &position, 4);
    int16_t v_ff = (int16_t)(velocity_ff * 1000.0f);
    int16_t t_ff = (int16_t)(torque_ff * 1000.0f);
    memcpy(data + 4, &v_ff, 2);
    memcpy(data + 6, &t_ff, 2);
    return odrive_send_data(hcan, can_id, data, 8);
}

uint8_t odrive_set_input_torque(CAN_HandleTypeDef *hcan, uint8_t node_id, float torque)
{
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_SET_INPUT_TORQUE;
    uint8_t data[4] = {0};
    memcpy(data, &torque, 4);
    return odrive_send_data(hcan, can_id, data, 4);
}


/**
 * @brief Set velocity limit
 */
uint8_t odrive_set_velocity_limit(CAN_HandleTypeDef* hcan, uint8_t node_id, float velocity_limit) {
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_SET_VELOCITY_LIMIT;
    uint8_t data[4] = {0};
    memcpy(data, &velocity_limit, 4);
    return odrive_send_data(hcan, can_id, data, 4);
}

/**
 * @brief Set trajectory velocity limit
 */
uint8_t odrive_set_traj_vel_limit(CAN_HandleTypeDef* hcan, uint8_t node_id, float traj_vel_limit) {
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_SET_TRAJ_VEL_LIMIT;
    uint8_t data[4] = {0};
    memcpy(data, &traj_vel_limit, 4);
    return odrive_send_data(hcan, can_id, data, 4);
}

/**
 * @brief Set trajectory acceleration limits
 */
uint8_t odrive_set_traj_accel_limits(CAN_HandleTypeDef* hcan, uint8_t node_id, 
                                    float accel_limit, float decel_limit) {
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_SET_TRAJ_ACCEL_LIMIT;
    uint8_t data[8] = {0};
    memcpy(data, &accel_limit, 4);
    memcpy(data + 4, &decel_limit, 4);
    
    return odrive_send_data(hcan, can_id, data, 8);
}

/**
 * @brief Set trajectory inertia
 */
uint8_t odrive_set_traj_inertia(CAN_HandleTypeDef* hcan, uint8_t node_id, float inertia) {
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_SET_TRAJ_INERTIA;
    uint8_t data[4] = {0};
    memcpy(data, &inertia, 4);
    return odrive_send_data(hcan, can_id, data, 4);
}

/**
 * @brief Enable anti-cogging
 */
uint8_t odrive_set_anticogging(CAN_HandleTypeDef* hcan, uint8_t node_id) {
    uint32_t can_id = (node_id << 5) | ODRIVE_MSG_SET_ANTICOGGING;
    uint8_t data[0] = {};
    return odrive_send_data(hcan, can_id, data, 0);
}


// 请求数据 (Remote Frame)
uint8_t odrive_req_vbus_voltage(CAN_HandleTypeDef *hcan, uint8_t node_id)
{
    return odrive_send_remote(hcan, (node_id << 5) | ODRIVE_MSG_GET_BUS_VOLTAGE_CURRENT);
}
uint8_t odrive_req_encoder_estimates(CAN_HandleTypeDef *hcan, uint8_t node_id)
{
    return odrive_send_remote(hcan, (node_id << 5) | ODRIVE_MSG_GET_ENCODER_ESTIMATES);
}
uint8_t odrive_req_heartbeat(CAN_HandleTypeDef* hcan, uint8_t node_id) {
    return odrive_send_remote(hcan, (node_id << 5) | ODRIVE_MSG_HEARTBEAT);
}
uint8_t odrive_req_encoder_count(CAN_HandleTypeDef* hcan, uint8_t node_id) {
    return odrive_send_remote(hcan, (node_id << 5) | ODRIVE_MSG_GET_ENCODER_COUNT);
}
uint8_t odrive_req_iq(CAN_HandleTypeDef* hcan, uint8_t node_id) {
    return odrive_send_remote(hcan,  (node_id << 5) | ODRIVE_MSG_GET_IQ);
}
uint8_t odrive_req_sensorless_estimates(CAN_HandleTypeDef* hcan, uint8_t node_id) {
    return odrive_send_remote(hcan, (node_id << 5) | ODRIVE_MSG_GET_SENSORLESS_ESTIMATES);
}
// =================================================================
// 3. 高级功能 (RTOS 适配)
// =================================================================

/**
 * @brief 初始�?ODrive 为闭环控�?(非阻塞式，使�?osDelay)
 * @note  请在任务中调用，不要在中断中调用�?
 */
uint8_t odrive_init_closed_loop(CAN_HandleTypeDef *hcan, uint8_t node_id)
{
    // 1. 清除错误
    odrive_clear_errors(hcan, node_id);
    osDelay(20); // 给一点时间处�?

    // 2. 设为 IDLE (复位状�?
    odrive_set_axis_state(hcan, node_id, ODRIVE_STATE_IDLE);
    osDelay(20);

    // 3. 设置控制模式 (速度控制 + 坡度滤波)
    // 这里的模式要根据你的需求改 (位置/速度/力矩)
    odrive_set_controller_modes(hcan, node_id,
                                ODRIVE_CTRL_MODE_VELOCITY_CONTROL,
                                ODRIVE_INPUT_MODE_VEL_RAMP);
    osDelay(20);

    // 4. 进入闭环
    odrive_set_axis_state(hcan, node_id, ODRIVE_STATE_CLOSED_LOOP_CONTROL);
    osDelay(20);

    return 1;
}

// =================================================================
// 4. CAN 接收中断回调 (核心数据解析)
// =================================================================
static void CAN_Process_Rx_Message(const CAN_RxMsg_t *msg)
{
    if (msg->ide == CAN_ID_STD) {
        g_can_rx_std++;
    } else {
        g_can_rx_ext++;
        return;
    }

    if (msg->rtr != CAN_RTR_DATA) {
        g_can_rx_rtr++;
        return;
    }

    if (msg->stdId == HOST_CMD_FRAME_ID)
    {
        g_can_rx_host_cmd++;
        if (msg->dlc < HOST_CMD_PAYLOAD_LEN)
        {
            g_can_rx_host_bad_dlc++;
            return;
        }

        chassis_control_parse_host_frame(msg->stdId, msg->data, msg->dlc);
        return;
    }

#if MOTOR_DRIVER_USED == DRIVER_TYPE_ODRIVE
    uint8_t node_id = msg->stdId >> 5;
    uint8_t cmd_id = msg->stdId & 0x1F;

    if (node_id >= ODRIVE_MAX_MOTORS)
    {
        g_can_rx_node_oob++;
        return;
    }

    volatile ODrive_Data_t *motor = &g_odrive_motors[node_id];
    motor->is_connected = 1;

    switch (cmd_id)
    {
    case ODRIVE_MSG_HEARTBEAT:
        if (msg->dlc >= 4)
        {
            uint32_t error;
            uint8_t state;
            memcpy(&error, msg->data, 4);
            memcpy(&state, msg->data + 4, 1);
            motor->axis_error = error;
            motor->axis_state = state;
        }
        break;

    case ODRIVE_MSG_GET_ENCODER_ESTIMATES:
        if (msg->dlc >= 8)
        {
            memcpy((void *)&motor->encoder_pos, msg->data, 4);
            memcpy((void *)&motor->encoder_vel, msg->data + 4, 4);
        }
        break;

    case ODRIVE_MSG_GET_BUS_VOLTAGE_CURRENT:
        if (msg->dlc >= 8)
        {
            memcpy((void *)&motor->vbus_voltage, msg->data, 4);
            memcpy((void *)&motor->vbus_current, msg->data + 4, 4);
        }
        break;

    case ODRIVE_MSG_GET_IQ:
        if (msg->dlc >= 8)
        {
            memcpy((void *)&motor->iq_setpoint, msg->data, 4);
            memcpy((void *)&motor->iq_measured, msg->data + 4, 4);
        }
        break;

    case ODRIVE_MSG_GET_MOTOR_ERROR:
        if (msg->dlc >= 4)
            memcpy((void *)&motor->motor_error, msg->data, 4);
        break;

    case ODRIVE_MSG_GET_ENCODER_ERROR:
        if (msg->dlc >= 4)
            memcpy((void *)&motor->encoder_error, msg->data, 4);
        break;
    }
#elif MOTOR_DRIVER_USED == DRIVER_TYPE_MSSD
    MSSD_Parse_Rx_Message(msg->stdId, (uint8_t *)msg->data, msg->dlc);
#elif MOTOR_DRIVER_USED == DRIVER_TYPE_OID
    OID_Parse_Rx_Message(msg->stdId, (uint8_t *)msg->data, msg->dlc);
#endif
}

void StartTask_CANRx(void *argument)
{
    CAN_RxMsg_t msg;

    for (;;)
    {
        if (osMessageQueueGet(Queue_CanRxHandle, &msg, NULL, osWaitForever) == osOK)
        {
            CAN_Process_Rx_Message(&msg);
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t rx_data[8];
    g_can_rx_irq_total++;

    // 获取 CAN 数据
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx_data) != HAL_OK)
    {
        g_can_rx_err++;
        return;
    }
    g_can_rx_ok++;

    CAN_RxMsg_t msg;
    msg.stdId = RxHeader.StdId;
    msg.dlc = RxHeader.DLC;
    msg.rtr = RxHeader.RTR;
    msg.ide = RxHeader.IDE;
    memcpy(msg.data, rx_data, sizeof(msg.data));

    if (osMessageQueuePut(Queue_CanRxHandle, &msg, 0, 0) != osOK)
    {
        g_can_rx_queue_full++;
    }
    return;

#if 0

    // 格式校验
    if (RxHeader.IDE == CAN_ID_STD) {
        g_can_rx_std++;
    } else {
        g_can_rx_ext++;
        return;
    }

    if (RxHeader.RTR != CAN_RTR_DATA) {
        g_can_rx_rtr++;
        return;
    }

    // ============================================================
    // 1. 上位机 (API Host) 命令解析 (各平台通用)
    // ============================================================
    if (RxHeader.StdId == HOST_CMD_FRAME_ID)
    {
        g_can_rx_host_cmd++;
        if (RxHeader.DLC < HOST_CMD_PAYLOAD_LEN)
        {
            g_can_rx_host_bad_dlc++;
            return;
        }

        chassis_control_parse_host_frame(RxHeader.StdId, rx_data, RxHeader.DLC);
        return;
    }

    // ============================================================
    // 2. 底层驱动器反馈解析 (按宏隔离)
    // ============================================================
#if MOTOR_DRIVER_USED == DRIVER_TYPE_ODRIVE

    // ODrive 协议解析 (Node_ID << 5 | CMD_ID)
    uint8_t node_id = RxHeader.StdId >> 5;
    uint8_t cmd_id = RxHeader.StdId & 0x1F;

    if (node_id >= ODRIVE_MAX_MOTORS)
    {
        g_can_rx_node_oob++;
        return;
    }

    volatile ODrive_Data_t *motor = &g_odrive_motors[node_id];
    motor->is_connected = 1;

    switch (cmd_id)
    {
    case ODRIVE_MSG_HEARTBEAT:
        if (RxHeader.DLC >= 4)
        {
            uint32_t error;
            uint8_t state;
            memcpy(&error, rx_data, 4);
            memcpy(&state, rx_data + 4, 1);
            motor->axis_error = error;
            motor->axis_state = state;
        }
        break;

    case ODRIVE_MSG_GET_ENCODER_ESTIMATES:
        if (RxHeader.DLC >= 8)
        {
            memcpy((void *)&motor->encoder_pos, rx_data, 4);
            memcpy((void *)&motor->encoder_vel, rx_data + 4, 4);
        }
        break;

    case ODRIVE_MSG_GET_BUS_VOLTAGE_CURRENT:
        if (RxHeader.DLC >= 8)
        {
            memcpy((void *)&motor->vbus_voltage, rx_data, 4);
            memcpy((void *)&motor->vbus_current, rx_data + 4, 4);
        }
        break;

    case ODRIVE_MSG_GET_IQ:
        if (RxHeader.DLC >= 8)
        {
            memcpy((void *)&motor->iq_setpoint, rx_data, 4);
            memcpy((void *)&motor->iq_measured, rx_data + 4, 4);
        }
        break;

    case ODRIVE_MSG_GET_MOTOR_ERROR:
        if (RxHeader.DLC >= 4)
            memcpy((void *)&motor->motor_error, rx_data, 4);
        break;

    case ODRIVE_MSG_GET_ENCODER_ERROR:
        if (RxHeader.DLC >= 4)
            memcpy((void *)&motor->encoder_error, rx_data, 4);
        break;
    }

#elif MOTOR_DRIVER_USED == DRIVER_TYPE_MSSD

    // 美思固 MSSD 协议解析
    // 美思固的 CAN ID 直接就是 Node ID，因此把 StdId 直接传进去即可
    MSSD_Parse_Rx_Message(RxHeader.StdId, rx_data, RxHeader.DLC);

#endif
#endif
}

// =================================================================
// 5. 初始�?CAN 过滤�?
// =================================================================
void User_CAN_Init(void)
{
    CAN_FilterTypeDef canFilterConfig;

    // 列表模式或掩码模式。为了简单，我们接受所�?ODrive 消息
    // ODrive ID 分布�?0x000 ~ 0x7FF 之间
    canFilterConfig.FilterBank = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

    // Mask 设为 0 表示不关�?ID，接收总线上所有数�?
    canFilterConfig.FilterIdHigh = 0x0000;
    canFilterConfig.FilterIdLow = 0x0000;
    canFilterConfig.FilterMaskIdHigh = 0x0000;
    canFilterConfig.FilterMaskIdLow = 0x0000;

    canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilterConfig.FilterActivation = ENABLE;
    canFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig) != HAL_OK)
    {

        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Serial_Printf(SERIAL_PORT_2, "[CAN] Start Error\r\n");
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Serial_Printf(SERIAL_PORT_2, "[CAN] IT Error\r\n");
        Error_Handler();
    }
}
