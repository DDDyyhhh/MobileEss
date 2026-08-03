#include <tim.h>

// EG2104 H桥控制结构体
// EG2104 H桥控制结构体
typedef struct
{
    TIM_HandleTypeDef *left_timer;  // 左侧EG2104 IN控制定时器
    TIM_HandleTypeDef *right_timer; // 右侧EG2104 IN控制定时器
    uint32_t left_in_channel;       // 左侧EG2104 IN控制通道 (TIM3_CH3)
    uint32_t right_in_channel;      // 右侧EG2104 IN控制通道 (TIM3_CH4)
    // GPIO_TypeDef* sd_port;           // SD使能控制端口
    // uint16_t sd_pin;                 // SD使能控制引脚
    float current_speed;     // 当前速度
    uint8_t direction;       // 方向
    uint32_t last_move_time; // 上次移动时间戳
    uint8_t is_moving;       // 是否正在移动
} EG2104_HBridge_t;

// 只声明函数，不定义变量
extern EG2104_HBridge_t front_bridge;
// 全局后桥实例
extern EG2104_HBridge_t behind_bridge;

// 新增高度控制函数
float EG2104_HBridge_GetCurrentHeight(EG2104_HBridge_t *bridge);
void EG2104_HBridge_HeightControlUpdate(EG2104_HBridge_t *bridge, float target_height);
void EG2104_HBridge_CalibrateHeight(EG2104_HBridge_t *bridge, float initial_height);

void EG2104_HBridge_Init(EG2104_HBridge_t *bridge);
HAL_StatusTypeDef EG2104_HBridge_SetSpeed(EG2104_HBridge_t *bridge, float speed);
void EG2104_HBridge_Stop(EG2104_HBridge_t *bridge);