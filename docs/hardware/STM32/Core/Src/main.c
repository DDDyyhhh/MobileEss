/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "E34.h"
#include "pairing_receiver.h"
#include "BladeMotor.h"
#include "car_Control.h"
#include "connet_check.h"
#include "car_Control.h"
#include "EG2104.h"

#include "read_adc.h"
#include "odrive_can.h"

#include <errno.h>
#include <stdarg.h>       // 娣诲姞杩欒
#include <string.h>       // 纭繚鍖呭惈string.h
#include <stdio.h>        // 纭繚鍖呭惈stdio.h
#include "serial_port.h"  // 涓插彛閫氫俊
#include "lifter.h"       //鐢靛姩鎺ㄦ潌
#include "mower_config.h" // <<<---绠＄悊鎵€鏈夌殑閰嶇疆鍙傛暟鍜屽畾涔?

#include "rs485.h"
#include "AXEBMS_RS485.h" // BMS鍗忚椹卞姩

#include "modbus_rtu_bus.h" // Modbus鎬荤嚎绠＄悊

#include "icm42688.h"
#include "bsp_imu_solver.h"
#include "tricolor_led.h"
#include "buzzer.h"
#include "heartbeat.h" // <<<--- 1. 鍖呭惈蹇冭烦妯″潡澶存枃浠?
#include "sr04_uart.h"
#include "cruise_control.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t rx_dma_buffer[RX_BUF_SIZE];
extern osMessageQueueId_t Queue_RxDataHandle;
uint8_t status[8];

// 鍏ㄥ眬鍙橀噺瀛樺偍浼犳劅鍣ㄦ暟鎹?
int16_t raw_accel[3];
int16_t raw_gyro[3];
float accel_g[3];  // 瀛樺偍鍔犻€熷害(g)
float gyro_dps[3]; // 瀛樺偍瑙掗€熷害(搴?绉?
uint8_t rx_buffer[255];
uint8_t uart1_rx_buffer[255] = {0};
uint8_t uart3_rx_buffer[255] = {0};
volatile uint16_t new_address1;
volatile uint8_t TESTok;

uint16_t currentA_int = 1;

uint16_t ly = 2048;
uint16_t rx = 2048;
uint8_t Current_angle = 0;

uint8_t Current_rpm;

// #define COMMUNICATION_TIMEOUT_MS 1000 // 1绉掕秴鏃?
// 鍦ㄥ叏灞€鍙橀噺鍖哄畾涔夊噺閫熺浉鍏崇殑鍙傛暟锛屾柟渚胯皟鏁?
#define ADAPTIVE_SPEED_SLOWDOWN_THRESHOLD_CM 100.0f
#define ADAPTIVE_SPEED_STOP_THRESHOLD_CM 50.0f
#define ADAPTIVE_SPEED_REDUCED_SPEED 0.5f

// 澹版槑澶栭儴鐨勫Э鎬佽缁撴瀯浣擄紝浠ヤ究鍦╩ain涓彲浠ヨ闂紙濡傛灉闇€瑕佺殑璇濓級
// extern Angles_t vehicle_angles;

static float g_ultrasonic_distance_cm = -1.0f; // 鐢ㄤ簬淇濆瓨鏈€鏂扮殑鏈夋晥璺濈

// 鍒涘缓涓€涓叏灞€鎴栭潤鎬佺殑BMS瀹炰緥
AXEBMS_RS485 my_bms;

// 鐢ㄦ埛璁惧畾鐨勫熀纭€閫熷害 (榛樿1.5)

float min_distance_cm = 999.0f; // 鍒濆鍖栦负涓€涓緢澶х殑鍊?

// 銆愭柊澧炪€戜繚瀛樼敤鎴疯瀹氱殑妗ｄ綅閫熷害锛岄粯璁?.5f

// 瀛樺偍鍘熷鏁版嵁 (16浣嶆暣鏁?
int16_t test_raw_accel[3];
int16_t test_raw_gyro[3];

// 瀛樺偍杞崲鍚庣殑鐗╃悊閲?(娴偣鏁?
float test_accel_g[3];
float test_gyro_dps[3];
float test_temp;

// 鍊剧炕淇濇姢闃堝€?(鍗曚綅锛氬害)
#define MAX_TILT_ANGLE 50.0f

uint16_t RPM;
uint16_t current_adc;

volatile uint8_t TEST_OK = 0;

Heartbeat_Handle_t g_hb_mower; // 瀹氫箟蹇冭烦瀹炰緥

extern osSemaphoreId_t Sem_TxCompleteHandle;
extern osSemaphoreId_t Sem_TxCompleteWirelessHandle;
extern osSemaphoreId_t Sem_RS485_RxHandle; // 寮曠敤 CubeMX 鍒涘缓鐨勪俊鍙烽噺

void RS485_TxCpltCallback(UART_HandleTypeDef *huart);

// 涓?ODrive 涓插彛鍒涘缓鐙珛鐨?DMA 鎺ユ敹缂撳啿鍖?
uint8_t rx_dma_buffer_port1[64];
uint8_t rx_dma_buffer_port2[64];

// 鐢ㄤ簬瀛樻斁瑙ｆ瀽鍚庣殑鍝嶅簲鏁版嵁
uint8_t g_odrive_port1_rx_buffer[64];
volatile uint16_t g_odrive_port1_rx_len = 0;
uint8_t g_odrive_port2_rx_buffer[64];
volatile uint16_t g_odrive_port2_rx_len = 0;

// 寮曠敤鐢?CubeMX 鐢熸垚鐨勫彞鏌?
extern osSemaphoreId_t ODrivePort1ResponseSemHandle;
extern osSemaphoreId_t ODrivePort2ResponseSemHandle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t g_10ms_task_flag = 0; // 10ms浠诲姟鏍囧織浣?

static uint32_t g_stopwatch_start_time = 0;
static uint8_t g_stopwatch_running = 0;

volatile uint8_t g_obstacle_detected = 0; // 鍏ㄥ眬闅滅鐗╂娴嬫爣蹇?

// --- 鐢ㄤ簬姝ヨ繘鎺у埗鐨勬寜閿簨浠舵娴嬪彉閲?---
static uint8_t key0_was_pressed = 0;
static uint8_t key1_was_pressed = 0;

bool g_last_key_state[8] = {false};        // 淇濆瓨8涓寜閿殑涓婁竴娆＄姸鎬?(false=鏉惧紑, true=鎸変笅)
static uint32_t g_key_press_time[8] = {0}; // 淇濆瓨8涓寜閿殑鎸変笅璧峰鏃堕棿

static uint32_t obstacle_clear_timestamp = 0; // 鐢ㄤ簬寤惰繜娓呴櫎鐨勮鏃跺櫒

// 褰撳墠瀹為檯杈撳嚭缁欑數鏈虹殑閫熷害 (鐢ㄤ簬骞虫粦澶勭悊)

// 鍔犻€熷害闄愬埗 (姣?00ms澧炲姞鐨勯€熷害鍊?
// 鍋囪 100ms 璋冪敤涓€娆★紝0.1f 琛ㄧず 1绉掑唴閫熷害澧炲姞 1.0
// #define SPEED_RAMP_UP_STEP 0.1f
// #define SPEED_RAMP_DOWN_STEP 0.5f // 鍑忛€熻蹇紝鍔犻€熻鎱?

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  float angles[3];
  // 1. 立即开启 GPIOA 时钟 (F4 系列)
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  // 2. 强行将 PA15 配置为普通的 GPIO 推挽输出，并拉低
  // 在 STM32F4 中，只要将 MODER 寄存器配置为 GPIO Output，引脚就会瞬间脱离 JTAG-JTDI 模式
  // 此时 SWD 调试线（PA13/PA14）依然100%正常工作，不会影响你下载和仿真
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  
  // 先将输出寄存器写 0，确保硬件初始化完毕的瞬间就是低电平，防止毛刺
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); 
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_CAN1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_TIM7_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  // ===================================================================
  //                      鏂扮殑銆佺粨鏋勫寲鐨勫垵濮嬪寲娴佺▼
  // ===================================================================

  // 1. 鍒濆鍖栧簳灞傚璁惧拰閫氫俊鎺ュ彛
  // 1. 浠呭垵濮嬪寲璋冭瘯涓插彛 (涓轰簡鑳芥墦鍗?Boot 淇℃伅)
  Serial_Init();
  Serial_Printf(SERIAL_PORT_2, "\r\n--- System Boot ---\r\n");
  
  // 2. 鍒濆鍖?CAN 杩囨护鍣ㄤ笌鎺ユ敹涓柇
  User_CAN_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */
     

  /* USER CODE END 3 */
}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  // 鍒ゆ柇鏄惁鏄?ADC1
  if (hadc->Instance == ADC1)
  {
    // 璇诲彇褰撳墠 ADC 鍊?
    current_adc = HAL_ADC_GetValue(hadc);

    // 鍒ゆ柇鏄惁杩囨祦 ( > 2200 )
    if (current_adc > 2200)
    {
      // 1. 绔嬪嵆鍋滄鐢垫満
      // BladeMotor_SetSpeed(0.0f);

      // 2. 鍙€夛細璁剧疆涓€涓姤璀︽爣蹇椾綅锛岄槻姝富寰幆涓嶇煡鎯呭張鎵撳紑
      // g_motor_fault = 1;

      // 3. 璋冭瘯鐢細涓轰簡闃叉涓柇閲岀柉鐙傛墦鍗板崱姝伙紝鍙互鍔犱釜璁℃暟鍣ㄩ檺鍒舵墦鍗?
      // static uint8_t print_once = 0;
      // if(print_once == 0) {
      //     Serial_Printf(SERIAL_PORT_2, "OVERCURRENT: %d\r\n", current_val);
      //     print_once = 1;
      // }
    }
  }
}

// 褰?DMA 缂撳啿鍖烘弧 鎴栬€?鍙戠敓 IDLE (绌洪棽) 涓柇鏃讹紝浼氳皟鐢ㄦ鍥炶皟
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  // 1. 澶勭悊鏃犵嚎妯″潡/閬ユ帶鍣?(USART3)
  if (huart->Instance == USART3)
  {
    UartRxMsg_t msg;
    msg.len = (Size > RX_BUF_SIZE) ? RX_BUF_SIZE : Size;
    memcpy(msg.data, rx_dma_buffer, msg.len);
    osMessageQueuePut(Queue_RxDataHandle, &msg, 0, 0);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_dma_buffer, RX_BUF_SIZE);
  }

  // 2. 澶勭悊 RS485 BMS (UART4)
  else if (huart->Instance == UART4)
  {
    // 璋冪敤 rs485.c 閲岀殑澶勭悊鍑芥暟
    RS485_RxEventCallback(huart, Size);
  }
  else if (huart->Instance == USART6)
  { // 瓒呭０娉?
    SR04_RxEventCallback(huart, Size);
  }

  // 鏉ヨ嚜 ODrive 1 (鍙宠疆) 鐨勫搷搴?(huart1)
  else if (huart->Instance == USART1)
  {
    g_odrive_port1_rx_len = (Size < 64) ? Size : 63;
    memcpy(g_odrive_port1_rx_buffer, rx_dma_buffer_port1, g_odrive_port1_rx_len);
    g_odrive_port1_rx_buffer[g_odrive_port1_rx_len] = '\0';

    if (ODrivePort1ResponseSemHandle != NULL)
    {
      osSemaphoreRelease(ODrivePort1ResponseSemHandle);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buffer_port1, 64);
  }
  // 鏉ヨ嚜 ODrive 2 (宸﹁疆) 鐨勫搷搴?(huart2)
  else if (huart->Instance == USART2)
  {
    g_odrive_port2_rx_len = (Size < 64) ? Size : 63;
    memcpy(g_odrive_port2_rx_buffer, rx_dma_buffer_port2, g_odrive_port2_rx_len);
    g_odrive_port2_rx_buffer[g_odrive_port2_rx_len] = '\0';

    if (ODrivePort2ResponseSemHandle != NULL)
    {
      osSemaphoreRelease(ODrivePort2ResponseSemHandle);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buffer_port2, 64);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1 || huart->Instance == USART2)
  {
    if (Sem_TxCompleteHandle != NULL)
    {
      osSemaphoreRelease(Sem_TxCompleteHandle);
    }
  }
  else if (huart->Instance == USART3)
  {
    if (Sem_TxCompleteWirelessHandle != NULL)
    {
      osSemaphoreRelease(Sem_TxCompleteWirelessHandle);
    }
  }
  else if (huart->Instance == UART4)
  {
    RS485_TxCpltCallback(huart);
  }
}

/**
 * @brief UART 閿欒鍥炶皟鍑芥暟
 * @note  褰撳彂鐢?ORE (婧㈠嚭), NE (鍣０), FE (甯ч敊璇? 鏃讹紝HAL搴撲細璋冪敤姝ゅ嚱鏁?
 *        涓?DMA 鎺ユ敹閫氬父浼氳 HAL 搴撹嚜鍔ㄥ叧闂€傛垜浠渶瑕佸湪杩欓噷閲嶅惎瀹冦€?
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  // 1. 澶勭悊鏃犵嚎妯″潡 (USART3) 鐨勯敊璇?
  if (huart->Instance == USART3)
  {
    // 杩欓噷鐨勯敊璇€氬父鏄?HAL_UART_ERROR_ORE (婧㈠嚭) 鎴?HAL_UART_ERROR_NE (鍣０)

    // 鏆村姏閲嶅惎鎺ユ敹
    // 娉ㄦ剰锛歳x_dma_buffer 鍜?RX_BUF_SIZE 闇€瑕佹槸鍏ㄥ眬鍙鐨勶紝鎴栬€呭湪杩欓噷閲嶆柊澹版槑寮曠敤
    extern uint8_t rx_dma_buffer[];
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_dma_buffer, 255); // 纭繚闀垮害瀹忎竴鑷?
  }

  // 2. 澶勭悊 RS485 (UART4) 鐨勯敊璇?
  else if (huart->Instance == UART4)
  {
    extern void RS485_Init(void); // 鎴栬€呭紩鐢?rs485.c 閲岀殑閲嶅惎鍑芥暟
    // 绠€鍗曠矖鏆村湴閲嶆柊鍒濆鍖栨帴鏀?
    // 鏇村ソ鐨勬柟寮忔槸璋冪敤 rs485.c 閲岀殑 RS485_Start_Receive()锛屼絾杩欓渶瑕佽鍑芥暟闈?static
    // 杩欓噷鍋囪浣犳妸 RS485_Init 鏀逛负浜嗛噸鍚帴鏀?
    RS485_Init();
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */



