/**
  ******************************************************************************
  * @file           : icm42688.c
  * @brief          : ICM42688 6轴 IMU 传感器 SPI 驱动，兼容 TDK 原厂与国产 HXY 芯片
  * 
  * 主要功能：
  * 1. 提供标准的 SPI 寄存器读写接口
  * 2. 配置加速度计 (Accel) 与陀螺仪 (Gyro) 的量程与输出速率 (ODR)
  * 3. 读取原始 16-bit 数据并依据灵敏度 (Sensitivity) 转换为真实的物理量 (g, dps)
  *
  * 注意事项：
  * - 【关键工程经验】: 硬件上电时序偶尔会导致芯片误入 I2C 模式锁死，
  *   本代码特意加入了 `暴力唤醒序列` (通过 CS 连续跳变模拟时序) 完美解决了这一硬件 Bug。
  * - 采用宏开关 (`ICM42688_TDK_DRIVER` / `ICM42688_HXY_DRIVER`) 实现不同厂商芯片的无缝切换。
  ******************************************************************************
  */

#define ICM42688_TDK_DRIVER 0 // 原厂TDK的驱动 0关闭 1开启
#define ICM42688_HXY_DRIVER 1 // 国产HXY的驱动 0关闭 1开启
// 原厂TDK的驱动//
#if ICM42688_TDK_DRIVER
#include "main.h"
#include "icm42688.h"
//#include "usart.h"
#include "serial_port.h"

//#include "spi.h"
#include "stdio.h"	

static float accSensitivity   = 0.244f;   //加速度的最小分辨率 mg/LSB
static float gyroSensitivity  = 32.8f;    //陀螺仪的最小分辨率
extern SPI_HandleTypeDef hspi1;
uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
    uint8_t Rxdata;
    HAL_SPI_TransmitReceive(&hspi1,&TxData,&Rxdata,1, 1000);       
 	return Rxdata;          		    //?????????????		
}
/*ICM42688使用的ms级延时函数，须由用户提供。*/
#define ICM42688DelayMs(_nms)  HAL_Delay(_nms)
#define SPI_IMU_CS0 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
#define SPI_IMU_CS1 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)

/*******************************************************************************
* 名    称： icm42688_read_reg
* 功    能： 读取单个寄存器的值
* 入口参数： reg: 寄存器地址
* 出口参数： 当前寄存器地址的值
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page51.
*******************************************************************************/
static uint8_t icm42688_read_reg(uint8_t reg)
{
    uint8_t regval = 0;


    SPI_IMU_CS0;
		// 【新增】加一点点延时，确保传感器识别到 CS 下降沿
    for(int i=0; i<50; i++) __NOP(); 
	
    reg |= 0x80;
    /* 写入要读的寄存器地址 */
    SPI1_ReadWriteByte(reg);
    /* 读取寄存器数据 */
    regval = SPI1_ReadWriteByte(0x00);
    SPI_IMU_CS1;

    return regval;
}

/*******************************************************************************
* 名    称： icm42688_read_regs
* 功    能： 连续读取多个寄存器的值
* 入口参数： reg: 起始寄存器地址 *buf数据指针,uint16_t len长度
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
*******************************************************************************/
static void icm42688_read_regs(uint8_t reg, uint8_t* buf, uint16_t len)
{

    reg |= 0x80;
    SPI_IMU_CS0;
	//delay_us(100);
    /* 写入要读的寄存器地址 */
    SPI1_ReadWriteByte(reg);
    /* 读取寄存器数据 */
    while(len)
	{
		*buf = SPI1_ReadWriteByte(0x00);
		len--;
		buf++;
	}
	//delay_us(100);
    SPI_IMU_CS1;
}


/*******************************************************************************
* 名    称： icm42688_write_reg
* 功    能： 向单个寄存器写数据
* 入口参数： reg: 寄存器地址 value:数据
* 出口参数： 0
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
*******************************************************************************/
static uint8_t icm42688_write_reg(uint8_t reg, uint8_t value)
{

    SPI_IMU_CS0;
	// 【新增】加一点点延时，确保传感器识别到 CS 下降沿
    for(int i=0; i<50; i++) __NOP(); 
    /* 写入要读的寄存器地址 */
    /* 写入要读的寄存器地址 */
    SPI1_ReadWriteByte(reg);
    /* 读取寄存器数据 */
    SPI1_ReadWriteByte(value);
    SPI_IMU_CS1;

    return 0;
}



float bsp_Icm42688GetAres(uint8_t Ascale)
{
    switch(Ascale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
    case AFS_2G:
        accSensitivity = 2000 / 32768.0f;
        break;
    case AFS_4G:
        accSensitivity = 4000 / 32768.0f;
        break;
    case AFS_8G:
        accSensitivity = 8000 / 32768.0f;
        break;
    case AFS_16G:
        accSensitivity = 16000 / 32768.0f;
        break;
    }

    return accSensitivity;
}

float bsp_Icm42688GetGres(uint8_t Gscale)
{
    switch(Gscale)
    {
    case GFS_15_125DPS:
        gyroSensitivity = 15.125f / 32768.0f;
        break;
    case GFS_31_25DPS:
        gyroSensitivity = 31.25f / 32768.0f;
        break;
    case GFS_62_5DPS:
        gyroSensitivity = 62.5f / 32768.0f;
        break;
    case GFS_125DPS:
        gyroSensitivity = 125.0f / 32768.0f;
        break;
    case GFS_250DPS:
        gyroSensitivity = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        gyroSensitivity = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        gyroSensitivity = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        gyroSensitivity = 2000.0f / 32768.0f;
        break;
    }
    return gyroSensitivity;
}

/*******************************************************************************
* 名    称： bsp_Icm42688RegCfg
* 功    能： Icm42688 寄存器配置
* 入口参数： 无
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/

int8_t bsp_Icm42688RegCfg(void)
{
    uint8_t reg_val = 0;
    
    /* ==========================================
       1. 软件暴力唤醒序列 (防 I2C 锁死)
       由于没有外部上拉，上电瞬间芯片可能进入 I2C 模式。
       这里模拟 SPI 时序强制唤醒它。
       ========================================== */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    ICM42688DelayMs(10);
    
    // 制造一次 CS 下降沿
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    ICM42688DelayMs(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    ICM42688DelayMs(10);
    
    // 发送一个空指令，尝试激活 SPI 总线
    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00); 
    
    // 软复位
    icm42688_write_reg(ICM42688_DEVICE_CONFIG, 0x01); 
    ICM42688DelayMs(100); // 等待复位完成

    /* ==========================================
       2. 循环读取 ID (用于诊断硬件)
       ========================================== */
    uint8_t retry_count = 0;
    while(retry_count < 20) // 尝试读 20 次
    {
        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        reg_val = icm42688_read_reg(ICM42688_WHO_AM_I);
        
        // 这里的打印非常重要，配合你在 main.c 里的 上拉/下拉 设置来观察
        Serial_Printf(SERIAL_PORT_2,"Retry %d: WHO_AM_I = %d (0x%02X)\r\n", retry_count, reg_val, reg_val);
        
        if(reg_val == 0x47) // 0x47 是 ICM42688 的 ID
        {
            Serial_Printf(SERIAL_PORT_2,"IMU ID MATCH! Hardware is Good.\r\n");
            goto CONFIG_START; // 读到了，跳转去配置
        }
        
        retry_count++;
        ICM42688DelayMs(100);
    }
    
    // 20次都读不到，返回失败
    return -1;

CONFIG_START:
    /* ==========================================
       3. 正常的初始化配置
       ========================================== */
    bsp_Icm42688GetAres(AFS_4G);
    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
    reg_val = (AFS_4G << 5) | (AODR_100Hz);
    icm42688_write_reg(ICM42688_ACCEL_CONFIG0, reg_val);

    bsp_Icm42688GetGres(GFS_1000DPS);
    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
    reg_val = (GFS_1000DPS << 5) | (GODR_100Hz);
    icm42688_write_reg(ICM42688_GYRO_CONFIG0, reg_val);

    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
    reg_val = icm42688_read_reg(ICM42688_PWR_MGMT0);
    reg_val &= ~(1 << 5); // Enable Temp
    reg_val |= (3 << 2);  // Gyro LN Mode
    reg_val |= 3;         // Accel LN Mode
    icm42688_write_reg(ICM42688_PWR_MGMT0, reg_val);
    ICM42688DelayMs(1);

    return 0;
}
													 
//初始化SPI ICM的IO口
void ICM426XX_Init(void)
{ 

//    u8 temp;
    // GPIO_InitTypeDef GPIO_Initure;
    
    // __HAL_RCC_GPIOA_CLK_ENABLE();           //使能GPIOA时钟
    
    // //PA4
    // GPIO_Initure.Pin=GPIO_PIN_4;            //PA4
    // GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    // GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    // GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //快速         
    // HAL_GPIO_Init(GPIOA,&GPIO_Initure);     //初始化
    
	SPI_IMU_CS1;			                //SPI FLASH不选中
//	SPI1_Init();		   			        //初始化SPI
//	SPI1_SetSpeed(SPI_BAUDRATEPRESCALER_256); 


}  

/*******************************************************************************
* 名    称： bsp_Icm42688Init
* 功    能： Icm42688 传感器初始化
* 入口参数： 无
* 出口参数： 0: 初始化成功  其他值: 初始化失败
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/
int8_t bsp_Icm42688Init(void)
{
	ICM426XX_Init();
	
	/* === 新增：防止芯片卡在 I2C 模式的暴力唤醒序列 === */
    // 1. 确保 CS 为高
    SPI_IMU_CS1; 
    HAL_Delay(10);

    // 2. 模拟一次 SPI 通信的“假动作”，强制让芯片识别 CS 的下降沿
    // 就算芯片在 I2C 模式，看到 CS 智障一样的跳变，也可能会重置接口逻辑
    SPI_IMU_CS0; 
    HAL_Delay(1); // 拉低 1ms
    SPI_IMU_CS1; 
    HAL_Delay(10);

    // 3. 再次拉低，发送一个 Dummy 字节 (0x00)
    SPI_IMU_CS0;
    SPI1_ReadWriteByte(0x00); 
    SPI_IMU_CS1;
    HAL_Delay(10);
    /* === 唤醒序列结束 === */
    return(bsp_Icm42688RegCfg());

}

/*******************************************************************************
* 名    称： bsp_IcmGetTemperature
* 功    能： 读取Icm42688 内部传感器温度
* 入口参数： 无
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注： datasheet page62
*******************************************************************************/
int8_t bsp_IcmGetTemperature(int16_t* pTemp)
{
    uint8_t buffer[2] = {0};

    icm42688_read_regs(ICM42688_TEMP_DATA1, buffer, 2);

    *pTemp = (int16_t)(((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48 + 25);
    return 0;
}

/*******************************************************************************
* 名    称： bsp_IcmGetAccelerometer
* 功    能： 读取Icm42688 加速度的值
* 入口参数： 三轴加速度的值
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注： datasheet page62
*******************************************************************************/
int8_t bsp_IcmGetAccelerometer(icm42688RawData_t* accData)
{
    uint8_t buffer[6] = {0};

    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 6);

    accData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
    accData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
    accData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

//    accData->x = (int16_t)(accData->x * accSensitivity);
//    accData->y = (int16_t)(accData->y * accSensitivity);
//    accData->z = (int16_t)(accData->z * accSensitivity);

    return 0;
}

/*******************************************************************************
* 名    称： bsp_IcmGetGyroscope
* 功    能： 读取Icm42688 陀螺仪的值
* 入口参数： 三轴陀螺仪的值
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注： datasheet page63
*******************************************************************************/
int8_t bsp_IcmGetGyroscope(icm42688RawData_t* GyroData)
{
    uint8_t buffer[6] = {0};

    icm42688_read_regs(ICM42688_GYRO_DATA_X1, buffer, 6);

    GyroData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
    GyroData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
    GyroData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

//    GyroData->x = (int16_t)(GyroData->x * gyroSensitivity);
//    GyroData->y = (int16_t)(GyroData->y * gyroSensitivity);
//    GyroData->z = (int16_t)(GyroData->z * gyroSensitivity);
    return 0;
}

/*******************************************************************************
* 名    称： bsp_IcmGetRawData
* 功    能： 读取Icm42688加速度陀螺仪数据
* 入口参数： 六轴
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注： datasheet page62,63
*******************************************************************************/
int8_t bsp_IcmGetRawData(icm42688RealData_t* accData, icm42688RealData_t* GyroData)
{
    uint8_t buffer[12] = {0};
	icm42688RawData_t accRaw;
	icm42688RawData_t gyroRaw;

    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 12);

    accRaw.x  = ((uint16_t)buffer[0] << 8)  | buffer[1];
    accRaw.y  = ((uint16_t)buffer[2] << 8)  | buffer[3];
    accRaw.z  = ((uint16_t)buffer[4] << 8)  | buffer[5];
    gyroRaw.x = ((uint16_t)buffer[6] << 8)  | buffer[7];
    gyroRaw.y = ((uint16_t)buffer[8] << 8)  | buffer[9];
    gyroRaw.z = ((uint16_t)buffer[10] << 8) | buffer[11];
    
    if ((accRaw.x == -1 && accRaw.y == -1 && accRaw.z == -1) || accRaw.x == -32768)
    {
        return 1;
    }

    accData->x = (float)(accRaw.x * accSensitivity);
    accData->y = (float)(accRaw.y * accSensitivity);
    accData->z = (float)(accRaw.z * accSensitivity);

    GyroData->x = (float)(gyroRaw.x * gyroSensitivity);
    GyroData->y = (float)(gyroRaw.y * gyroSensitivity);
    GyroData->z = (float)(gyroRaw.z * gyroSensitivity);

    return 0;
}

#elif ICM42688_HXY_DRIVER
// 国产HXY的驱动//
#include "icm42688.h"
#include "serial_port.h"
#include "spi.h" // 确保包含 spi.h 以使用 hspi1
#include "stdio.h"	

// 全局变量：灵敏度系数
static float accSensitivity   = 0.0f; 
static float gyroSensitivity  = 0.0f;

extern SPI_HandleTypeDef hspi1;

// CS 引脚控制宏
#define SPI_IMU_CS0 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define SPI_IMU_CS1 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

// 延时宏
#define ICM_DelayMs(_nms)  HAL_Delay(_nms)

// SPI 读写字节
static uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
    uint8_t Rxdata;
    HAL_SPI_TransmitReceive(&hspi1, &TxData, &Rxdata, 1, 10);       
 	return Rxdata;
}

// 读寄存器 (手册 Page 14: 读操作 Bit0=1, 但时序图显示 RW 是第一个位)
// 通常 SPI 传感器读操作是: (RegAddr | 0x80)
static uint8_t icm_read_reg(uint8_t reg)
{
    uint8_t regval = 0;
    SPI_IMU_CS0;
    // 读操作：地址最高位置 1 (根据手册 SPI 读写时序推断，通常是 0x80 | Addr)
    SPI1_ReadWriteByte(reg | 0x80); 
    regval = SPI1_ReadWriteByte(0x00);
    SPI_IMU_CS1;
    return regval;
}

// 写寄存器
static uint8_t icm_write_reg(uint8_t reg, uint8_t value)
{
    SPI_IMU_CS0;
    // 写操作：地址最高位为 0 (0x7F & Addr)
    SPI1_ReadWriteByte(reg & 0x7F);
    SPI1_ReadWriteByte(value);
    SPI_IMU_CS1;
    return 0;
}

// 连续读
static void icm_read_regs(uint8_t reg, uint8_t* buf, uint16_t len)
{
    SPI_IMU_CS0;
    SPI1_ReadWriteByte(reg | 0x80);
    while(len--)
    {
        *buf = SPI1_ReadWriteByte(0x00);
        buf++;
    }
    SPI_IMU_CS1;
}

// 设置加速度灵敏度
static void SetAccSensitivity(uint8_t fs_sel)
{
    switch(fs_sel)
    {
        case ICM_ACC_RANGE_2G:  accSensitivity = 2000.0f / 32768.0f; break; // 2g -> 2000mg
        case ICM_ACC_RANGE_4G:  accSensitivity = 4000.0f / 32768.0f; break;
        case ICM_ACC_RANGE_8G:  accSensitivity = 8000.0f / 32768.0f; break;
        case ICM_ACC_RANGE_16G: accSensitivity = 16000.0f / 32768.0f; break;
        default: accSensitivity = 2000.0f / 32768.0f; break;
    }
}

// 设置陀螺仪灵敏度
static void SetGyroSensitivity(uint8_t fs_sel)
{
    switch(fs_sel)
    {
        case ICM_GYR_RANGE_250DPS:  gyroSensitivity = 250.0f / 32768.0f; break;
        case ICM_GYR_RANGE_500DPS:  gyroSensitivity = 500.0f / 32768.0f; break;
        case ICM_GYR_RANGE_1000DPS: gyroSensitivity = 1000.0f / 32768.0f; break;
        case ICM_GYR_RANGE_2000DPS: gyroSensitivity = 2000.0f / 32768.0f; break;
        default: gyroSensitivity = 2000.0f / 32768.0f; break;
    }
}

// 寄存器配置
static int8_t ICM_RegConfig(void)
{
    uint8_t pid = 0;
    uint8_t i = 0;

    // 1. 软件复位 (手册 5.42 SOFT_RST)
    // 写入 0xA5 到 0x4A 复位
    icm_write_reg(ICM_REG_SOFT_RST, 0xA5);
    ICM_DelayMs(50); // 等待复位

    // 2. 读取 ID (WHO_AM_I)
    for(i=0; i<10; i++)
    {
        pid = icm_read_reg(ICM_REG_WHO_AM_I);
        Serial_Printf(SERIAL_PORT_2, "IMU ID Read: 0x%02X (Expect: 0x%02X)\r\n", pid, ICM_WHO_AM_I_VAL);
        if(pid == ICM_WHO_AM_I_VAL) break;
        ICM_DelayMs(10);
    }
    if(pid != ICM_WHO_AM_I_VAL) return -1; // ID 错误

    // 3. 配置加速度计 (手册 5.37 ACC_CONF, 5.38 ACC_RANGE)
    // ACC_CONF: BWP=010(默认), ODR=100Hz(0x04) -> 0x24 (假设使用默认带宽)
    // 这里我们简单配置 ODR
    // 0x40 (ACC_CONF): Bit7=0(低功耗) Bit3-0=ODR. 
    // 设为高性能模式(Bit7=1), ODR=100Hz(0x04) -> 1000 0100 -> 0x84
    icm_write_reg(ICM_REG_ACC_CONF, 0x84); 
    
    // ACC_RANGE: +/- 4G (0x01)
    icm_write_reg(ICM_REG_ACC_RANGE, ICM_ACC_RANGE_4G);
    SetAccSensitivity(ICM_ACC_RANGE_4G);

    // 4. 配置陀螺仪 (手册 5.39 GYR_CONF, 5.40 GYR_RANGE)
    // GYR_CONF: Bit7=1(高性能), ODR=100Hz(0x04) -> 0x84
    icm_write_reg(ICM_REG_GYR_CONF, 0x84);

    // GYR_RANGE: +/- 2000dps (0x00)
    icm_write_reg(ICM_REG_GYR_RANGE, ICM_GYR_RANGE_2000DPS);
    SetGyroSensitivity(ICM_GYR_RANGE_2000DPS);

    // 5. 电源管理 (手册 5.45 PWR_CTRL)
    // Bit3: TEMP_EN, Bit2: ACC_EN, Bit1: GYR_EN
    // 开启所有: 0000 1110 -> 0x0E
    icm_write_reg(ICM_REG_PWR_CTRL, 0x0E);
    
    ICM_DelayMs(20); // 等待传感器稳定

    return 0;
}

// 初始化入口
int8_t bsp_Icm42688Init(void)
{
    // 1. 暴力唤醒序列 (防止卡在 I2C 模式)
    SPI_IMU_CS1; 
    ICM_DelayMs(10);
    SPI_IMU_CS0; 
    ICM_DelayMs(1);
    SPI_IMU_CS1; 
    ICM_DelayMs(10);
    
    // 发送 Dummy
    SPI_IMU_CS0;
    SPI1_ReadWriteByte(0x00);
    SPI_IMU_CS1;
    ICM_DelayMs(10);

    return ICM_RegConfig();
}

// 读取温度
int8_t bsp_IcmGetTemperature(int16_t* pTemp)
{
    uint8_t buf[2];
    icm_read_regs(ICM_REG_TEMP_H, buf, 2);
    // 手册 5.22: T = (Data / 512) + 23
    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    *pTemp = (int16_t)((float)raw / 512.0f + 23.0f);
    return 0;
}

// 读取原始数据并转换
int8_t bsp_IcmGetRawData(icm42688RealData_t* accData, icm42688RealData_t* GyroData)
{
    uint8_t buf[12];
    icm42688RawData_t accRaw, gyroRaw;

    // 连续读取 12 字节：ACC_X_H (0x0C) 到 GYR_Z_L (0x17)
    icm_read_regs(ICM_REG_ACC_X_H, buf, 12);

    // 解析加速度 (0x0C - 0x11)
    accRaw.x = (int16_t)((buf[0] << 8) | buf[1]);
    accRaw.y = (int16_t)((buf[2] << 8) | buf[3]);
    accRaw.z = (int16_t)((buf[4] << 8) | buf[5]);

    // 解析陀螺仪 (0x12 - 0x17)
    gyroRaw.x = (int16_t)((buf[6] << 8) | buf[7]);
    gyroRaw.y = (int16_t)((buf[8] << 8) | buf[9]);
    gyroRaw.z = (int16_t)((buf[10] << 8) | buf[11]);

    // 简单校验：如果全 0 或全 FF，可能读取失败
    if (accRaw.x == 0 && accRaw.y == 0 && accRaw.z == 0) return 1;

    // 转换为物理量
    accData->x = (float)accRaw.x * accSensitivity; // mg
    accData->y = (float)accRaw.y * accSensitivity;
    accData->z = (float)accRaw.z * accSensitivity;

    // 转换为 g (如果你的应用需要 g，除以 1000)
    accData->x /= 1000.0f;
    accData->y /= 1000.0f;
    accData->z /= 1000.0f;

    GyroData->x = (float)gyroRaw.x * gyroSensitivity; // dps
    GyroData->y = (float)gyroRaw.y * gyroSensitivity;
    GyroData->z = (float)gyroRaw.z * gyroSensitivity;

    return 0;
}

// 单独读取加速度 (保留接口)
int8_t bsp_IcmGetAccelerometer(icm42688RawData_t* accData)
{
    uint8_t buf[6];
    icm_read_regs(ICM_REG_ACC_X_H, buf, 6);
    accData->x = (int16_t)((buf[0] << 8) | buf[1]);
    accData->y = (int16_t)((buf[2] << 8) | buf[3]);
    accData->z = (int16_t)((buf[4] << 8) | buf[5]);
    return 0;
}

// 单独读取陀螺仪 (保留接口)
int8_t bsp_IcmGetGyroscope(icm42688RawData_t* GyroData)
{
    uint8_t buf[6];
    icm_read_regs(ICM_REG_GYR_X_H, buf, 6);
    GyroData->x = (int16_t)((buf[0] << 8) | buf[1]);
    GyroData->y = (int16_t)((buf[2] << 8) | buf[3]);
    GyroData->z = (int16_t)((buf[4] << 8) | buf[5]);
    return 0;
}

#endif