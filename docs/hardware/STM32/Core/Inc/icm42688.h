#define ICM42688_TDK_DRIVER 0 // 原厂TDK的驱动 0关闭 1开启
#define ICM42688_HXY_DRIVER 1 // 国产HXY的驱动 0关闭 1开启
// 原厂TDK的驱动//
#if ICM42688_TDK_DRIVER

//原厂TDK的驱动

#ifndef __ICM42688_H__
#define __ICM42688_H__

#include <stdint.h>

#define ICM_USE_HARD_SPI
//#define ICM_USE_I2C

/* ICM42688 registers
https://store.invensense.com/datasheets/invensense/DS-ICM-42688v1-2.pdf
*/
// Bank 0
#define ICM42688_DEVICE_CONFIG             0x11
#define ICM42688_DRIVE_CONFIG              0x13
#define ICM42688_INT_CONFIG                0x14
#define ICM42688_FIFO_CONFIG               0x16
#define ICM42688_TEMP_DATA1                0x1D
#define ICM42688_TEMP_DATA0                0x1E
#define ICM42688_ACCEL_DATA_X1             0x1F
#define ICM42688_ACCEL_DATA_X0             0x20
#define ICM42688_ACCEL_DATA_Y1             0x21
#define ICM42688_ACCEL_DATA_Y0             0x22
#define ICM42688_ACCEL_DATA_Z1             0x23
#define ICM42688_ACCEL_DATA_Z0             0x24
#define ICM42688_GYRO_DATA_X1              0x25
#define ICM42688_GYRO_DATA_X0              0x26
#define ICM42688_GYRO_DATA_Y1              0x27
#define ICM42688_GYRO_DATA_Y0              0x28
#define ICM42688_GYRO_DATA_Z1              0x29
#define ICM42688_GYRO_DATA_Z0              0x2A
#define ICM42688_TMST_FSYNCH               0x2B
#define ICM42688_TMST_FSYNCL               0x2C
#define ICM42688_INT_STATUS                0x2D
#define ICM42688_FIFO_COUNTH               0x2E
#define ICM42688_FIFO_COUNTL               0x2F
#define ICM42688_FIFO_DATA                 0x30
#define ICM42688_APEX_DATA0                0x31
#define ICM42688_APEX_DATA1                0x32
#define ICM42688_APEX_DATA2                0x33
#define ICM42688_APEX_DATA3                0x34
#define ICM42688_APEX_DATA4                0x35
#define ICM42688_APEX_DATA5                0x36
#define ICM42688_INT_STATUS2               0x37
#define ICM42688_INT_STATUS3               0x38
#define ICM42688_SIGNAL_PATH_RESET         0x4B
#define ICM42688_INTF_CONFIG0              0x4C
#define ICM42688_INTF_CONFIG1              0x4D
#define ICM42688_PWR_MGMT0                 0x4E
#define ICM42688_GYRO_CONFIG0              0x4F
#define ICM42688_ACCEL_CONFIG0             0x50
#define ICM42688_GYRO_CONFIG1              0x51
#define ICM42688_GYRO_ACCEL_CONFIG0        0x52
#define ICM42688_ACCEL_CONFIG1             0x53
#define ICM42688_TMST_CONFIG               0x54
#define ICM42688_APEX_CONFIG0              0x56
#define ICM42688_SMD_CONFIG                0x57
#define ICM42688_FIFO_CONFIG1              0x5F
#define ICM42688_FIFO_CONFIG2              0x60
#define ICM42688_FIFO_CONFIG3              0x61
#define ICM42688_FSYNC_CONFIG              0x62
#define ICM42688_INT_CONFIG0               0x63
#define ICM42688_INT_CONFIG1               0x64
#define ICM42688_INT_SOURCE0               0x65
#define ICM42688_INT_SOURCE1               0x66
#define ICM42688_INT_SOURCE3               0x68
#define ICM42688_INT_SOURCE4               0x69
#define ICM42688_FIFO_LOST_PKT0            0x6C
#define ICM42688_FIFO_LOST_PKT1            0x6D
#define ICM42688_SELF_TEST_CONFIG          0x70
#define ICM42688_WHO_AM_I                  0x75
#define ICM42688_REG_BANK_SEL              0x76

// Bank 1
#define ICM42688_SENSOR_CONFIG0            0x03
#define ICM42688_GYRO_CONFIG_STATIC2       0x0B
#define ICM42688_GYRO_CONFIG_STATIC3       0x0C
#define ICM42688_GYRO_CONFIG_STATIC4       0x0D
#define ICM42688_GYRO_CONFIG_STATIC5       0x0E
#define ICM42688_GYRO_CONFIG_STATIC6       0x0F
#define ICM42688_GYRO_CONFIG_STATIC7       0x10
#define ICM42688_GYRO_CONFIG_STATIC8       0x11
#define ICM42688_GYRO_CONFIG_STATIC9       0x12
#define ICM42688_GYRO_CONFIG_STATIC10      0x13
#define ICM42688_XG_ST_DATA                0x5F
#define ICM42688_YG_ST_DATA                0x60
#define ICM42688_ZG_ST_DATA                0x61
#define ICM42688_TMSTVAL0                  0x62
#define ICM42688_TMSTVAL1                  0x63
#define ICM42688_TMSTVAL2                  0x64
#define ICM42688_INTF_CONFIG4              0x7A
#define ICM42688_INTF_CONFIG5              0x7B
#define ICM42688_INTF_CONFIG6              0x7C

// Bank 2
#define ICM42688_ACCEL_CONFIG_STATIC2      0x03
#define ICM42688_ACCEL_CONFIG_STATIC3      0x04
#define ICM42688_ACCEL_CONFIG_STATIC4      0x05
#define ICM42688_XA_ST_DATA                0x3B
#define ICM42688_YA_ST_DATA                0x3C
#define ICM42688_ZA_ST_DATA                0x3D

// Bank 4
#define ICM42688_GYRO_ON_OFF_CONFIG        0x0E
#define ICM42688_APEX_CONFIG1              0x40
#define ICM42688_APEX_CONFIG2              0x41
#define ICM42688_APEX_CONFIG3              0x42
#define ICM42688_APEX_CONFIG4              0x43
#define ICM42688_APEX_CONFIG5              0x44
#define ICM42688_APEX_CONFIG6              0x45
#define ICM42688_APEX_CONFIG7              0x46
#define ICM42688_APEX_CONFIG8              0x47
#define ICM42688_APEX_CONFIG9              0x48
#define ICM42688_ACCEL_WOM_X_THR           0x4A
#define ICM42688_ACCEL_WOM_Y_THR           0x4B
#define ICM42688_ACCEL_WOM_Z_THR           0x4C
#define ICM42688_INT_SOURCE6               0x4D
#define ICM42688_INT_SOURCE7               0x4E
#define ICM42688_INT_SOURCE8               0x4F
#define ICM42688_INT_SOURCE9               0x50
#define ICM42688_INT_SOURCE10              0x51
#define ICM42688_OFFSET_USER0              0x77
#define ICM42688_OFFSET_USER1              0x78
#define ICM42688_OFFSET_USER2              0x79
#define ICM42688_OFFSET_USER3              0x7A
#define ICM42688_OFFSET_USER4              0x7B
#define ICM42688_OFFSET_USER5              0x7C
#define ICM42688_OFFSET_USER6              0x7D
#define ICM42688_OFFSET_USER7              0x7E
#define ICM42688_OFFSET_USER8              0x7F

#define ICM42688_ADDRESS                   0xD2   // Address of ICM42688 accel/gyro when ADO = HIGH


#define AFS_2G  0x03
#define AFS_4G  0x02
#define AFS_8G  0x01
#define AFS_16G 0x00  // default

#define GFS_2000DPS   0x00 // default
#define GFS_1000DPS   0x01
#define GFS_500DPS    0x02
#define GFS_250DPS    0x03
#define GFS_125DPS    0x04
#define GFS_62_5DPS   0x05
#define GFS_31_25DPS  0x06
#define GFS_15_125DPS 0x07

#define AODR_8000Hz   0x03
#define AODR_4000Hz   0x04
#define AODR_2000Hz   0x05
#define AODR_1000Hz   0x06 // default
#define AODR_200Hz    0x07
#define AODR_100Hz    0x08
#define AODR_50Hz     0x09
#define AODR_25Hz     0x0A
#define AODR_12_5Hz   0x0B
#define AODR_6_25Hz   0x0C
#define AODR_3_125Hz  0x0D
#define AODR_1_5625Hz 0x0E
#define AODR_500Hz    0x0F

#define GODR_8000Hz  0x03
#define GODR_4000Hz  0x04
#define GODR_2000Hz  0x05
#define GODR_1000Hz  0x06 // default
#define GODR_200Hz   0x07
#define GODR_100Hz   0x08
#define GODR_50Hz    0x09
#define GODR_25Hz    0x0A
#define GODR_12_5Hz  0x0B
#define GODR_500Hz   0x0F



#define ICM42688_ID	             0x47	


typedef struct {
  int16_t x; /**< Raw int16_t value from the x axis */
  int16_t y; /**< Raw int16_t value from the y axis */
  int16_t z; /**< Raw int16_t value from the z axis */
} icm42688RawData_t;


typedef struct {
  float x; /**< value from the x axis */
  float y; /**< value from the y axis */
  float z; /**< value from the z axis */
} icm42688RealData_t;

#define	SPI_IMU_CS PAout(4)  //选中IMU	
//--------------------------------------------------------//
int8_t bsp_Icm42688Init(void);
int8_t bsp_IcmGetTemperature(int16_t* pTemp);
int8_t bsp_IcmGetAccelerometer(icm42688RawData_t *accData);
int8_t bsp_IcmGetGyroscope(icm42688RawData_t *GyroData);
int8_t bsp_IcmGetRawData(icm42688RealData_t *accData,icm42688RealData_t *GyroData);

#endif

#elif ICM42688_HXY_DRIVER
//国产HXY的驱动
#ifndef __ICM42688_H__
#define __ICM42688_H__

#include <stdint.h>
#include "main.h" // 包含 HAL 库定义

// ==============================================================================
// HXY ICM-42688P 寄存器定义 (基于华选阳数据手册)
// ==============================================================================

// 通用寄存器
#define ICM_REG_WHO_AM_I       0x01
#define ICM_REG_OIS_CONF       0x04
#define ICM_REG_COM_CFG        0x05
#define ICM_REG_INT_CFG1       0x06
#define ICM_REG_INT_CFG2       0x07
#define ICM_REG_HPF_LPF_CFG    0x08
#define ICM_REG_DATA_SAT       0x0A
#define ICM_REG_DATA_STAT      0x0B

// 数据寄存器 (X_H, X_L, Y_H, Y_L, Z_H, Z_L)
#define ICM_REG_ACC_X_H        0x0C
#define ICM_REG_ACC_X_L        0x0D
#define ICM_REG_ACC_Y_H        0x0E
#define ICM_REG_ACC_Y_L        0x0F
#define ICM_REG_ACC_Z_H        0x10
#define ICM_REG_ACC_Z_L        0x11

#define ICM_REG_GYR_X_H        0x12
#define ICM_REG_GYR_X_L        0x13
#define ICM_REG_GYR_Y_H        0x14
#define ICM_REG_GYR_Y_L        0x15
#define ICM_REG_GYR_Z_H        0x16
#define ICM_REG_GYR_Z_L        0x17

#define ICM_REG_TEMP_H         0x22
#define ICM_REG_TEMP_L         0x23

// 配置寄存器
#define ICM_REG_ACC_CONF       0x40
#define ICM_REG_ACC_RANGE      0x41
#define ICM_REG_GYR_CONF       0x42
#define ICM_REG_GYR_RANGE      0x43
#define ICM_REG_SOFT_RST       0x4A
#define ICM_REG_PWR_CTRL       0x7D

// 期望的 WHO_AM_I 值 (手册 5.1 节)
#define ICM_WHO_AM_I_VAL       0x6A

// 量程定义 (手册 5.38 ACC_RANGE)
#define ICM_ACC_RANGE_2G       0x00 // 00: +/- 2g
#define ICM_ACC_RANGE_4G       0x01 // 01: +/- 4g
#define ICM_ACC_RANGE_8G       0x02 // 10: +/- 8g
#define ICM_ACC_RANGE_16G      0x03 // 11: +/- 16g

// 量程定义 (手册 5.40 GYR_RANGE)
#define ICM_GYR_RANGE_2000DPS  0x00 // 000: +/- 2000dps
#define ICM_GYR_RANGE_1000DPS  0x01 // 001: +/- 1000dps
#define ICM_GYR_RANGE_500DPS   0x02 // 010: +/- 500dps
#define ICM_GYR_RANGE_250DPS   0x03 // 011: +/- 250dps

// ODR 定义 (手册 5.37 ACC_CONF / 5.39 GYR_CONF)
// 这里简化定义，具体值参考手册
#define ICM_ODR_1600HZ         0x00
#define ICM_ODR_800HZ          0x01
#define ICM_ODR_400HZ          0x02
#define ICM_ODR_200HZ          0x03
#define ICM_ODR_100HZ          0x04
#define ICM_ODR_50HZ           0x05
#define ICM_ODR_25HZ           0x06

// 数据结构
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} icm42688RawData_t;

typedef struct {
  float x;
  float y;
  float z;
} icm42688RealData_t;

// 函数声明
int8_t bsp_Icm42688Init(void);
int8_t bsp_IcmGetTemperature(int16_t* pTemp);
int8_t bsp_IcmGetAccelerometer(icm42688RawData_t *accData);
int8_t bsp_IcmGetGyroscope(icm42688RawData_t *GyroData);
int8_t bsp_IcmGetRawData(icm42688RealData_t *accData, icm42688RealData_t *GyroData);

#endif
#endif