#ifndef ICM20602_SPI_H
#define ICM20602_SPI_H
#include "management.h"
#include <stdbool.h>
#include <string.h>

#define ICM20602_XG_OFFSET_TC_H     0x04
#define ICM20602_XG_OFFSET_TC_L     0x05
#define ICM20602_YG_OFFSET_TC_H     0x07
#define ICM20602_YG_OFFSET_TC_L     0x08
#define ICM20602_ZG_OFFSET_TC_H     0x0A
#define ICM20602_ZG_OFFSET_TC_L     0x0B

#define ICM20602_SELF_TEST_X_ACCEL  0x0D
#define ICM20602_SELF_TEST_Y_ACCEL  0x0E    
#define ICM20602_SELF_TEST_Z_ACCEL  0x0F
//#define SELF_TEST_A               0x10

#define ICM20602_XG_OFFS_USRH       0x13  // User-defined trim values for gyroscope
#define ICM20602_XG_OFFS_USRL       0x14
#define ICM20602_YG_OFFS_USRH       0x15
#define ICM20602_YG_OFFS_USRL       0x16
#define ICM20602_ZG_OFFS_USRH       0x17
#define ICM20602_ZG_OFFS_USRL       0x18
#define ICM20602_SMPLRT_DIV         0x19
#define ICM20602_CONFIG             0x1A
#define ICM20602_GYRO_CONFIG        0x1B
#define ICM20602_ACCEL_CONFIG       0x1C
#define ICM20602_ACCEL_CONFIG2      0x1D  // Free-fall
#define ICM20602_LP_MODE_CFG        0x1E  // Free-fall
#define ICM20602_ACCEL_WOM_THR      0x1F  // Motion detection threshold bits [7:0]
//#define MOT_DUR                   0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
//#define ZMOT_THR                  0x21  // Zero-motion detection threshold bits [7:0]
//#define ZRMOT_DUR                 0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define ICM20602_FIFO_EN            0x23
//#define I2C_MST_CTRL              0x24
//#define I2C_SLV0_ADDR             0x25
//#define I2C_SLV0_REG              0x26
//#define I2C_SLV0_CTRL             0x27
//#define I2C_SLV1_ADDR             0x28
//#define I2C_SLV1_REG              0x29
//#define I2C_SLV1_CTRL             0x2A
//#define I2C_SLV2_ADDR             0x2B
//#define I2C_SLV2_REG              0x2C
//#define I2C_SLV2_CTRL             0x2D
//#define I2C_SLV3_ADDR             0x2E
//#define I2C_SLV3_REG              0x2F
//#define I2C_SLV3_CTRL             0x30
//#define I2C_SLV4_ADDR             0x31
//#define I2C_SLV4_REG              0x32
//#define I2C_SLV4_DO               0x33
//#define I2C_SLV4_CTRL             0x34
//#define I2C_SLV4_DI               0x35
#define ICM20602_FSYNC_INT          0x36
#define ICM20602_INT_PIN_CFG        0x37
#define ICM20602_INT_ENABLE         0x38
//#define DMP_INT_STATUS            0x39  // Check DMP interrupt
#define ICM20602_INT_STATUS         0x3A
#define ICM20602_ACCEL_XOUT_H       0x3B
#define ICM20602_ACCEL_XOUT_L       0x3C
#define ICM20602_ACCEL_YOUT_H       0x3D
#define ICM20602_ACCEL_YOUT_L       0x3E
#define ICM20602_ACCEL_ZOUT_H       0x3F
#define ICM20602_ACCEL_ZOUT_L       0x40
#define ICM20602_TEMP_OUT_H         0x41
#define ICM20602_TEMP_OUT_L         0x42
#define ICM20602_GYRO_XOUT_H        0x43
#define ICM20602_GYRO_XOUT_L        0x44
#define ICM20602_GYRO_YOUT_H        0x45
#define ICM20602_GYRO_YOUT_L        0x46
#define ICM20602_GYRO_ZOUT_H        0x47
#define ICM20602_GYRO_ZOUT_L        0x48
//#define EXT_SENS_DATA_00          0x49
//#define EXT_SENS_DATA_01          0x4A
//#define EXT_SENS_DATA_02          0x4B
//#define EXT_SENS_DATA_03          0x4C
//#define EXT_SENS_DATA_04          0x4D
//#define EXT_SENS_DATA_05          0x4E
//#define EXT_SENS_DATA_06          0x4F
#define ICM20602_SELF_TEST_X_GYRO   0x50
#define ICM20602_SELF_TEST_y_GYRO   0x51
#define ICM20602_SELF_TEST_z_GYRO   0x52
//#define EXT_SENS_DATA_10          0x53
//#define EXT_SENS_DATA_11          0x54
//#define EXT_SENS_DATA_12          0x55
//#define EXT_SENS_DATA_13          0x56
//#define EXT_SENS_DATA_14          0x57
//#define EXT_SENS_DATA_15          0x58
//#define EXT_SENS_DATA_16          0x59
//#define EXT_SENS_DATA_17          0x5A
//#define EXT_SENS_DATA_18          0x5B
//#define EXT_SENS_DATA_19          0x5C
//#define EXT_SENS_DATA_20          0x5D
//#define EXT_SENS_DATA_21          0x5E
//#define EXT_SENS_DATA_22          0x5F
//#define EXT_SENS_DATA_23          0x60
//#define MOT_DETECT_STATUS         0x61
//#define I2C_SLV0_DO               0x63
//#define I2C_SLV1_DO               0x64
//#define I2C_SLV2_DO               0x65
//#define I2C_SLV3_DO               0x66
//#define I2C_MST_DELAY_CTRL        0x67
#define ICM20602_SIGNAL_PATH_RESET  0x68
#define ICM20602_ACCEL_INTEL_CTRL   0x69
#define ICM20602_USER_CTRL          0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define ICM20602_PWR_MGMT_1         0x6B  // Device defaults to the SLEEP mode
#define ICM20602_PWR_MGMT_2         0x6C
//#define DMP_BANK                  0x6D  // Activates a specific bank in the DMP
//#define DMP_RW_PNT                0x6E  // Set read/write pointer to a specific start address in specified DMP bank
//#define DMP_REG                   0x6F  // Register in DMP from which to read or to which to write
//#define DMP_REG_1                 0x70
//#define DMP_REG_2                 0x71
#define ICM20602_FIFO_COUNTH        0x72
#define ICM20602_FIFO_COUNTL        0x73
#define ICM20602_FIFO_R_W           0x74
#define ICM20602_WHO_AM_I           0x75 // Should return 0x68

#define ICM20602_ACCEL_XOFFSET_H    0x77
#define ICM20602_ACCEL_XOFFSET_L    0x78
#define ICM20602_ACCEL_YOFFSET_H    0x7A
#define ICM20602_ACCEL_YOFFSET_L    0x7B
#define ICM20602_ACCEL_ZOFFSET_H    0x7D
#define ICM20602_ACCEL_ZOFFSET_L    0x7E

// Using the GY-521 breakout board, I set ADO to 0 by grounding through a 4k7 resistor
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0
#if ADO
#define ICM20602_slave_addr 0x69<<1  // Device address when ADO = 1
#else
#define ICM20602_slave_addr 0x68<<1  // Device address when ADO = 0
#endif

#define IMU_ONE_G 9.80665f
#define SHRINK_ERROR 0.0001

uint16_t ICM20602_getWhoAmI();
void    ICM20602_init();
int16_t ICM20602_getAccXvalue();
int16_t ICM20602_getAccYvalue();
int16_t ICM20602_getAccZvalue();
int16_t ICM20602_getGyrXvalue();
int16_t ICM20602_getGyrYvalue();
int16_t ICM20602_getGyrZvalue();
int16_t ICM20602_getIMUTemp();
float   ICM20602_setAccRange(int Ascale);
float   ICM20602_setGyroRange(int Gscale);

int     ICM20602_getAccRange(void);
int     ICM20602_getGyroRange(void);

void ICM20602_writeByte(uint8_t, uint8_t);
uint8_t ICM20602_readByte(uint8_t);

void ICM20602_read_IMU_data(float imu_dt_sec, float * yaw_input);
void ICM20602_IMU_calibration();
void ICM20602_IMU_calibration2();
void ICM20602_IMU_compensate();

float ICM20602_integral(float val, float val_prv, float dt);
void  ICM20602_clearAngle(void);
void  ICM20602_setAngle(float pitch, float roll, float yaw);
float ICM20602_normAngle(float deg);

float ICM20602_complementaryWeight(float first, float second);
float ICM20602_complementaryFilter(float val);

void ICM20602_medianFilter(void);
#endif
