/*
 * mpu6050.h
 * MPU6050 设备驱动 - MSPM0G3507 移植版
 * 使用软件I2C通信，引脚: SCL-B23, SDA-B22
 */

#ifndef CODE_MPU6050_H_
#define CODE_MPU6050_H_

#include "zf_common_headfile.h"

// ========== 软件 IIC 引脚配置 ==========
#define MPU6050_SOFT_IIC_DELAY      (100)       // 库内设备用10~50，MPU6050稍慢一点用100
#define MPU6050_SCL_PIN             B23
#define MPU6050_SDA_PIN             B22

#define MPU6050_TIMEOUT_COUNT       (200)       // 超时计数（200×10ms=2s）

// ========== MPU6050 寄存器地址 ==========
#define MPU6050_DEV_ADDR            (0x68)      // IIC 设备地址 7位
#define MPU6050_SMPLRT_DIV          (0x19)
#define MPU6050_CONFIG              (0x1A)
#define MPU6050_GYRO_CONFIG         (0x1B)
#define MPU6050_ACCEL_CONFIG        (0x1C)
#define MPU6050_INT_PIN_CFG         (0x37)
#define MPU6050_ACCEL_XOUT_H        (0x3B)
#define MPU6050_GYRO_XOUT_H         (0x43)
#define MPU6050_USER_CONTROL        (0x6A)
#define MPU6050_PWR_MGMT_1          (0x6B)
#define MPU6050_WHO_AM_I            (0x75)

#define MPU6050_ACC_SAMPLE          (0x10)      // 加速度计量程 ±8g
#define MPU6050_GYR_SAMPLE          (0x18)      // 陀螺仪量程 ±2000dps

// ========== 全局变量 ==========
extern int16 mpu6050_gyro_x, mpu6050_gyro_y, mpu6050_gyro_z;
extern int16 mpu6050_acc_x,  mpu6050_acc_y,  mpu6050_acc_z;

// ========== API ==========
void    mpu6050_get_acc         (void);
void    mpu6050_get_gyro        (void);
float   mpu6050_acc_transition  (int16 acc_value);
float   mpu6050_gyro_transition (int16 gyro_value);
uint8   mpu6050_init            (void);

#endif /* CODE_MPU6050_H_ */
