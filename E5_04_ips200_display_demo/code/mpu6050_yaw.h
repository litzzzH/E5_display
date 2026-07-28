/*
 * mpu6050_yaw.h
 * MPU6050 Z轴角度积分模块 - MSPM0G3507 移植版
 */

#ifndef CODE_MPU6050_YAW_H_
#define CODE_MPU6050_YAW_H_

#include "zf_common_headfile.h"
#include "mpu6050.h"

// MPU6050 采样周期 200Hz → dt = 5ms = 0.005s
#define MPU6050_SAMPLE_DT   (0.005f)

extern float mpu6050_yaw;
extern float mpu6050_gyro_z_dps;
extern float mpu6050_gyro_z_offset;

uint8 mpu6050_yaw_init(void);
void  mpu6050_yaw_update(void);
float mpu6050_yaw_get(void);
void  mpu6050_yaw_reset(void);
void  mpu6050_yaw_display(uint16 x, uint16 y);

#endif /* CODE_MPU6050_YAW_H_ */
