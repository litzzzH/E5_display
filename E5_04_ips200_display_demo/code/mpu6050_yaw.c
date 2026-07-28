/*
 * mpu6050_yaw.c
 * MPU6050 Z轴角度积分模块 - MSPM0G3507 移植版
 */

#include "mpu6050_yaw.h"

#define CALIBRATION_SAMPLES  500

float mpu6050_yaw            = 0.0f;
float mpu6050_gyro_z_dps     = 0.0f;
float mpu6050_gyro_z_offset  = 0.0f;

uint8 mpu6050_yaw_init(void)
{
    uint8 ret;
    int i;
    float sum = 0.0f;

    ret = mpu6050_init();
    if(ret)
    {
        return 1;
    }

    system_delay_ms(200);

    // 采集零漂（保持车辆静止）
    for(i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        mpu6050_get_gyro();
        sum += mpu6050_gyro_transition(mpu6050_gyro_z);
        system_delay_ms(2);
    }
    mpu6050_gyro_z_offset = sum / (float)CALIBRATION_SAMPLES;

    mpu6050_yaw = 0.0f;
    mpu6050_gyro_z_dps = 0.0f;

    return 0;
}

void mpu6050_yaw_update(void)
{
    mpu6050_get_gyro();
    mpu6050_gyro_z_dps = mpu6050_gyro_transition(mpu6050_gyro_z) - mpu6050_gyro_z_offset;
    mpu6050_yaw += mpu6050_gyro_z_dps * MPU6050_SAMPLE_DT;
}

float mpu6050_yaw_get(void)
{
    return mpu6050_yaw;
}

void mpu6050_yaw_reset(void)
{
    mpu6050_yaw = 0.0f;
}

void mpu6050_yaw_display(uint16 x, uint16 y)
{
    char buf[32];

    ips200_show_string(x, y, "YAW:");
    ips200_show_float(x + 30, y, mpu6050_yaw, 4, 2);

    sprintf(buf, "Gz:%+.2f  ", mpu6050_gyro_z_dps);
    ips200_show_string(x, y + 10, buf);

    sprintf(buf, "Off:%+.3f", mpu6050_gyro_z_offset);
    ips200_show_string(x, y + 20, buf);
}
