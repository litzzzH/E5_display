/*
 * imu660rb_yaw.c
 * IMU660RB Z轴角度积分模块
 * 初始化时自动采集零飘偏移，之后在PIT中断中以5ms周期积分yaw角
 */

#include "imu660rb_yaw.h"
#include <stdio.h>

volatile float imu660rb_yaw          = 0.0f;
volatile float imu660rb_gyro_z_dps   = 0.0f;
float imu660rb_gyro_z_offset = 0.0f;

// PIT中断回调
static void imu660rb_yaw_pit_callback(uint32 flag, void *ptr)
{
    (void)flag;
    (void)ptr;
    imu660rb_yaw_update();
}

//---------------------------------------------------
// 初始化 IMU660RB 并校准Z轴零飘，启动5ms PIT定时器
// 返回: 0-成功  1-失败
// 注意: 校准期间请保持车辆静止
//---------------------------------------------------
uint8 imu660rb_yaw_init(void)
{
    uint8 ret;
    int i;
    float sum = 0.0f;

    ret = imu660rb_init();
    if(ret)
    {
        return 1;
    }

    // 等待传感器稳定
    system_delay_ms(500);

    // 先丢弃前100个读数
    for(i = 0; i < 100; i++)
    {
        imu660rb_get_gyro();
        system_delay_ms(2);
    }

    // 采集零飘
    for(i = 0; i < IMU660RB_CALIBRATION_SAMPLES; i++)
    {
        imu660rb_get_gyro();
        sum += imu660rb_gyro_transition(imu660rb_gyro_z);
        system_delay_ms(2);
    }
    imu660rb_gyro_z_offset = sum / (float)IMU660RB_CALIBRATION_SAMPLES;

    imu660rb_yaw = 0.0f;
    imu660rb_gyro_z_dps = 0.0f;

    // 启动PIT定时器，5ms周期调用yaw更新
    pit_ms_init(IMU660RB_PIT_TIMER, 5, imu660rb_yaw_pit_callback, NULL);
    pit_enable(IMU660RB_PIT_TIMER);

    return 0;
}

//---------------------------------------------------
// 读取陀螺仪并更新yaw角（由PIT中断自动调用）
//---------------------------------------------------
void imu660rb_yaw_update(void)
{
    float gz;
    imu660rb_get_gyro();
    gz = imu660rb_gyro_transition(imu660rb_gyro_z) - imu660rb_gyro_z_offset;

    // 死区滤波：小于阈值的角速度视为0
    if(gz > -IMU660RB_GYRO_DEADZONE && gz < IMU660RB_GYRO_DEADZONE)
        gz = 0.0f;

    imu660rb_gyro_z_dps = gz;
    imu660rb_yaw += gz * IMU660RB_SAMPLE_DT;
}

float imu660rb_yaw_get(void)
{
    return imu660rb_yaw;
}

void imu660rb_yaw_reset(void)
{
    pit_disable(IMU660RB_PIT_TIMER);
    imu660rb_yaw = 0.0f;
    pit_enable(IMU660RB_PIT_TIMER);
}

void imu660rb_yaw_display(uint16 x, uint16 y)
{
    char buf[32];

    ips200_show_string(x, y, "YAW:");
    ips200_show_float(x + 30, y, imu660rb_yaw, 4, 2);

    sprintf(buf, "Gz:%+.2f  ", imu660rb_gyro_z_dps);
    ips200_show_string(x, y + 10, buf);

    sprintf(buf, "Off:%+.3f", imu660rb_gyro_z_offset);
    ips200_show_string(x, y + 20, buf);
}
