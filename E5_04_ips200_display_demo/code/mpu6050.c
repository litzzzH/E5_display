/*
 * mpu6050.c
 * MPU6050 设备驱动 - MSPM0G3507 移植版
 * 使用逐飞库 soft_iic 驱动
 */

#include "mpu6050.h"

int16 mpu6050_gyro_x = 0, mpu6050_gyro_y = 0, mpu6050_gyro_z = 0;
int16 mpu6050_acc_x  = 0, mpu6050_acc_y  = 0, mpu6050_acc_z  = 0;

static soft_iic_info_struct mpu6050_iic_struct;

#define mpu6050_write_register(reg, data)       (soft_iic_write_8bit_register(&mpu6050_iic_struct, (reg), (data)))
#define mpu6050_read_register(reg)              (soft_iic_read_8bit_register(&mpu6050_iic_struct, (reg)))
#define mpu6050_read_registers(reg, data, len)  (soft_iic_read_8bit_registers(&mpu6050_iic_struct, (reg), (data), (len)))

static uint8 mpu6050_self_check(void)
{
    uint8 dat = 0, return_state = 0;
    uint16 timeout_count = 0;

    mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00);
    mpu6050_write_register(MPU6050_SMPLRT_DIV, 0x07);

    while(0x07 != dat)
    {
        if(MPU6050_TIMEOUT_COUNT < timeout_count++)
        {
            return_state = 1;
            break;
        }
        dat = mpu6050_read_register(MPU6050_SMPLRT_DIV);
        system_delay_ms(10);
    }
    return return_state;
}

void mpu6050_get_acc(void)
{
    uint8 dat[6];
    mpu6050_read_registers(MPU6050_ACCEL_XOUT_H, dat, 6);
    mpu6050_acc_x = (int16)(((uint16)dat[0] << 8) | dat[1]);
    mpu6050_acc_y = (int16)(((uint16)dat[2] << 8) | dat[3]);
    mpu6050_acc_z = (int16)(((uint16)dat[4] << 8) | dat[5]);
}

void mpu6050_get_gyro(void)
{
    uint8 dat[6];
    mpu6050_read_registers(MPU6050_GYRO_XOUT_H, dat, 6);
    mpu6050_gyro_x = (int16)(((uint16)dat[0] << 8) | dat[1]);
    mpu6050_gyro_y = (int16)(((uint16)dat[2] << 8) | dat[3]);
    mpu6050_gyro_z = (int16)(((uint16)dat[4] << 8) | dat[5]);
}

float mpu6050_acc_transition(int16 acc_value)
{
    float acc_data = 0;
    switch(MPU6050_ACC_SAMPLE)
    {
        case 0x00: acc_data = (float)acc_value / 16384; break;
        case 0x08: acc_data = (float)acc_value / 8192;  break;
        case 0x10: acc_data = (float)acc_value / 4096;  break;
        case 0x18: acc_data = (float)acc_value / 2048;  break;
        default: break;
    }
    return acc_data;
}

float mpu6050_gyro_transition(int16 gyro_value)
{
    float gyro_data = 0;
    switch(MPU6050_GYR_SAMPLE)
    {
        case 0x00: gyro_data = (float)gyro_value / 131.0f;  break;
        case 0x08: gyro_data = (float)gyro_value / 65.5f;   break;
        case 0x10: gyro_data = (float)gyro_value / 32.8f;   break;
        case 0x18: gyro_data = (float)gyro_value / 16.4f;   break;
        default: break;
    }
    return gyro_data;
}

uint8 mpu6050_init(void)
{
    uint8 return_state = 0;
    uint8 who_am_i = 0;

    soft_iic_init(&mpu6050_iic_struct, MPU6050_DEV_ADDR, MPU6050_SOFT_IIC_DELAY,
                  MPU6050_SCL_PIN, MPU6050_SDA_PIN);
    system_delay_ms(100);

    // 1. 硬件复位 MPU6050
    mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x80);   // DEVICE_RESET
    system_delay_ms(100);                                // 等待复位完成

    // 2. 读 WHO_AM_I 验证 I2C 通信
    who_am_i = mpu6050_read_register(MPU6050_WHO_AM_I);
    // 屏幕显示调试信息
    ips200_show_string(0, 30, "WHOAMI:0x");
    ips200_show_uint(54, 30, who_am_i, 3);

    if(who_am_i != 0x68 && who_am_i != 0x98)
    {
        // 尝试备用地址 0x69 (AD0=HIGH)
        mpu6050_iic_struct.addr = 0x69;
        mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x80);
        system_delay_ms(100);
        who_am_i = mpu6050_read_register(MPU6050_WHO_AM_I);
        ips200_show_string(0, 40, "0x69:0x");
        ips200_show_uint(48, 40, who_am_i, 3);
        if(who_am_i != 0x68 && who_am_i != 0x98)
        {
            mpu6050_iic_struct.addr = MPU6050_DEV_ADDR;
            return 1;
        }
    }

    // 3. 解除休眠，选择时钟源
    mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x01);   // 选用PLL X轴陀螺仪作时钟源
    system_delay_ms(50);

    do {
        if(mpu6050_self_check())
        {
            return_state = 1;
            break;
        }
        mpu6050_write_register(MPU6050_SMPLRT_DIV, 0x07);           // 125Hz采样率
        mpu6050_write_register(MPU6050_CONFIG, 0x04);               // 低通滤波
        mpu6050_write_register(MPU6050_GYRO_CONFIG, MPU6050_GYR_SAMPLE);
        mpu6050_write_register(MPU6050_ACCEL_CONFIG, MPU6050_ACC_SAMPLE);
        mpu6050_write_register(MPU6050_USER_CONTROL, 0x00);
        mpu6050_write_register(MPU6050_INT_PIN_CFG, 0x02);
    } while(0);

    return return_state;
}
