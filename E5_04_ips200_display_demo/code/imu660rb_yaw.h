/*
 * imu660rb_yaw.h
 * IMU660RB Z轴角度积分模块
 * 使用逐飞库 zf_device_imu660rb 驱动，SPI通信
 * 引脚: SCL-B23, SDA-B22, SDO-B21, CS-B19（库默认值）
 */

#ifndef CODE_IMU660RB_YAW_H_
#define CODE_IMU660RB_YAW_H_

#include "zf_common_headfile.h"
#include "zf_device_imu660rb.h"

// 采样周期 200Hz → dt = 5ms = 0.005s
#define IMU660RB_SAMPLE_DT   (0.005f)

// 零飘校准采样数量
#define IMU660RB_CALIBRATION_SAMPLES  1000

// 死区阈值(°/s)：角速度绝对值小于此值时视为0，防止零飘积分
#define IMU660RB_GYRO_DEADZONE   (1.0f)

// yaw更新PIT定时器（system_tick已用G0）
#define IMU660RB_PIT_TIMER   PIT_TIM_G6

extern volatile float imu660rb_yaw;              // 当前yaw角（度）
extern volatile float imu660rb_gyro_z_dps;       // Z轴角速度（°/s）
extern float imu660rb_gyro_z_offset;    // Z轴零飘偏差

uint8 imu660rb_yaw_init(void);                  // 初始化IMU660RB并校准零飘，启动PIT
void  imu660rb_yaw_update(void);                // 读取陀螺仪并更新yaw（PIT回调内调用）
float imu660rb_yaw_get(void);                   // 获取当前yaw角度
void  imu660rb_yaw_reset(void);                 // yaw角归零
void  imu660rb_yaw_display(uint16 x, uint16 y); // 屏幕显示角度信息

#endif /* CODE_IMU660RB_YAW_H_ */
