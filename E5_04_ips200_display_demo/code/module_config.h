/*
 * module_config.h
 * 模块快捷开关 - 设置 0 关闭, 1 开启
 */

#ifndef CODE_MODULE_CONFIG_H_
#define CODE_MODULE_CONFIG_H_

#define ENABLE_MOTOR        1       // 电机模块
#define ENABLE_GRAYSCALE    1       // 灰度传感器模块
#define ENABLE_IMU          0       // IMU660RB 姿态传感器模块
#define ENABLE_TRACKING     1       // 循迹模块 (依赖电机+灰度+IMU)
#define ENABLE_LASER        0       // 激光模块

#endif /* CODE_MODULE_CONFIG_H_ */
