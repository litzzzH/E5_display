/*********************************************************************************************************************
* MSPM0G3507 Opensource Library 即（MSPM0G3507 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MSPM0G3507 开源库的一部分
* 
* MSPM0G3507 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          mian
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK 5.37
* 适用平台          MSPM0G3507
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2025-06-1        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "module_config.h"
#include "motor.h"
#include "grayscale.h"
#include "tracking.h"
#include "tuning_ui.h"
#include "system_tick.h"
#include "imu660rb_yaw.h"
#include "laser.h"

// *************************** 循迹小车 MSPM0G3507 移植 ***************************
// 硬件引脚分配:
//   电机: PWM1-A27, PWM2-A26, DIR1-B11, DIR2-B10
//   八路红外: S1-B12, S2-B8, S3-B13, S4-B9, S5-B23, S6-B21, S7-B22, S8-B17
//   IMU 已关闭，其原有 B23/B21/B22 引脚用于八路红外传感器
//   激光: A0
//   显示屏: SCL-A12, SDA-A9, RES-A7, DC-A15, CS-A8, BLK-A13 (库默认)
//
// 模块开关见 module_config.h

// **************************** 代码区域 ****************************

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>
    debug_init();                   // 调试串口信息初始化

    // ===== 初始化系统毫秒计时器 =====
    system_tick_init();

#if ENABLE_MOTOR
    // ===== 初始化电机 =====
    Motor_Init();
#endif

    // ===== 初始化IPS200显示屏 (SPI模式, 引脚使用库默认值) =====
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE_SPI);
    ips200_clear();
    ips200_set_font(IPS200_6X8_FONT);

#if ENABLE_GRAYSCALE
    // ===== 初始化八路红外循迹传感器 =====
    gray_init();
#endif

#if ENABLE_IMU
    // ===== 初始化 IMU660RB 陀螺仪 =====
    ips200_show_string(0, 0, "IMU Init...");
    if(imu660rb_yaw_init())
    {
        ips200_show_string(0, 10, "IMU FAIL!");
        system_delay_ms(2000);
    }
    else
    {
        ips200_show_string(0, 10, "IMU OK!");
        system_delay_ms(500);
    }
    ips200_clear();
#endif

#if ENABLE_TRACKING
    // ===== 初始化循迹模块 =====
    tracking_init();
    tuning_ui_init();
#endif

#if ENABLE_LASER
    // ===== 初始化激光模块 =====
    laser_init();
    laser_on();
#endif

    ips200_show_string(0, 0, "Ready!");
    system_delay_ms(500);
    ips200_clear();

    while(true)
    {
#if ENABLE_TRACKING
    tuning_ui_update();

        // 循迹控制（读八路红外 + 驱动电机）
        tracking_update();
#endif

        // 显示信息
        ips200_show_string(0, 0, "Tracking Car");
#if ENABLE_GRAYSCALE
        gray_display_result(0, 16);
#endif
#if ENABLE_TRACKING
        tracking_display(0, 50);
    tuning_ui_display(0, 96);
#endif
#if ENABLE_IMU
        imu660rb_yaw_display(0, 64);
#endif
#if ENABLE_LASER
        ips200_show_string(0, 80, "LASER: ON ");
#endif

        system_delay_ms(10);
    }
}

// **************************** 代码区域 ****************************
