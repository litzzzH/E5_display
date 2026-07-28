/*
 * motor.h
 * 电机驱动模块 - MSPM0G3507 移植版
 * 引脚: PWM1-A27, PWM2-A26, DIR1-B11, DIR2-B10
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"

#define MOTOR_DIR1          B11
#define MOTOR_DIR2          B10
#define MOTOR_PWM1          PWM_TIM_G7_CH1_A27
#define MOTOR_PWM2          PWM_TIM_G7_CH0_A26

#define MOTOR_FREQUENCY     17000                   // 电机PWM频率
#define PWM_MAX             5000                    // PWM占空比最大值

void Motor_Init(void);
void Motor_Setpwm_L(int duty);
void Motor_Setpwm_R(int duty);
void Motor_Stop(void);

#endif /* CODE_MOTOR_H_ */
