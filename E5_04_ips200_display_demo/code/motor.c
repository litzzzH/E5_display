/*
 * motor.c
 * 电机驱动模块 - MSPM0G3507 移植版
 */

#include "motor.h"

void Motor_Init(void)
{
    gpio_init(MOTOR_DIR1, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR_DIR2, GPO, 0, GPO_PUSH_PULL);
    pwm_init(MOTOR_PWM1, MOTOR_FREQUENCY, 0);
    pwm_init(MOTOR_PWM2, MOTOR_FREQUENCY, 0);
}

void Motor_Setpwm_L(int duty)
{
    if(duty > PWM_MAX)
        duty = PWM_MAX;
    if(duty < -PWM_MAX)
        duty = -PWM_MAX;

    if(duty >= 0)
    {
        gpio_set_level(MOTOR_DIR2, 1);
        pwm_set_duty(MOTOR_PWM2, duty);
    }
    else
    {
        gpio_set_level(MOTOR_DIR2, 0);
        pwm_set_duty(MOTOR_PWM2, -duty);
    }
}

void Motor_Setpwm_R(int duty)
{
    if(duty > PWM_MAX)
        duty = PWM_MAX;
    if(duty < -PWM_MAX)
        duty = -PWM_MAX;

    if(duty >= 0)
    {
        gpio_set_level(MOTOR_DIR1, 1);
        pwm_set_duty(MOTOR_PWM1, duty);
    }
    else
    {
        gpio_set_level(MOTOR_DIR1, 0);
        pwm_set_duty(MOTOR_PWM1, -duty);
    }
}

void Motor_Stop(void)
{
    pwm_set_duty(MOTOR_PWM1, 0);
    pwm_set_duty(MOTOR_PWM2, 0);
}
