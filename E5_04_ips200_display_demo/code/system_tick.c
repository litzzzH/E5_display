/*
 * system_tick.c
 * 系统毫秒计时器 - MSPM0G3507 移植版
 * 使用 PIT_TIM_G0 提供1ms中断计数
 */

#include "system_tick.h"

static volatile uint32 system_ms_counter = 0;

// PIT中断回调函数
static void system_tick_pit_callback(uint32 flag, void *ptr)
{
    (void)flag;
    (void)ptr;
    system_ms_counter++;
}

void system_tick_init(void)
{
    system_ms_counter = 0;
    pit_ms_init(PIT_TIM_G0, 1, system_tick_pit_callback, NULL);
    pit_enable(PIT_TIM_G0);
}

void system_tick_handler(void)
{
    system_ms_counter++;
}

uint32 system_getval_ms(void)
{
    return system_ms_counter;
}
