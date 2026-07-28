/*
 * system_tick.h
 * 系统毫秒计时器 - MSPM0G3507 移植版
 * 提供 system_getval_ms() 功能（原TC264库自带，MSPM0G3507需自行实现）
 */

#ifndef CODE_SYSTEM_TICK_H_
#define CODE_SYSTEM_TICK_H_

#include "zf_common_headfile.h"

void   system_tick_init(void);          // 初始化1ms PIT定时器
void   system_tick_handler(void);       // 在PIT中断回调中调用
uint32 system_getval_ms(void);          // 获取系统运行毫秒数

#endif /* CODE_SYSTEM_TICK_H_ */
