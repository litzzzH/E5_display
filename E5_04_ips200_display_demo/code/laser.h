/*
 * laser.h
 * 激光模块 - MSPM0G3507
 * 引脚: A0 (GPIO输出, 高电平开启)
 */

#ifndef CODE_LASER_H_
#define CODE_LASER_H_

#include "zf_common_headfile.h"

#define LASER_PIN           A0

void laser_init(void);
void laser_on(void);
void laser_off(void);
void laser_toggle(void);

#endif /* CODE_LASER_H_ */
