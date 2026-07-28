/*
 * grayscale.h
 * 五路灰度传感器模块 - MSPM0G3507 移植版
 * 引脚: OUT1-B17, OUT2-B9, OUT3-B13, OUT4-B8, OUT5-B12
 */

#ifndef CODE_GRAYSCALE_H_
#define CODE_GRAYSCALE_H_

#include "zf_common_headfile.h"

#define GRAY_OUT1           B17
#define GRAY_OUT2           B9
#define GRAY_OUT3           B13
#define GRAY_OUT4           B8
#define GRAY_OUT5           B12

#define GRAY_CHANNEL_COUNT  5

extern uint8 gray_result[GRAY_CHANNEL_COUNT];
extern uint8 gray_result_pack;

void  gray_init(void);
void  gray_read(void);
uint8 gray_read_pack(void);
void  gray_display_result(uint16 x, uint16 y);

#endif /* CODE_GRAYSCALE_H_ */
