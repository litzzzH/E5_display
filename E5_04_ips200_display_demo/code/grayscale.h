/*
 * grayscale.h
 * 八路红外循迹传感器
 * 从左到右: S1-B12, S2-B8, S3-B13, S4-B9,
 *           S5-B23, S6-B21, S7-B22, S8-B17
 * 实测黑线输出高电平，gray_result 中 1 表示检测到黑线。
 */

#ifndef CODE_GRAYSCALE_H_
#define CODE_GRAYSCALE_H_

#include "zf_common_headfile.h"

#define GRAY_S1             B12
#define GRAY_S2             B8
#define GRAY_S3             B13
#define GRAY_S4             B9
#define GRAY_S5             B23
#define GRAY_S6             B21
#define GRAY_S7             B22
#define GRAY_S8             B17

#define GRAY_CHANNEL_COUNT  8

extern uint8 gray_result[GRAY_CHANNEL_COUNT];
extern uint8 gray_result_pack;

void  gray_init(void);
void  gray_read(void);
uint8 gray_read_pack(void);
void  gray_display_result(uint16 x, uint16 y);

#endif /* CODE_GRAYSCALE_H_ */
