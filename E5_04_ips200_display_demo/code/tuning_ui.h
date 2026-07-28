/*
 * tuning_ui.h
 * 板载按键 UI 调参模块
 */

#ifndef CODE_TUNING_UI_H_
#define CODE_TUNING_UI_H_

#include "zf_common_headfile.h"

void tuning_ui_init(void);
void tuning_ui_update(void);
void tuning_ui_display(uint16 x, uint16 y);

#endif /* CODE_TUNING_UI_H_ */