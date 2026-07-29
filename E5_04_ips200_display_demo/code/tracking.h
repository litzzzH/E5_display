/*
 * tracking.h
 * 循迹模块 - MSPM0G3507 移植版
 */

#ifndef CODE_TRACKING_H_
#define CODE_TRACKING_H_

#include "zf_common_headfile.h"
#include "grayscale.h"
#include "motor.h"
#include "system_tick.h"

// ======================== 可配置参数区 ========================

// --- 灰度通道索引（从左到右） ---
#define IDX_L2  0
#define IDX_L1  1
#define IDX_M   2
#define IDX_R1  3
#define IDX_R2  4

typedef enum
{
    TRACK_SPEED_MODE_LOW = 0,
    TRACK_SPEED_MODE_HIGH,
    TRACK_SPEED_MODE_COUNT,
} track_speed_mode_e;

typedef struct
{
    int16  base_pwm_l;
    int16  base_pwm_r;
    int16  diff_small;
    int16  diff_med;
    int16  diff_large;
    uint16 startup_ms;
    uint8  startup_min_pct;
} track_speed_profile_t;

typedef struct
{
    uint8  tracking_enabled;
    track_speed_mode_e speed_mode;
    uint16 line_lost_stop_ms;
    track_speed_profile_t speed_profile[TRACK_SPEED_MODE_COUNT];
} track_runtime_config_t;

extern track_runtime_config_t track_cfg;

#define TRACK_ACTIVE_SPEED_CFG     (track_cfg.speed_profile[track_cfg.speed_mode])

// --- 普通循迹 ---
#define TRACK_BASE_PWM_L           (TRACK_ACTIVE_SPEED_CFG.base_pwm_l)
#define TRACK_BASE_PWM_R           (TRACK_ACTIVE_SPEED_CFG.base_pwm_r)
#define TRACK_DIFF_SMALL           (TRACK_ACTIVE_SPEED_CFG.diff_small)
#define TRACK_DIFF_MED             (TRACK_ACTIVE_SPEED_CFG.diff_med)
#define TRACK_DIFF_LARGE           (TRACK_ACTIVE_SPEED_CFG.diff_large)

// --- 启动线性加速 ---
#define TRACK_STARTUP_MS           (TRACK_ACTIVE_SPEED_CFG.startup_ms)
#define TRACK_STARTUP_MIN          ((float)TRACK_ACTIVE_SPEED_CFG.startup_min_pct / 100.0f)

// --- 丢线停车保护 ---
#define TRACK_LINE_LOST_STOP_MS    (track_cfg.line_lost_stop_ms)

// ======================== 可配置参数区 END ========================

typedef enum
{
    TRACK_DIR_CENTER = 0,
    TRACK_DIR_LEFT,
    TRACK_DIR_RIGHT,
} track_dir_e;

extern track_dir_e   track_last_dir;
extern uint8         track_finished;     // 停车标志

void tracking_init(void);
void tracking_update(void);
void tracking_display(uint16 x, uint16 y);
void tracking_reset_runtime_state(void);

#endif /* CODE_TRACKING_H_ */
