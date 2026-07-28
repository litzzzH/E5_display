/*
 * tracking.h
 * 循迹模块 - MSPM0G3507 移植版（陀螺仪直角弯）
 */

#ifndef CODE_TRACKING_H_
#define CODE_TRACKING_H_

#include "zf_common_headfile.h"
#include "grayscale.h"
#include "motor.h"
#include "system_tick.h"
#include "imu660rb_yaw.h"

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
    uint16 turn_prepare_ms;
    uint16 turn_pwm;
    uint16 turn_pwm_min;
    uint16 turn_accel_ms;
    uint8  turn_accel_min_pct;
    uint16 startup_ms;
    uint8  startup_min_pct;
} track_speed_profile_t;

typedef struct
{
    uint8  tracking_enabled;
    track_speed_mode_e speed_mode;
    uint8  turns_per_lap;
    uint8  total_laps;
    uint16 turn_angle_x10;
    uint16 turn_slow_deg_x10;
    uint16 turn_brake_deg_x10;
    uint16 turn_m_exit_deg_x10;
    uint8  ra_persist_cycles;
    uint16 turn_cooldown_ms;
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

// --- 直角弯（陀螺仪方式） ---
#define TRACK_TURN_PREPARE_MS      (TRACK_ACTIVE_SPEED_CFG.turn_prepare_ms)
#define TRACK_TURN_PWM             (TRACK_ACTIVE_SPEED_CFG.turn_pwm)
#define TRACK_TURN_PWM_MIN         (TRACK_ACTIVE_SPEED_CFG.turn_pwm_min)
#define TRACK_TURN_ANGLE           ((float)track_cfg.turn_angle_x10 / 10.0f)
#define TRACK_TURN_SLOW_DEG        ((float)track_cfg.turn_slow_deg_x10 / 10.0f)
#define TRACK_TURN_BRAKE_DEG       ((float)track_cfg.turn_brake_deg_x10 / 10.0f)
#define TRACK_TURN_M_EXIT_DEG      ((float)track_cfg.turn_m_exit_deg_x10 / 10.0f)
#define TRACK_TURN_ACCEL_MS        (TRACK_ACTIVE_SPEED_CFG.turn_accel_ms)
#define TRACK_TURN_ACCEL_MIN       ((float)TRACK_ACTIVE_SPEED_CFG.turn_accel_min_pct / 100.0f)

// --- 启动线性加速 ---
#define TRACK_STARTUP_MS           (TRACK_ACTIVE_SPEED_CFG.startup_ms)
#define TRACK_STARTUP_MIN          ((float)TRACK_ACTIVE_SPEED_CFG.startup_min_pct / 100.0f)

// --- 计圈停车 ---
#define TRACK_TURNS_PER_LAP        (track_cfg.turns_per_lap)
#define TRACK_TOTAL_LAPS           (track_cfg.total_laps)

// --- 直角弯检测辅助 ---
#define TRACK_RA_PERSIST_CYCLES    (track_cfg.ra_persist_cycles)
#define TRACK_TURN_COOLDOWN_MS     (track_cfg.turn_cooldown_ms)

// --- 丢线停车保护 ---
#define TRACK_LINE_LOST_STOP_MS    (track_cfg.line_lost_stop_ms)

// ======================== 可配置参数区 END ========================

typedef enum
{
    TRACK_STATE_NORMAL = 0,
    TRACK_STATE_RIGHT_ANGLE,
} track_state_e;

typedef enum
{
    TRACK_DIR_CENTER = 0,
    TRACK_DIR_LEFT,
    TRACK_DIR_RIGHT,
} track_dir_e;

extern track_state_e track_state;
extern track_dir_e   track_last_dir;
extern uint8         track_turning;
extern uint16        track_turn_count;   // 已完成直角弯数
extern uint8         track_lap_count;    // 已完成圈数
extern uint8         track_finished;     // 停车标志

void tracking_init(void);
void tracking_update(void);
void tracking_display(uint16 x, uint16 y);
void tracking_reset_runtime_state(void);

#endif /* CODE_TRACKING_H_ */
