/*
 * tracking.h
 * 八路红外循迹控制
 */

#ifndef CODE_TRACKING_H_
#define CODE_TRACKING_H_

#include "zf_common_headfile.h"
#include "grayscale.h"
#include "motor.h"
#include "system_tick.h"

typedef enum
{
    TRACK_TASK_Q2 = 0,
    TRACK_TASK_BALL,
    TRACK_TASK_COUNT,
} track_task_mode_e;

typedef struct
{
    int16  base_pwm_l;
    int16  base_pwm_r;
    int16  kp;
    int16  kd;
    uint16 action_start_ms;
    uint16 slow_stop_ms;
    uint16 startup_ms;
    uint8  startup_min_pct;
} track_control_profile_t;

typedef struct
{
    uint8  tracking_enabled;
    uint8  timer_enabled;
    uint8  finish_find_speed_pct;
    track_task_mode_e task_mode;
    uint16 line_lost_stop_ms;
    track_control_profile_t control_profile[TRACK_TASK_COUNT];
} track_runtime_config_t;

typedef enum
{
    TRACK_DIR_CENTER = 0,
    TRACK_DIR_LEFT,
    TRACK_DIR_RIGHT,
} track_dir_e;

extern track_runtime_config_t track_cfg;
extern track_dir_e track_last_dir;
extern uint8 track_finished;
extern int16 track_error;
extern int16 track_correction;
extern uint32 track_elapsed_ms;

void tracking_init(void);
void tracking_update(void);
void tracking_display(uint16 x, uint16 y);
void tracking_reset_runtime_state(void);
void tracking_pause_runtime(void);
void tracking_timer_config_changed(void);

#endif /* CODE_TRACKING_H_ */
