/*
 * tracking.c
 * 八路加权位置与 PD 差速循迹
 *
 * 误差为正表示黑线在车体左侧，误差为负表示黑线在车体右侧。
 * 左右轮始终做等量反向差速，基础速度不随直道或圆弧改变。
 */

#include "tracking.h"

#define TRACK_ERROR_SCALE       10
#define TRACK_MAX_ERROR         70
#define TRACK_FINISH_FIRST_SENSOR  1
#define TRACK_FINISH_LAST_SENSOR   6
#define TRACK_FINISH_MIN_ADJACENT  4
#define TRACK_FINISH_CONFIRM_FRAMES 3

track_runtime_config_t track_cfg = {
    0,
    1,
    20,
    TRACK_TASK_Q2,
    1000,
    {
        {3500, 3500, 200, 60, 13800, 0, 500, 60},
        {2050, 2050, 135, 45, 30000, 3000, 1500, 25}
    }
};

track_dir_e track_last_dir = TRACK_DIR_CENTER;
uint8 track_finished = 0;
int16 track_error = 0;
int16 track_correction = 0;
uint32 track_elapsed_ms = 0;

static uint8 track_startup = 1;
static uint32 track_startup_ms = 0;
static uint32 track_run_start_ms = 0;
static uint32 track_run_elapsed_ms = 0;
static uint32 track_timer_start_ms = 0;
static uint8 track_timer_active = 0;
static uint8 track_timer_started = 0;
static uint8 track_timer_completed = 0;
static uint8 track_line_lost_active = 0;
static uint32 track_line_lost_ms = 0;
static int16 track_last_error = 0;
static uint8 track_finish_frames = 0;
static uint8 track_finish_marker_confirmed = 0;

// S1 至 S8 从左到右，权值放大 10 倍以保留多探头求平均的精度。
static const int8 track_sensor_weight[GRAY_CHANNEL_COUNT] = {
    70, 50, 30, 10, -10, -30, -50, -70
};

static const char *track_dir_str[] = {"CENTER", "LEFT  ", "RIGHT "};

static uint8 tracking_calculate_error(int16 *error);
static uint8 tracking_q2_marker_detected(void);
static int tracking_limit_correction(int correction, int base_l, int base_r);
static void tracking_begin_finish_stop(void);
static void tracking_complete_ball_stop(void);
static void tracking_follow_line(void);
static void tracking_set_motor(int left_pwm, int right_pwm);

void tracking_reset_runtime_state(void)
{
    track_last_dir = TRACK_DIR_CENTER;
    track_finished = 0;
    track_error = 0;
    track_correction = 0;
    track_elapsed_ms = 0;
    track_last_error = 0;
    track_startup = 1;
    track_startup_ms = system_getval_ms();
    track_run_start_ms = track_startup_ms;
    track_run_elapsed_ms = 0;
    track_timer_start_ms = track_startup_ms;
    track_timer_active = track_cfg.timer_enabled && track_cfg.tracking_enabled;
    track_timer_started = track_cfg.tracking_enabled;
    track_timer_completed = 0;
    track_line_lost_active = 0;
    track_line_lost_ms = 0;
    track_finish_frames = 0;
    track_finish_marker_confirmed = 0;
}

void tracking_init(void)
{
    tracking_reset_runtime_state();
}

void tracking_pause_runtime(void)
{
    if(track_timer_active)
    {
        track_elapsed_ms = system_getval_ms() - track_timer_start_ms;
        track_timer_active = 0;
    }

    // RUN 从 ON 切到 OFF 视为手动结束本轮，锁定当前用时。
    if(track_cfg.timer_enabled && track_timer_started)
    {
        track_timer_completed = 1;
    }
}

void tracking_timer_config_changed(void)
{
    uint32 now_ms = system_getval_ms();

    if(!track_cfg.timer_enabled)
    {
        if(track_timer_active)
        {
            track_elapsed_ms = now_ms - track_timer_start_ms;
        }
        track_timer_active = 0;
    }
    else if(track_cfg.tracking_enabled && !track_timer_completed)
    {
        // 中途重新打开后从保留值继续，不补算关闭期间的时间。
        track_timer_start_ms = now_ms - track_elapsed_ms;
        track_timer_active = 1;
        track_timer_started = 1;
    }
}

void tracking_update(void)
{
    uint32 now_ms;
    track_control_profile_t *profile;

    gray_read();

    if(!track_cfg.tracking_enabled)
    {
        Motor_Stop();
        return;
    }

    now_ms = system_getval_ms();
    track_run_elapsed_ms = now_ms - track_run_start_ms;
    profile = &track_cfg.control_profile[track_cfg.task_mode];

    if(track_finished)
    {
        Motor_Stop();
        return;
    }

    if(track_timer_active)
    {
        track_elapsed_ms = now_ms - track_timer_start_ms;
    }

    if(TRACK_TASK_BALL == track_cfg.task_mode
       && track_run_elapsed_ms >= profile->action_start_ms)
    {
        tracking_complete_ball_stop();
        return;
    }

    if(TRACK_TASK_Q2 == track_cfg.task_mode && !track_finish_marker_confirmed)
    {
        if(track_run_elapsed_ms >= profile->action_start_ms
           && tracking_q2_marker_detected())
        {
            if(track_finish_frames < TRACK_FINISH_CONFIRM_FRAMES)
            {
                track_finish_frames++;
            }
        }
        else
        {
            track_finish_frames = 0;
        }

        if(track_finish_frames >= TRACK_FINISH_CONFIRM_FRAMES)
        {
            track_finish_marker_confirmed = 1;
            tracking_begin_finish_stop();
            return;
        }
    }

    // 确认期间保持上一帧输出，避免把宽横杠误当作大偏差而突然转向。
    if(TRACK_TASK_Q2 == track_cfg.task_mode && track_finish_frames > 0)
    {
        return;
    }

    tracking_follow_line();
}

static uint8 tracking_calculate_error(int16 *error)
{
    int16 weight_sum = 0;
    uint8 active_count = 0;
    uint8 index;

    for(index = 0; index < GRAY_CHANNEL_COUNT; index++)
    {
        if(gray_result[index])
        {
            weight_sum += track_sensor_weight[index];
            active_count++;
        }
    }

    if(0 == active_count)
    {
        return 0;
    }

    *error = weight_sum / active_count;
    return 1;
}

static uint8 tracking_q2_marker_detected(void)
{
    uint8 adjacent_count = 0;
    uint8 index;

    // 终点只看中间六路 S2-S7，最外侧 S1/S8 只参与正常循迹。
    for(index = TRACK_FINISH_FIRST_SENSOR; index <= TRACK_FINISH_LAST_SENSOR; index++)
    {
        if(gray_result[index])
        {
            adjacent_count++;
            if(adjacent_count >= TRACK_FINISH_MIN_ADJACENT)
            {
                return 1;
            }
        }
        else
        {
            adjacent_count = 0;
        }
    }

    return 0;
}

static void tracking_begin_finish_stop(void)
{
    // 第 3 个确认帧即为比赛结束时刻。
    if(track_timer_active)
    {
        track_elapsed_ms = system_getval_ms() - track_timer_start_ms;
        track_timer_active = 0;
        track_timer_completed = 1;
    }
    Motor_Stop();
    track_finished = 1;
}

static void tracking_complete_ball_stop(void)
{
    // 主循环可能因屏幕刷新晚于阈值进入此处，自动停车计时锁定为设定时刻。
    if(track_timer_active)
    {
        track_elapsed_ms = track_cfg.control_profile[TRACK_TASK_BALL].action_start_ms;
        track_timer_active = 0;
        track_timer_completed = 1;
    }
    Motor_Stop();
    track_finished = 1;
}

static int tracking_limit_correction(int correction, int base_l, int base_r)
{
    int positive_limit = base_l;
    int negative_limit = base_r;

    if((PWM_MAX - base_r) < positive_limit)
    {
        positive_limit = PWM_MAX - base_r;
    }
    if((PWM_MAX - base_l) < negative_limit)
    {
        negative_limit = PWM_MAX - base_l;
    }

    if(correction > positive_limit)
    {
        correction = positive_limit;
    }
    else if(correction < -negative_limit)
    {
        correction = -negative_limit;
    }

    return correction;
}

static void tracking_follow_line(void)
{
    track_control_profile_t *profile = &track_cfg.control_profile[track_cfg.task_mode];
    int base_l = profile->base_pwm_l;
    int base_r = profile->base_pwm_r;
    int16 current_error;
    int correction;

    // 第二问在接近终点时降速；钢珠任务始终使用正常基础速度。
    if(TRACK_TASK_Q2 == track_cfg.task_mode
       && track_run_elapsed_ms >= profile->action_start_ms)
    {
        base_l = base_l * track_cfg.finish_find_speed_pct / 100;
        base_r = base_r * track_cfg.finish_find_speed_pct / 100;
    }

    // 仅在启动阶段平滑加速，进入正常运行后基础速度保持不变。
    if(track_startup)
    {
        uint32 elapsed = system_getval_ms() - track_startup_ms;
        if(elapsed < profile->startup_ms && profile->startup_ms > 0)
        {
            uint32 ratio = profile->startup_min_pct
                + (100U - profile->startup_min_pct) * elapsed / profile->startup_ms;
            base_l = base_l * (int)ratio / 100;
            base_r = base_r * (int)ratio / 100;
        }
        else
        {
            track_startup = 0;
        }
    }

    if(tracking_calculate_error(&current_error))
    {
        track_line_lost_active = 0;
        track_error = current_error;

        if(track_error > 0)
        {
            track_last_dir = TRACK_DIR_LEFT;
        }
        else if(track_error < 0)
        {
            track_last_dir = TRACK_DIR_RIGHT;
        }
        else
        {
            track_last_dir = TRACK_DIR_CENTER;
        }
    }
    else
    {
        if(!track_line_lost_active)
        {
            track_line_lost_active = 1;
            track_line_lost_ms = system_getval_ms();
        }

        if((system_getval_ms() - track_line_lost_ms) >= track_cfg.line_lost_stop_ms)
        {
            Motor_Stop();
            track_finished = 1;
            return;
        }

        // 丢线时向最后发现黑线的一侧继续搜索。
        if(TRACK_DIR_LEFT == track_last_dir)
        {
            track_error = TRACK_MAX_ERROR;
        }
        else if(TRACK_DIR_RIGHT == track_last_dir)
        {
            track_error = -TRACK_MAX_ERROR;
        }
        else
        {
            track_error = 0;
        }
    }

    correction = ((int)profile->kp * track_error
                + (int)profile->kd * (track_error - track_last_error))
                / TRACK_ERROR_SCALE;
    correction = tracking_limit_correction(correction, base_l, base_r);
    track_correction = (int16)correction;
    track_last_error = track_error;

    if(TRACK_TASK_BALL == track_cfg.task_mode && profile->slow_stop_ms > 0)
    {
        uint32 slow_start_ms = profile->action_start_ms - profile->slow_stop_ms;

        if(track_run_elapsed_ms >= slow_start_ms)
        {
            uint32 remaining_ms = profile->action_start_ms - track_run_elapsed_ms;
            int left_pwm = base_l - correction;
            int right_pwm = base_r + correction;

            left_pwm = left_pwm * (int)remaining_ms / profile->slow_stop_ms;
            right_pwm = right_pwm * (int)remaining_ms / profile->slow_stop_ms;
            tracking_set_motor(left_pwm, right_pwm);
            return;
        }
    }

    tracking_set_motor(base_l - correction, base_r + correction);
}

static void tracking_set_motor(int left_pwm, int right_pwm)
{
    Motor_Setpwm_L(left_pwm);
    Motor_Setpwm_R(right_pwm);
}

void tracking_display(uint16 x, uint16 y)
{
    char buffer[32];
    char bar_status[12];
    int error_abs = (track_error < 0) ? -track_error : track_error;
    const char *run_state;
    track_control_profile_t *profile = &track_cfg.control_profile[track_cfg.task_mode];

    if(track_finished)
    {
        run_state = "STOP";
    }
    else if(track_cfg.tracking_enabled)
    {
        if(TRACK_TASK_Q2 == track_cfg.task_mode
           && track_run_elapsed_ms >= profile->action_start_ms)
        {
            run_state = "FIND";
        }
        else if(TRACK_TASK_BALL == track_cfg.task_mode
                && profile->slow_stop_ms > 0
                && track_run_elapsed_ms >= profile->action_start_ms - profile->slow_stop_ms)
        {
            run_state = "SLOW";
        }
        else
        {
            run_state = "RUN ";
        }
    }
    else
    {
        run_state = "IDLE";
    }

    sprintf(buffer, "ERR:%c%d.%d OUT:%+d   ",
            (track_error < 0) ? '-' : '+',
            error_abs / TRACK_ERROR_SCALE,
            error_abs % TRACK_ERROR_SCALE,
            track_correction);
    ips200_show_string(x, y, buffer);

    sprintf(buffer, "DIR:%s %s       ",
            track_dir_str[track_last_dir], run_state);
    ips200_show_string(x, y + 10, buffer);

    sprintf(buffer, "MODE:%s KP:%d KD:%d   ",
            (TRACK_TASK_Q2 == track_cfg.task_mode) ? "Q2  " : "BALL",
            track_cfg.control_profile[track_cfg.task_mode].kp,
            track_cfg.control_profile[track_cfg.task_mode].kd);
    ips200_show_string(x, y + 20, buffer);

    if(TRACK_TASK_Q2 == track_cfg.task_mode)
    {
        sprintf(bar_status, "BAR:%u/%u", track_finish_frames, TRACK_FINISH_CONFIRM_FRAMES);
    }
    else
    {
        sprintf(bar_status, "AUTO:%lus", (unsigned long)(profile->action_start_ms / 1000U));
    }

    if(!track_cfg.timer_enabled)
    {
        sprintf(buffer, "TIME:OFF %s       ", bar_status);
    }
    else if(track_timer_active || track_timer_completed || track_timer_started)
    {
        sprintf(buffer, "TIME:%lu.%03lus %s ",
                (unsigned long)(track_elapsed_ms / 1000U),
                (unsigned long)(track_elapsed_ms % 1000U),
                bar_status);
    }
    else
    {
        sprintf(buffer, "TIME:WAIT %s      ", bar_status);
    }
    ips200_show_string(x, y + 30, buffer);
}
