/*
 * tuning_ui.c
 * 板载按键 UI 调参模块
 * B1: 增加参数
 * B0: 减少参数
 * A31: 下一参数
 * A30: 上一参数
 */

#include "tuning_ui.h"
#include "tracking.h"
#include "zf_device_key.h"

#define TUNING_UI_SCAN_PERIOD_MS      10

#define TUNE_BASE_PWM_STEP            50
#define TUNE_PD_STEP                  5
#define TUNE_TIME_STEP_MS             20
#define TUNE_FIND_TIME_STEP_MS        100
#define TUNE_PERCENT_STEP             5
#define TUNING_UI_VISIBLE_ROWS        8

typedef enum
{
    TUNE_PARAM_TASK_MODE = 0,
    TUNE_PARAM_TRACK_ENABLE,
    TUNE_PARAM_TIMER_ENABLE,
    TUNE_PARAM_ACTION_START_MS,
    TUNE_PARAM_SLOW_STOP_MS,
    TUNE_PARAM_FINISH_FIND_SPEED_PCT,
    TUNE_PARAM_BASE_PWM_L,
    TUNE_PARAM_BASE_PWM_R,
    TUNE_PARAM_KP,
    TUNE_PARAM_KD,
    TUNE_PARAM_STARTUP_MS,
    TUNE_PARAM_STARTUP_MIN,
    TUNE_PARAM_LINE_LOST_STOP_MS,
    TUNE_PARAM_COUNT,
} tuning_param_e;

static tuning_param_e tuning_param_index = TUNE_PARAM_TASK_MODE;
static uint8 tuning_ui_dirty = 1;

static const tuning_param_e tuning_q2_params[] = {
    TUNE_PARAM_TASK_MODE,
    TUNE_PARAM_TRACK_ENABLE,
    TUNE_PARAM_TIMER_ENABLE,
    TUNE_PARAM_ACTION_START_MS,
    TUNE_PARAM_FINISH_FIND_SPEED_PCT,
    TUNE_PARAM_BASE_PWM_L,
    TUNE_PARAM_BASE_PWM_R,
    TUNE_PARAM_KP,
    TUNE_PARAM_KD,
    TUNE_PARAM_STARTUP_MS,
    TUNE_PARAM_STARTUP_MIN,
    TUNE_PARAM_LINE_LOST_STOP_MS,
};

static const tuning_param_e tuning_ball_params[] = {
    TUNE_PARAM_TASK_MODE,
    TUNE_PARAM_TRACK_ENABLE,
    TUNE_PARAM_TIMER_ENABLE,
    TUNE_PARAM_ACTION_START_MS,
    TUNE_PARAM_SLOW_STOP_MS,
    TUNE_PARAM_BASE_PWM_L,
    TUNE_PARAM_BASE_PWM_R,
    TUNE_PARAM_KP,
    TUNE_PARAM_KD,
    TUNE_PARAM_STARTUP_MS,
    TUNE_PARAM_STARTUP_MIN,
    TUNE_PARAM_LINE_LOST_STOP_MS,
};

static const tuning_param_e *tuning_visible_params(uint8 *count)
{
    if(TRACK_TASK_Q2 == track_cfg.task_mode)
    {
        *count = (uint8)(sizeof(tuning_q2_params) / sizeof(tuning_q2_params[0]));
        return tuning_q2_params;
    }

    *count = (uint8)(sizeof(tuning_ball_params) / sizeof(tuning_ball_params[0]));
    return tuning_ball_params;
}

static uint8 tuning_visible_index(void)
{
    uint8 count;
    uint8 index;
    const tuning_param_e *params = tuning_visible_params(&count);

    for(index = 0; index < count; index++)
    {
        if(params[index] == tuning_param_index)
        {
            return index;
        }
    }

    tuning_param_index = params[0];
    return 0;
}

static int32 tuning_clamp_i32(int32 value, int32 min_value, int32 max_value)
{
    if(value < min_value)
    {
        return min_value;
    }
    if(value > max_value)
    {
        return max_value;
    }
    return value;
}

static uint16 tuning_adjust_u16(uint16 value, int32 delta, uint16 min_value, uint16 max_value)
{
    return (uint16)tuning_clamp_i32((int32)value + delta, min_value, max_value);
}

static uint8 tuning_adjust_u8(uint8 value, int32 delta, uint8 min_value, uint8 max_value)
{
    return (uint8)tuning_clamp_i32((int32)value + delta, min_value, max_value);
}

static track_control_profile_t *tuning_active_profile(void)
{
    return &track_cfg.control_profile[track_cfg.task_mode];
}

static void tuning_normalize_control_profile(track_control_profile_t *profile)
{
    profile->base_pwm_l = (int16)tuning_clamp_i32(profile->base_pwm_l, 0, PWM_MAX);
    profile->base_pwm_r = (int16)tuning_clamp_i32(profile->base_pwm_r, 0, PWM_MAX);
    profile->kp = (int16)tuning_clamp_i32(profile->kp, 0, 1000);
    profile->kd = (int16)tuning_clamp_i32(profile->kd, 0, 1000);
    profile->action_start_ms = (uint16)tuning_clamp_i32(profile->action_start_ms, 0, 60000);
    profile->slow_stop_ms = (uint16)tuning_clamp_i32(profile->slow_stop_ms, 0, profile->action_start_ms);
    profile->startup_ms = (uint16)tuning_clamp_i32(profile->startup_ms, 0, 3000);
    profile->startup_min_pct = tuning_adjust_u8(profile->startup_min_pct, 0, 0, 100);

}

static void tuning_normalize_config(void)
{
    uint8 index;

    track_cfg.task_mode = (track_task_mode_e)tuning_clamp_i32(track_cfg.task_mode,
                                                              TRACK_TASK_Q2,
                                                              TRACK_TASK_BALL);
    track_cfg.timer_enabled = tuning_adjust_u8(track_cfg.timer_enabled, 0, 0, 1);
    track_cfg.finish_find_speed_pct = tuning_adjust_u8(track_cfg.finish_find_speed_pct, 0, 10, 100);
    track_cfg.line_lost_stop_ms = (uint16)tuning_clamp_i32(track_cfg.line_lost_stop_ms, 0, 5000);

    for(index = 0; index < TRACK_TASK_COUNT; index++)
    {
        tuning_normalize_control_profile(&track_cfg.control_profile[index]);
    }
}

static void tuning_select_next(void)
{
    uint8 count;
    uint8 index = tuning_visible_index();
    const tuning_param_e *params = tuning_visible_params(&count);

    tuning_param_index = params[(index + 1) % count];
}

static void tuning_select_prev(void)
{
    uint8 count;
    uint8 index = tuning_visible_index();
    const tuning_param_e *params = tuning_visible_params(&count);

    if(0 == index)
    {
        tuning_param_index = params[count - 1];
    }
    else
    {
        tuning_param_index = params[index - 1];
    }
}

static void tuning_adjust_selected(int8 direction)
{
    track_control_profile_t *profile = tuning_active_profile();

    switch(tuning_param_index)
    {
        case TUNE_PARAM_TASK_MODE:
            if(!track_cfg.tracking_enabled)
            {
                track_task_mode_e task_mode = (track_task_mode_e)tuning_clamp_i32(track_cfg.task_mode + direction,
                                                                                  TRACK_TASK_Q2,
                                                                                  TRACK_TASK_BALL);
                if(task_mode != track_cfg.task_mode)
                {
                    track_cfg.task_mode = task_mode;
                    tuning_visible_index();
                }
            }
            break;

        case TUNE_PARAM_BASE_PWM_L:
            profile->base_pwm_l = (int16)(profile->base_pwm_l + direction * TUNE_BASE_PWM_STEP);
            break;

        case TUNE_PARAM_BASE_PWM_R:
            profile->base_pwm_r = (int16)(profile->base_pwm_r + direction * TUNE_BASE_PWM_STEP);
            break;

        case TUNE_PARAM_KP:
            profile->kp = (int16)(profile->kp + direction * TUNE_PD_STEP);
            break;

        case TUNE_PARAM_KD:
            profile->kd = (int16)(profile->kd + direction * TUNE_PD_STEP);
            break;

        case TUNE_PARAM_STARTUP_MS:
            profile->startup_ms = tuning_adjust_u16(profile->startup_ms,
                                                    direction * TUNE_TIME_STEP_MS,
                                                    0,
                                                    3000);
            break;

        case TUNE_PARAM_STARTUP_MIN:
            profile->startup_min_pct = tuning_adjust_u8(profile->startup_min_pct,
                                                        direction * TUNE_PERCENT_STEP,
                                                        0,
                                                        100);
            break;

        case TUNE_PARAM_LINE_LOST_STOP_MS:
            track_cfg.line_lost_stop_ms = tuning_adjust_u16(track_cfg.line_lost_stop_ms,
                                                            direction * TUNE_TIME_STEP_MS,
                                                            0,
                                                            5000);
            break;

        case TUNE_PARAM_TRACK_ENABLE:
        {
            uint8 enabled = tuning_adjust_u8(track_cfg.tracking_enabled, direction, 0, 1);
            if(enabled != track_cfg.tracking_enabled)
            {
                track_cfg.tracking_enabled = enabled;
                Motor_Stop();
                if(enabled)
                {
                    // 只有 OFF -> ON 才开始新一轮并清零计时。
                    tracking_reset_runtime_state();
                }
                else
                {
                    tracking_pause_runtime();
                }
            }
            break;
        }

        case TUNE_PARAM_TIMER_ENABLE:
        {
            uint8 timer_enabled = tuning_adjust_u8(track_cfg.timer_enabled, direction, 0, 1);
            if(timer_enabled != track_cfg.timer_enabled)
            {
                track_cfg.timer_enabled = timer_enabled;
                tracking_timer_config_changed();
            }
            break;
        }

        case TUNE_PARAM_ACTION_START_MS:
            profile->action_start_ms = tuning_adjust_u16(profile->action_start_ms,
                                                         direction * TUNE_FIND_TIME_STEP_MS,
                                                         0,
                                                         60000);
            break;

        case TUNE_PARAM_SLOW_STOP_MS:
            profile->slow_stop_ms = tuning_adjust_u16(profile->slow_stop_ms,
                                                      direction * TUNE_FIND_TIME_STEP_MS,
                                                      0,
                                                      profile->action_start_ms);
            break;

        case TUNE_PARAM_FINISH_FIND_SPEED_PCT:
            track_cfg.finish_find_speed_pct = tuning_adjust_u8(track_cfg.finish_find_speed_pct,
                                                               direction * TUNE_PERCENT_STEP,
                                                               10,
                                                               100);
            break;

        default:
            break;
    }

    tuning_normalize_config();
    tuning_ui_dirty = 1;
}

static const char *tuning_param_name(tuning_param_e param)
{
    switch(param)
    {
        case TUNE_PARAM_TASK_MODE:
            return "MODE";

        case TUNE_PARAM_TRACK_ENABLE:
            return "RUN";

        case TUNE_PARAM_TIMER_ENABLE:
            return "TIMER";

        case TUNE_PARAM_ACTION_START_MS:
            return (TRACK_TASK_Q2 == track_cfg.task_mode) ? "FNDMS" : "STOPMS";

        case TUNE_PARAM_SLOW_STOP_MS:
            return "SSTOP";

        case TUNE_PARAM_FINISH_FIND_SPEED_PCT:
            return "FNDP";

        case TUNE_PARAM_BASE_PWM_L:
            return "BASEL";

        case TUNE_PARAM_BASE_PWM_R:
            return "BASER";

        case TUNE_PARAM_KP:
            return "KP";

        case TUNE_PARAM_KD:
            return "KD";

        case TUNE_PARAM_STARTUP_MS:
            return "SUPMS";

        case TUNE_PARAM_STARTUP_MIN:
            return "SUPMN";

        case TUNE_PARAM_LINE_LOST_STOP_MS:
            return "LSTOP";

        default:
            return "UNK";
    }
}

static void tuning_format_param_value(tuning_param_e param, char *buffer)
{
    track_control_profile_t *profile = tuning_active_profile();

    switch(param)
    {
        case TUNE_PARAM_TASK_MODE:
            sprintf(buffer, "%s", (TRACK_TASK_Q2 == track_cfg.task_mode) ? "Q2" : "BALL");
            break;

        case TUNE_PARAM_TRACK_ENABLE:
            sprintf(buffer, "%s", track_cfg.tracking_enabled ? "ON" : "OFF");
            break;

        case TUNE_PARAM_TIMER_ENABLE:
            sprintf(buffer, "%s", track_cfg.timer_enabled ? "ON" : "OFF");
            break;

        case TUNE_PARAM_ACTION_START_MS:
            sprintf(buffer, "%u", profile->action_start_ms);
            break;

        case TUNE_PARAM_SLOW_STOP_MS:
            sprintf(buffer, "%u", profile->slow_stop_ms);
            break;

        case TUNE_PARAM_FINISH_FIND_SPEED_PCT:
            sprintf(buffer, "%u%%", track_cfg.finish_find_speed_pct);
            break;

        case TUNE_PARAM_BASE_PWM_L:
            sprintf(buffer, "%d", profile->base_pwm_l);
            break;

        case TUNE_PARAM_BASE_PWM_R:
            sprintf(buffer, "%d", profile->base_pwm_r);
            break;

        case TUNE_PARAM_KP:
            sprintf(buffer, "%d", profile->kp);
            break;

        case TUNE_PARAM_KD:
            sprintf(buffer, "%d", profile->kd);
            break;

        case TUNE_PARAM_STARTUP_MS:
            sprintf(buffer, "%u", profile->startup_ms);
            break;

        case TUNE_PARAM_STARTUP_MIN:
            sprintf(buffer, "%u%%", profile->startup_min_pct);
            break;

        case TUNE_PARAM_LINE_LOST_STOP_MS:
            sprintf(buffer, "%u", track_cfg.line_lost_stop_ms);
            break;

        default:
            sprintf(buffer, "-");
            break;
    }
}

void tuning_ui_init(void)
{
    key_init(TUNING_UI_SCAN_PERIOD_MS);
    tuning_normalize_config();
    tuning_ui_dirty = 1;
}

void tuning_ui_update(void)
{
    key_scanner();

    if(KEY_SHORT_PRESS == key_get_state(KEY_4))
    {
        tuning_adjust_selected(1);
        key_clear_all_state();
    }
    else if(KEY_SHORT_PRESS == key_get_state(KEY_3))
    {
        tuning_adjust_selected(-1);
        key_clear_all_state();
    }
    else if(KEY_SHORT_PRESS == key_get_state(KEY_2))
    {
        tuning_select_next();
        tuning_ui_dirty = 1;
        key_clear_all_state();
    }
    else if(KEY_SHORT_PRESS == key_get_state(KEY_1))
    {
        tuning_select_prev();
        tuning_ui_dirty = 1;
        key_clear_all_state();
    }
}

void tuning_ui_display(uint16 x, uint16 y)
{
    char line_buffer[32];
    char value_buffer[16];
    uint8 row;
    uint8 selected_index;
    uint8 param_count;
    uint8 start_index = 0;
    uint8 visible_rows = TUNING_UI_VISIBLE_ROWS;
    const tuning_param_e *params = tuning_visible_params(&param_count);

    if(!tuning_ui_dirty)
    {
        return;
    }

    selected_index = tuning_visible_index();

    if(param_count <= TUNING_UI_VISIBLE_ROWS)
    {
        start_index = 0;
        visible_rows = param_count;
    }
    else if(selected_index >= (TUNING_UI_VISIBLE_ROWS / 2))
    {
        start_index = (uint8)(selected_index - (TUNING_UI_VISIBLE_ROWS / 2));
        if((start_index + TUNING_UI_VISIBLE_ROWS) > param_count)
        {
            start_index = (uint8)(param_count - TUNING_UI_VISIBLE_ROWS);
        }
    }

    sprintf(line_buffer, "CFG:%s %02u/%02u      ",
            (TRACK_TASK_Q2 == track_cfg.task_mode) ? "Q2  " : "BALL",
            selected_index + 1,
            param_count);
    ips200_show_string(x, y, line_buffer);

    for(row = 0; row < visible_rows; row++)
    {
        tuning_param_e param = params[start_index + row];
        tuning_format_param_value(param, value_buffer);
        sprintf(line_buffer, "%c%-5s %-8s      ",
                (param == tuning_param_index) ? '>' : ' ',
                tuning_param_name(param),
                value_buffer);
        ips200_show_string(x, y + 10 + row * 10, line_buffer);
    }

    for(; row < TUNING_UI_VISIBLE_ROWS; row++)
    {
        ips200_show_string(x, y + 10 + row * 10, "                        ");
    }

    ips200_show_string(x, y + 10 + visible_rows * 10, "+B1 -B0 A31> A30<   ");
    tuning_ui_dirty = 0;
}
