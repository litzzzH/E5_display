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
#define TUNE_DIFF_STEP                20
#define TUNE_TIME_STEP_MS             20
#define TUNE_PERCENT_STEP             5
#define TUNING_UI_VISIBLE_ROWS        8

typedef enum
{
    TUNE_PARAM_SPEED_MODE = 0,
    TUNE_PARAM_TRACK_ENABLE,
    TUNE_PARAM_BASE_PWM_L,
    TUNE_PARAM_BASE_PWM_R,
    TUNE_PARAM_DIFF_SMALL,
    TUNE_PARAM_DIFF_MED,
    TUNE_PARAM_DIFF_LARGE,
    TUNE_PARAM_STARTUP_MS,
    TUNE_PARAM_STARTUP_MIN,
    TUNE_PARAM_LINE_LOST_STOP_MS,
    TUNE_PARAM_COUNT,
} tuning_param_e;

static tuning_param_e tuning_param_index = TUNE_PARAM_SPEED_MODE;
static uint8 tuning_ui_dirty = 1;

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

static track_speed_profile_t *tuning_active_profile(void)
{
    return &track_cfg.speed_profile[track_cfg.speed_mode];
}

static void tuning_normalize_speed_profile(track_speed_profile_t *profile)
{
    profile->base_pwm_l = (int16)tuning_clamp_i32(profile->base_pwm_l, 0, PWM_MAX);
    profile->base_pwm_r = (int16)tuning_clamp_i32(profile->base_pwm_r, 0, PWM_MAX);
    profile->diff_small = (int16)tuning_clamp_i32(profile->diff_small, 0, PWM_MAX);
    profile->diff_med   = (int16)tuning_clamp_i32(profile->diff_med, 0, PWM_MAX);
    profile->diff_large = (int16)tuning_clamp_i32(profile->diff_large, 0, PWM_MAX);

    if(profile->diff_med < profile->diff_small)
    {
        profile->diff_med = profile->diff_small;
    }
    if(profile->diff_large < profile->diff_med)
    {
        profile->diff_large = profile->diff_med;
    }

    profile->startup_ms      = (uint16)tuning_clamp_i32(profile->startup_ms, 0, 3000);
    profile->startup_min_pct = (uint8)tuning_clamp_i32(profile->startup_min_pct, 0, 100);
}

static void tuning_normalize_config(void)
{
    uint8 index;

    track_cfg.speed_mode = (track_speed_mode_e)tuning_clamp_i32(track_cfg.speed_mode,
                                                                TRACK_SPEED_MODE_LOW,
                                                                TRACK_SPEED_MODE_HIGH);
    track_cfg.line_lost_stop_ms = (uint16)tuning_clamp_i32(track_cfg.line_lost_stop_ms, 0, 5000);

    for(index = 0; index < TRACK_SPEED_MODE_COUNT; index++)
    {
        tuning_normalize_speed_profile(&track_cfg.speed_profile[index]);
    }
}

static void tuning_select_next(void)
{
    tuning_param_index = (tuning_param_e)((tuning_param_index + 1) % TUNE_PARAM_COUNT);
}

static void tuning_select_prev(void)
{
    if(TUNE_PARAM_SPEED_MODE == tuning_param_index)
    {
        tuning_param_index = (tuning_param_e)(TUNE_PARAM_COUNT - 1);
    }
    else
    {
        tuning_param_index = (tuning_param_e)(tuning_param_index - 1);
    }
}

static void tuning_adjust_selected(int8 direction)
{
    track_speed_profile_t *profile = tuning_active_profile();

    switch(tuning_param_index)
    {
        case TUNE_PARAM_SPEED_MODE:
            track_cfg.speed_mode = (track_speed_mode_e)tuning_clamp_i32(track_cfg.speed_mode + direction,
                                                                        TRACK_SPEED_MODE_LOW,
                                                                        TRACK_SPEED_MODE_HIGH);
            break;

        case TUNE_PARAM_BASE_PWM_L:
            profile->base_pwm_l = (int16)(profile->base_pwm_l + direction * TUNE_BASE_PWM_STEP);
            break;

        case TUNE_PARAM_BASE_PWM_R:
            profile->base_pwm_r = (int16)(profile->base_pwm_r + direction * TUNE_BASE_PWM_STEP);
            break;

        case TUNE_PARAM_DIFF_SMALL:
            profile->diff_small = (int16)(profile->diff_small + direction * TUNE_DIFF_STEP);
            break;

        case TUNE_PARAM_DIFF_MED:
            profile->diff_med = (int16)(profile->diff_med + direction * TUNE_DIFF_STEP);
            break;

        case TUNE_PARAM_DIFF_LARGE:
            profile->diff_large = (int16)(profile->diff_large + direction * TUNE_DIFF_STEP);
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
            track_cfg.tracking_enabled = tuning_adjust_u8(track_cfg.tracking_enabled, direction, 0, 1);
            Motor_Stop();
            tracking_reset_runtime_state();
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
        case TUNE_PARAM_SPEED_MODE:
            return "MODE";

        case TUNE_PARAM_TRACK_ENABLE:
            return "RUN";

        case TUNE_PARAM_BASE_PWM_L:
            return "BASEL";

        case TUNE_PARAM_BASE_PWM_R:
            return "BASER";

        case TUNE_PARAM_DIFF_SMALL:
            return "DIFFS";

        case TUNE_PARAM_DIFF_MED:
            return "DIFFM";

        case TUNE_PARAM_DIFF_LARGE:
            return "DIFFL";

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
    track_speed_profile_t *profile = tuning_active_profile();

    switch(param)
    {
        case TUNE_PARAM_SPEED_MODE:
            sprintf(buffer, "%s", (TRACK_SPEED_MODE_LOW == track_cfg.speed_mode) ? "LOW" : "HIGH");
            break;

        case TUNE_PARAM_TRACK_ENABLE:
            sprintf(buffer, "%s", track_cfg.tracking_enabled ? "ON" : "OFF");
            break;

        case TUNE_PARAM_BASE_PWM_L:
            sprintf(buffer, "%d", profile->base_pwm_l);
            break;

        case TUNE_PARAM_BASE_PWM_R:
            sprintf(buffer, "%d", profile->base_pwm_r);
            break;

        case TUNE_PARAM_DIFF_SMALL:
            sprintf(buffer, "%d", profile->diff_small);
            break;

        case TUNE_PARAM_DIFF_MED:
            sprintf(buffer, "%d", profile->diff_med);
            break;

        case TUNE_PARAM_DIFF_LARGE:
            sprintf(buffer, "%d", profile->diff_large);
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
    uint8 start_index = 0;
    uint8 visible_rows = TUNING_UI_VISIBLE_ROWS;

    if(!tuning_ui_dirty)
    {
        return;
    }

    if(TUNE_PARAM_COUNT <= TUNING_UI_VISIBLE_ROWS)
    {
        start_index = 0;
        visible_rows = (uint8)TUNE_PARAM_COUNT;
    }
    else if(tuning_param_index >= (TUNING_UI_VISIBLE_ROWS / 2))
    {
        start_index = (uint8)(tuning_param_index - (TUNING_UI_VISIBLE_ROWS / 2));
        if((start_index + TUNING_UI_VISIBLE_ROWS) > TUNE_PARAM_COUNT)
        {
            start_index = (uint8)(TUNE_PARAM_COUNT - TUNING_UI_VISIBLE_ROWS);
        }
    }

    sprintf(line_buffer, "CFG:%s %02u/%02u      ",
            (TRACK_SPEED_MODE_LOW == track_cfg.speed_mode) ? "LOW " : "HIGH",
            (uint8)tuning_param_index + 1,
            (uint8)TUNE_PARAM_COUNT);
    ips200_show_string(x, y, line_buffer);

    for(row = 0; row < visible_rows; row++)
    {
        tuning_param_e param = (tuning_param_e)(start_index + row);
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
