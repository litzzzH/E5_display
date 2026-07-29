/*
 * tracking.c
 * 循迹模块 - MSPM0G3507 移植版
 *
 * 线偏左 → 车要左转 → 右轮快左轮慢
 * 线偏右 → 车要右转 → 左轮快右轮慢
 */

#include "tracking.h"

track_runtime_config_t track_cfg = {
    0,
    TRACK_SPEED_MODE_LOW,
    2000,
    {
        {3000, 3000, 200, 400, 700, 500, 30},
        {4000, 4000, 200, 400, 400, 500, 30}
    }
};

track_dir_e   track_last_dir = TRACK_DIR_CENTER;
uint8         track_finished   = 0;     // 停车标志

static uint8  track_startup           = 1;     // 启动加速标志（1=首次启动）
static uint32 track_startup_ms        = 0;     // 启动时刻(ms)
static uint8  track_line_lost_active  = 0;     // 连续全白标志
static uint32 track_line_lost_ms      = 0;     // 连续全白起始时刻

static const char* dir_str[]   = {"CENTER", "LEFT  ", "RIGHT "};

#define L2  (gray_result[IDX_L2])
#define L1  (gray_result[IDX_L1])
#define M   (gray_result[IDX_M])
#define R1  (gray_result[IDX_R1])
#define R2  (gray_result[IDX_R2])

static void tracking_follow_line(void);
static void tracking_set_motor(int left_pwm, int right_pwm);

void tracking_reset_runtime_state(void)
{
    track_last_dir = TRACK_DIR_CENTER;
    track_finished   = 0;
    track_startup  = 1;
    track_startup_ms = system_getval_ms();
    track_line_lost_active = 0;
    track_line_lost_ms = 0;
}

void tracking_init(void)
{
    tracking_reset_runtime_state();
}

void tracking_update(void)
{
    if(!track_cfg.tracking_enabled)
    {
        Motor_Stop();
        return;
    }

    // 已停车，不再执行
    if(track_finished)
    {
        Motor_Stop();
        return;
    }

    gray_read();
    tracking_follow_line();
}

// ---------- 普通循迹 ----------
// 核心原则：线偏左 → 车要左转 → 右轮快左轮慢
//           线偏右 → 车要右转 → 左轮快右轮慢
static void tracking_follow_line(void)
{
    int base_l = TRACK_BASE_PWM_L;
    int base_r = TRACK_BASE_PWM_R;

    // 启动线性加速
    if(track_startup)
    {
        uint32 elapsed = system_getval_ms() - track_startup_ms;
        if(elapsed < TRACK_STARTUP_MS)
        {
            float ratio = TRACK_STARTUP_MIN
                + (1.0f - TRACK_STARTUP_MIN) * (float)elapsed / (float)TRACK_STARTUP_MS;
            base_l = (int)(base_l * ratio);
            base_r = (int)(base_r * ratio);
        }
        else
        {
            track_startup = 0;
        }
    }

    // 仅中间检测到 → 直行
    if(!L2 && !L1 && M && !R1 && !R2)
    {
        track_line_lost_active = 0;
        track_last_dir = TRACK_DIR_CENTER;
        tracking_set_motor(base_l, base_r);
        return;
    }

    // L2+L1同时检测到 → 向左大幅修正
    if(L2 && L1 && !R2)
    {
        track_line_lost_active = 0;
        track_last_dir = TRACK_DIR_LEFT;
        tracking_set_motor(base_l - TRACK_DIFF_LARGE, base_r + TRACK_DIFF_LARGE);
        return;
    }
    // R1+R2同时检测到 → 大幅修正兜底
    if(!L2 && R1 && R2)
    {
        track_line_lost_active = 0;
        track_last_dir = TRACK_DIR_RIGHT;
        tracking_set_motor(base_l + TRACK_DIFF_LARGE, base_r - TRACK_DIFF_LARGE);
        return;
    }

    if(!L2 && !R2)
    {
        // L1+M 检测到，线偏左 → 左转修正
        if(L1 && M && !R1)
        {
            track_line_lost_active = 0;
            track_last_dir = TRACK_DIR_LEFT;
            tracking_set_motor(base_l - TRACK_DIFF_SMALL, base_r + TRACK_DIFF_SMALL);
            return;
        }
        // M+R1 检测到，线偏右 → 右转修正
        if(!L1 && M && R1)
        {
            track_line_lost_active = 0;
            track_last_dir = TRACK_DIR_RIGHT;
            tracking_set_motor(base_l + TRACK_DIFF_SMALL, base_r - TRACK_DIFF_SMALL);
            return;
        }
        // 仅L1 检测到，线偏左更多
        if(L1 && !M && !R1)
        {
            track_line_lost_active = 0;
            track_last_dir = TRACK_DIR_LEFT;
            tracking_set_motor(base_l - TRACK_DIFF_MED, base_r + TRACK_DIFF_MED);
            return;
        }
        // 仅R1 检测到，线偏右更多
        if(!L1 && !M && R1)
        {
            track_line_lost_active = 0;
            track_last_dir = TRACK_DIR_RIGHT;
            tracking_set_motor(base_l + TRACK_DIFF_MED, base_r - TRACK_DIFF_MED);
            return;
        }
    }

    // L2 检测到，线严重偏左
    if(L2 && !R2)
    {
        track_line_lost_active = 0;
        track_last_dir = TRACK_DIR_LEFT;
        tracking_set_motor(base_l - TRACK_DIFF_LARGE, base_r + TRACK_DIFF_LARGE);
        return;
    }
    // R2 检测到，线严重偏右
    if(!L2 && R2)
    {
        track_line_lost_active = 0;
        track_last_dir = TRACK_DIR_RIGHT;
        tracking_set_motor(base_l + TRACK_DIFF_LARGE, base_r - TRACK_DIFF_LARGE);
        return;
    }

    // 全部丢线(全白) → 先按上次方向找线，连续超时后再停车
    if(!L2 && !L1 && !M && !R1 && !R2)
    {
        if(!track_line_lost_active)
        {
            track_line_lost_active = 1;
            track_line_lost_ms = system_getval_ms();
        }

        if((system_getval_ms() - track_line_lost_ms) >= TRACK_LINE_LOST_STOP_MS)
        {
            Motor_Stop();
            track_finished = 1;
            return;
        }

        switch(track_last_dir)
        {
            case TRACK_DIR_LEFT:
                tracking_set_motor(base_l - TRACK_DIFF_LARGE, base_r + TRACK_DIFF_LARGE);
                break;
            case TRACK_DIR_RIGHT:
                tracking_set_motor(base_l + TRACK_DIFF_LARGE, base_r - TRACK_DIFF_LARGE);
                break;
            default:
                tracking_set_motor(base_l, base_r);
                break;
        }
        return;
    }

    track_line_lost_active = 0;
    tracking_set_motor(base_l, base_r);
}

// ---------- 电机封装 ----------
static void tracking_set_motor(int left_pwm, int right_pwm)
{
    Motor_Setpwm_L(left_pwm);
    Motor_Setpwm_R(right_pwm);
}

// ---------- 循迹信息显示 ----------
void tracking_display(uint16 x, uint16 y)
{
    char buf[32];

    sprintf(buf, "L2:%u L1:%u M:%u R1:%u R2:%u",
            gray_result[IDX_L2], gray_result[IDX_L1], gray_result[IDX_M],
            gray_result[IDX_R1], gray_result[IDX_R2]);
    ips200_show_string(x, y, buf);

    sprintf(buf, "DIR:%s %s       ",
            dir_str[track_last_dir], track_finished ? "STOP" : "RUN ");
    ips200_show_string(x, y + 10, buf);

    sprintf(buf, "MODE:%s              ",
            (TRACK_SPEED_MODE_LOW == track_cfg.speed_mode) ? "LOW " : "HIGH");
    ips200_show_string(x, y + 20, buf);
}
