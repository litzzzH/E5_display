/*
 * tracking.c
 * 循迹模块 - MSPM0G3507 移植版（陀螺仪直角弯）
 *
 * 线偏左 → 车要左转 → 右轮快左轮慢
 * 线偏右 → 车要右转 → 左轮快右轮慢
 */

#include "tracking.h"

track_runtime_config_t track_cfg = {
    0,
    TRACK_SPEED_MODE_LOW,
    4,
    3,
    900,
    400,
    0,
    300,
    2,
    500,
    2000,
    {
        {3000, 3000, 200, 400, 700, 450, 1000, 800, 800, 30, 500, 30},
        {4000, 4000, 200, 400, 400, 250, 1500, 800, 800, 30, 500, 30}
    }
};

track_state_e track_state    = TRACK_STATE_NORMAL;
track_dir_e   track_last_dir = TRACK_DIR_CENTER;
uint8         track_turning  = 0;
uint16        track_turn_count = 0;     // 已完成直角弯数
uint8         track_lap_count  = 0;     // 已完成圈数
uint8         track_finished   = 0;     // 停车标志

static uint8  track_turn_prepare_done = 0;     // 直角弯前直行阶段完成标志
static uint32 track_turn_prepare_ms   = 0;     // 检测直角弯时刻(ms)
static uint8  track_accelerating      = 0;     // 转弯后加速阶段标志
static uint32 track_accel_start_ms    = 0;     // 加速阶段开始时刻(ms)
static uint8  track_startup           = 1;     // 启动加速标志（1=首次启动）
static uint32 track_startup_ms        = 0;     // 启动时刻(ms)
static uint8  ra_l2_count             = 0;     // L2持续触发计数
static uint8  ra_r2_count             = 0;     // R2持续触发计数
static uint32 track_turn_cooldown_ms  = 0;     // 转弯完成时刻(用于冷却期)
static uint8  track_line_lost_active  = 0;     // 连续全白标志
static uint32 track_line_lost_ms      = 0;     // 连续全白起始时刻

static const char* state_str[] = {"NORMAL", "R-ANGLE", "TURNIN"};
static const char* dir_str[]   = {"CENTER", "LEFT  ", "RIGHT "};

#define L2  (gray_result[IDX_L2])
#define L1  (gray_result[IDX_L1])
#define M   (gray_result[IDX_M])
#define R1  (gray_result[IDX_R1])
#define R2  (gray_result[IDX_R2])

static void tracking_check_state(void);
static void tracking_handle_normal(void);
static void tracking_handle_right_angle(void);
static void tracking_set_motor(int left_pwm, int right_pwm);

void tracking_reset_runtime_state(void)
{
    track_state    = TRACK_STATE_NORMAL;
    track_last_dir = TRACK_DIR_CENTER;
    track_turning  = 0;
    track_turn_count = 0;
    track_lap_count  = 0;
    track_finished   = 0;
    track_turn_prepare_done = 0;
    track_turn_prepare_ms = 0;
    track_accelerating = 0;
    track_accel_start_ms = 0;
    track_startup  = 1;
    track_startup_ms = system_getval_ms();
    ra_l2_count = 0;
    ra_r2_count = 0;
    track_turn_cooldown_ms = 0;
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
    tracking_check_state();

    switch(track_state)
    {
        case TRACK_STATE_NORMAL:
            tracking_handle_normal();
            break;
        case TRACK_STATE_RIGHT_ANGLE:
            tracking_handle_right_angle();
            break;
        default:
            tracking_handle_normal();
            break;
    }
}

// ---------- 状态判断 ----------
static void tracking_check_state(void)
{
    if(track_turning)
    {
        ra_l2_count = 0;
        ra_r2_count = 0;
        return;
    }

    // 转弯完成后冷却期，防止在路口附近重复检测
    if(track_turn_cooldown_ms != 0
       && (system_getval_ms() - track_turn_cooldown_ms) < TRACK_TURN_COOLDOWN_MS)
    {
        ra_l2_count = 0;
        ra_r2_count = 0;
        track_state = TRACK_STATE_NORMAL;
        return;
    }

    // 更新L2/R2持续触发计数
    if(L2 && !R2) ra_l2_count++; else ra_l2_count = 0;
    if(R2 && !L2) ra_r2_count++; else ra_r2_count = 0;

    // ---- 多传感器条件：立即触发 ----

    // 左侧多路检测到 → 左直角弯
    if(L2 && L1 && M && R1 && !R2)
    {
        track_state = TRACK_STATE_RIGHT_ANGLE;
        track_last_dir = TRACK_DIR_LEFT;
        ra_l2_count = 0;
        return;
    }
    if(L2 && L1 && M && !R1 && !R2)
    {
        track_state = TRACK_STATE_RIGHT_ANGLE;
        track_last_dir = TRACK_DIR_LEFT;
        ra_l2_count = 0;
        return;
    }
    if(L2 && L1 && !R1 && !R2)
    {
        track_state = TRACK_STATE_RIGHT_ANGLE;
        track_last_dir = TRACK_DIR_LEFT;
        ra_l2_count = 0;
        return;
    }
    if(L2 && M && !R1 && !R2)
    {
        track_state = TRACK_STATE_RIGHT_ANGLE;
        track_last_dir = TRACK_DIR_LEFT;
        ra_l2_count = 0;
        return;
    }
    if(L2 && M && R1 && !R2)
    {
        track_state = TRACK_STATE_RIGHT_ANGLE;
        track_last_dir = TRACK_DIR_LEFT;
        ra_l2_count = 0;
        return;
    }
    if(L2 && L1 && R1 && !R2)
    {
        track_state = TRACK_STATE_RIGHT_ANGLE;
        track_last_dir = TRACK_DIR_LEFT;
        ra_l2_count = 0;
        return;
    }
    // 右侧多路检测到 → 右直角弯
    if(!L2 && L1 && M && R1 && R2)
    {
        track_state = TRACK_STATE_RIGHT_ANGLE;
        track_last_dir = TRACK_DIR_RIGHT;
        ra_r2_count = 0;
        return;
    }
    if(!L2 && !L1 && M && R1 && R2)
    {
        track_state = TRACK_STATE_RIGHT_ANGLE;
        track_last_dir = TRACK_DIR_RIGHT;
        ra_r2_count = 0;
        return;
    }
    if(!L2 && !L1 && R1 && R2)
    {
        track_state = TRACK_STATE_RIGHT_ANGLE;
        track_last_dir = TRACK_DIR_RIGHT;
        ra_r2_count = 0;
        return;
    }
    if(!L2 && !L1 && M && R2)
    {
        track_state = TRACK_STATE_RIGHT_ANGLE;
        track_last_dir = TRACK_DIR_RIGHT;
        ra_r2_count = 0;
        return;
    }
    if(!L2 && L1 && M && R2)
    {
        track_state = TRACK_STATE_RIGHT_ANGLE;
        track_last_dir = TRACK_DIR_RIGHT;
        ra_r2_count = 0;
        return;
    }
    if(!L2 && L1 && R1 && R2)
    {
        track_state = TRACK_STATE_RIGHT_ANGLE;
        track_last_dir = TRACK_DIR_RIGHT;
        ra_r2_count = 0;
        return;
    }

    // ---- 仅L2或仅R2：持续N个周期确认后触发 ----
    if(ra_l2_count >= TRACK_RA_PERSIST_CYCLES)
    {
        track_state = TRACK_STATE_RIGHT_ANGLE;
        track_last_dir = TRACK_DIR_LEFT;
        ra_l2_count = 0;
        return;
    }
    if(ra_r2_count >= TRACK_RA_PERSIST_CYCLES)
    {
        track_state = TRACK_STATE_RIGHT_ANGLE;
        track_last_dir = TRACK_DIR_RIGHT;
        ra_r2_count = 0;
        return;
    }

    track_state = TRACK_STATE_NORMAL;
}

// ---------- 普通循迹 ----------
// 核心原则：线偏左 → 车要左转 → 右轮快左轮慢
//           线偏右 → 车要右转 → 左轮快右轮慢
static void tracking_handle_normal(void)
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

    // 转弯后线性加速阶段：缩放基准PWM
    if(track_accelerating)
    {
        uint32 elapsed = system_getval_ms() - track_accel_start_ms;
        if(elapsed < TRACK_TURN_ACCEL_MS)
        {
            float ratio = TRACK_TURN_ACCEL_MIN
                + (1.0f - TRACK_TURN_ACCEL_MIN) * (float)elapsed / (float)TRACK_TURN_ACCEL_MS;
            base_l = (int)(base_l * ratio);
            base_r = (int)(base_r * ratio);
        }
        else
        {
            track_accelerating = 0;
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

    // L2+L1同时检测到 → 直角弯已在check_state处理，这里做大幅修正兜底
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

// ---------- 直角弯处理（陀螺仪方式，移植自wanmo原始逻辑）----------
static void tracking_handle_right_angle(void)
{
    float yaw_abs;

    // 首次进入直角弯：记录时间，进入直行过渡阶段
    if(!track_turning)
    {
        track_turning = 1;
        track_turn_prepare_done = 0;
        track_turn_prepare_ms = system_getval_ms();
        imu660rb_yaw_reset();          // 清零yaw角
    }

    // 直角弯前线性减速直行，让车身到路口并降到低速
    if(!track_turn_prepare_done)
    {
        uint32 elapsed_ms = system_getval_ms() - track_turn_prepare_ms;
        if(elapsed_ms < TRACK_TURN_PREPARE_MS)
        {
            // 线性减速：从100%基准PWM降到0
            float ratio = 1.0f - (float)elapsed_ms / (float)TRACK_TURN_PREPARE_MS;
            int pwm_l = (int)(TRACK_BASE_PWM_L * ratio);
            int pwm_r = (int)(TRACK_BASE_PWM_R * ratio);
            tracking_set_motor(pwm_l, pwm_r);
            return;
        }
        Motor_Stop();
        track_turn_prepare_done = 1;
        imu660rb_yaw_reset();          // 从此刻开始计角度
    }

    // 获取当前角度绝对值
    yaw_abs = imu660rb_yaw_get();
    if(yaw_abs < 0) yaw_abs = -yaw_abs;

    // 还没转够目标角度，继续转弯
    if(yaw_abs < TRACK_TURN_ANGLE)
    {
        float remain = TRACK_TURN_ANGLE - yaw_abs;
        int pwm;

        // 提前退出：接近目标角度且中间探头M已检测到线
        if((remain < TRACK_TURN_M_EXIT_DEG) && M)
        {
            Motor_Stop();
            track_turning = 0;
            track_turn_prepare_done = 0;
            track_state   = TRACK_STATE_NORMAL;
            track_last_dir = TRACK_DIR_CENTER;
            imu660rb_yaw_reset();
            track_accelerating = 1;
            track_accel_start_ms = system_getval_ms();
            track_turn_cooldown_ms = system_getval_ms();
            // 计圈
            track_turn_count++;
            track_lap_count = track_turn_count / TRACK_TURNS_PER_LAP;
            if(track_lap_count >= TRACK_TOTAL_LAPS)
                track_finished = 1;
            return;
        }

        // 分段PWM：距目标角度小时线性减速
        if(remain < TRACK_TURN_BRAKE_DEG)
        {
            Motor_Stop();
            return;
        }
        else if(remain < TRACK_TURN_SLOW_DEG)
        {
            // 线性插值：PWM从 PWM_MIN 到 TURN_PWM
            pwm = TRACK_TURN_PWM_MIN + (int)((float)(TRACK_TURN_PWM - TRACK_TURN_PWM_MIN)
                  * (remain - TRACK_TURN_BRAKE_DEG) / (TRACK_TURN_SLOW_DEG - TRACK_TURN_BRAKE_DEG));
        }
        else
        {
            pwm = TRACK_TURN_PWM;
        }

        // 差速原地转弯
        if(track_last_dir == TRACK_DIR_LEFT)
            tracking_set_motor(-pwm, pwm);
        else
            tracking_set_motor(pwm, -pwm);
    }
    else
    {
        // 已转够目标角度，停止转弯，回到普通循迹
        Motor_Stop();
        track_turning = 0;
        track_turn_prepare_done = 0;
        track_state   = TRACK_STATE_NORMAL;
        track_last_dir = TRACK_DIR_CENTER;
        imu660rb_yaw_reset();
        track_accelerating = 1;
        track_accel_start_ms = system_getval_ms();
        track_turn_cooldown_ms = system_getval_ms();
        // 计圈
        track_turn_count++;
        track_lap_count = track_turn_count / TRACK_TURNS_PER_LAP;
        if(track_lap_count >= TRACK_TOTAL_LAPS)
            track_finished = 1;
    }
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

    sprintf(buf, "ST:%s DIR:%s   ",
            state_str[track_turning ? 2 : track_state], dir_str[track_last_dir]);
    ips200_show_string(x, y + 10, buf);

    sprintf(buf, "T:%u LAP:%u/%u %s",
            track_turn_count, track_lap_count, TRACK_TOTAL_LAPS,
            track_finished ? "DONE" : "    ");
    ips200_show_string(x, y + 20, buf);
}
