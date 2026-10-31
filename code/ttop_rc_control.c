/*
 * ttop_rc_control.c
 *
 *  Created on: 2026-04-05
 *      Author: hhttiuui
 *
 *  赛事功能：遥控器通道控制。
 *  - CH0 -> g_yaw_target_deg
 *  - CH1 -> motor_set_target_counts_per_5ms
 */

#include "zf_common_headfile.h"
#include "ttop_rc_control.h"
#include "Menu.h"
#include "flash_storage.h"

static ttop_rc_control_cfg_t s_rc_cfg;
static float s_rc_speed_target_out = 0.0f;
static unsigned char s_rc_ch4_reset_latched = 0u;
static unsigned char s_rc_ch4_start_latched = 0u;
static unsigned char s_rc_emergency_active = 0u;

// ============================ RC Yaw 录制/回放（50ms采样） ============================
#ifndef TTOP_RC_YAW_SAMPLE_TICKS_5MS
#define TTOP_RC_YAW_SAMPLE_TICKS_5MS   (12u) // 12 * 5ms = 60ms
#endif

#ifndef TTOP_RC_YAW_RECORD_MAX_POINTS
#define TTOP_RC_YAW_RECORD_MAX_POINTS  (2038u) // 2 × 1019 点 × 60ms ≈ 122s
#endif

static float s_rc_yaw_record_buf[TTOP_RC_YAW_RECORD_MAX_POINTS];
static uint16 s_rc_yaw_record_count = 0u;
static uint16 s_rc_yaw_record_tick_5ms = 0u;
static uint8 s_rc_yaw_record_active = 0u;

static uint16 s_rc_yaw_replay_index = 0u;
static uint16 s_rc_yaw_replay_tick_5ms = 0u;
static uint8 s_rc_yaw_replay_active = 0u;

#ifndef TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS
#define TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS   (3.0f)  //1f：柔和，响应速度慢  2~3f：适中，响应速度适中  4~5f：快速，响应速度快，稳定性差
#endif

#ifndef TTOP_RC_LOW_SPEED_RUN_COUNTS_PER_5MS
#define TTOP_RC_LOW_SPEED_RUN_COUNTS_PER_5MS (120.0f)
#endif

static inline float ttop_rc_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float ttop_rc_apply_sensitivity(float x, float center, float sensitivity)
{
    if (sensitivity <= 0.0f) sensitivity = 1.0f;
    return center + (x - center) * sensitivity;
}

static float ttop_rc_map_axis(float in_raw, const ttop_rc_axis_cfg_t *cfg)
{
    float x = ttop_rc_apply_sensitivity(in_raw, cfg->in_center, cfg->sensitivity);
    x = ttop_rc_clampf(x, cfg->in_min, cfg->in_max);

    if (x <= cfg->in_center)
    {
        float den = (cfg->in_center - cfg->in_min);
        float t = (den > 0.0f) ? ((x - cfg->in_min) / den) : 0.0f;
        return cfg->out_min + t * (cfg->out_center - cfg->out_min);
    }
    else
    {
        float den = (cfg->in_max - cfg->in_center);
        float t = (den > 0.0f) ? ((x - cfg->in_center) / den) : 0.0f;
        return cfg->out_center + t * (cfg->out_max - cfg->out_center);
    }
}

static float ttop_rc_map_ch0_to_unit_cmd(float ch0)
{
    // 【修改点 1】增加摇杆中位死区
    // 如果摇杆偏离中位在 ±30 以内，强制回中，防止遥控器没回正导致“前轮偷偷打歪”
    float deadzone = 30.0f;
    if (ch0 > (s_rc_cfg.yaw.in_center - deadzone) && ch0 < (s_rc_cfg.yaw.in_center + deadzone))
    {
        ch0 = s_rc_cfg.yaw.in_center;
    }

    float x = ttop_rc_apply_sensitivity(ch0, s_rc_cfg.yaw.in_center, s_rc_cfg.yaw.sensitivity);
    x = ttop_rc_clampf(x, s_rc_cfg.yaw.in_min, s_rc_cfg.yaw.in_max);

    if (x <= s_rc_cfg.yaw.in_center)
    {
        float den = (s_rc_cfg.yaw.in_center - s_rc_cfg.yaw.in_min);
        return (den > 0.0f) ? ((x - s_rc_cfg.yaw.in_center) / den) : 0.0f; // [-1, 0]
    }
    else
    {
        float den = (s_rc_cfg.yaw.in_max - s_rc_cfg.yaw.in_center);
        return (den > 0.0f) ? ((x - s_rc_cfg.yaw.in_center) / den) : 0.0f; // [0, 1]
    }
}

static int16 ttop_rc_map_ch1_to_speed_counts(float ch1)
{
    float speed = ttop_rc_map_axis(ch1, &s_rc_cfg.speed);
    speed = ttop_rc_clampf(speed, s_rc_cfg.speed.out_min, s_rc_cfg.speed.out_max);

    // 【修改点 2】起步保底速度
    // 平衡车最难骑的就是低速(比如20~60)区间，很容易蛇形。
    // 我们强制：只要油门超了一点点(比如10)，就立刻给到一个稳定的保底速度(比如80)
    // 跨过危险起步区。
    float startup_threshold = 10.0f; // 起步推杆阈值
    float base_speed = (float)TTOP_RC_LOW_SPEED_RUN_COUNTS_PER_5MS; // 稳定的最低起步速度

    if (speed > startup_threshold && speed < base_speed)
    {
        speed = base_speed;
    }
    // 反向如果也有倒车逻辑，可以同样处理，这里假设是只往前开发车

    return (int16)roundf(speed);
}

void ttop_rc_control_init(void)
{
    // 初始化默认参数：后续可通过 ttop_rc_control_set_config 覆盖。
    s_rc_cfg.yaw.in_min = 200.0f;
    s_rc_cfg.yaw.in_center = 800.0f;
    s_rc_cfg.yaw.in_max = 1400.0f;
    s_rc_cfg.yaw.out_min = -1.0f;
    s_rc_cfg.yaw.out_center = 0.0f;
    s_rc_cfg.yaw.out_max = 1.0f;
    s_rc_cfg.yaw.sensitivity = 0.6f;
    s_rc_cfg.yaw_rate_limit_deg_per_5ms = 3.5f;

    s_rc_cfg.speed.in_min = 800.0f;
    s_rc_cfg.speed.in_center = 800.0f;
    s_rc_cfg.speed.in_max = 1400.0f;
    s_rc_cfg.speed.out_min = 0.0f;
    s_rc_cfg.speed.out_center = 0.0f;
    s_rc_cfg.speed.out_max = 600.0f;
    s_rc_cfg.speed.sensitivity = 1.0f;

    s_rc_speed_target_out = 0.0f;

    // 上电默认加载槽位 1；START 时会按菜单选择的槽位再次加载。
    (void)ttop_rc_yaw_record_load_from_flash(1u);
}

void ttop_rc_control_set_config(const ttop_rc_control_cfg_t *cfg)
{
    if (cfg == NULL) return;
    s_rc_cfg = *cfg;
}

const ttop_rc_control_cfg_t *ttop_rc_control_get_config(void)
{
    return &s_rc_cfg;
}

void ttop_rc_control_zero_reset_yaw_center_by_ch0(void)
{
    // yaw 控制使用 CH0 输入，因此中心校准读取 CH0。
    s_rc_cfg.yaw.in_center = (float)uart_receiver.channel[0];
}

void ttop_rc_control_emergency_stop_now(void)
{
    int16 ch4 = uart_receiver.channel[3];

    // CH4 一键复位：
    // - 150~250：触发一次 reset，并进入急停态
    // - 1750~1850：触发一次 start，并退出急停态
    // - 950~1050：不操作，退出急停态并清除锁存，允许下一次触发
    if ((ch4 >= 150) && (ch4 <= 250))
    {
        if (!s_rc_ch4_reset_latched)
        {
            Menu_ResetAll();
            s_rc_ch4_reset_latched = 1u;
        }

        s_rc_emergency_active = 1u;
        s_rc_speed_target_out = 0.0f;
        //motor_set_target_counts_per_5ms(0);
    }
    else if ((ch4 >= 1750) && (ch4 <= 1850))
    {
        if (!s_rc_ch4_start_latched)
        {
            Menu_StartAll();
            s_rc_ch4_start_latched = 1u;
        }

        s_rc_emergency_active = 0u;
    }
    else if ((ch4 >= 950) && (ch4 <= 1050))
    {
        s_rc_ch4_reset_latched = 0u;
        s_rc_ch4_start_latched = 0u;
        s_rc_emergency_active = 0u;
    }
}

void ttop_rc_control_task_5ms(void)
{
    // 遥控器未就绪或失控时不更新控制量。
    if (uart_receiver.finsh_flag == 0u) return;
    if (uart_receiver.state == 0u)
    {
        uart_receiver.finsh_flag = 0u;
        return;
    }

    float yaw_unit_cmd = ttop_rc_map_ch0_to_unit_cmd((float)uart_receiver.channel[0]);
    float yaw_rate_lim = s_rc_cfg.yaw_rate_limit_deg_per_5ms;
    float speed_target_raw;
    float delta;

    if (yaw_rate_lim < 0.0f) yaw_rate_lim = -yaw_rate_lim;

    // CH4 急停态下，持续屏蔽本周期控制输出（CH4 判定已移到 emergency_stop 函数）
    if (s_rc_emergency_active)
    {
        s_rc_speed_target_out = 0.0f;
        motor_set_target_counts_per_5ms(0);
        uart_receiver.finsh_flag = 0u;
        return;
    }

    // 【修改点 3】低速转向衰减
    // 当车速极低时（甚至0速），强烈打方向容易直接破坏平衡倒车（因为没有足够前向稳定和陀螺效应）
    // 所以在低速时，让它打方向的反应变“迟钝”一点。
    float current_speed_abs = s_rc_speed_target_out;
    if(current_speed_abs < 0) current_speed_abs = -current_speed_abs;

    float yaw_attenuation = 1.0f; // 满舵
    if (current_speed_abs < ((float)TTOP_RC_LOW_SPEED_RUN_COUNTS_PER_5MS * 0.4f)) {
        yaw_attenuation = 0.2f; // 极低速，只有 20% 舵量
    } else if (current_speed_abs < (float)TTOP_RC_LOW_SPEED_RUN_COUNTS_PER_5MS) {
        yaw_attenuation = 0.5f; // 起步初段，一半舵量
    }

    g_yaw_target_deg -= yaw_unit_cmd * yaw_rate_lim * yaw_attenuation;

    speed_target_raw = (float)ttop_rc_map_ch1_to_speed_counts((float)uart_receiver.channel[1]);
    delta = speed_target_raw - s_rc_speed_target_out;

    if (delta > TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS)
        delta = TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS;
    else if (delta < -TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS)
        delta = -TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS;

    s_rc_speed_target_out += delta;
    motor_set_target_counts_per_5ms((int16)roundf(s_rc_speed_target_out));

    uart_receiver.finsh_flag = 0u;
}

void ttop_rc_control_speed_only_task_5ms(void)
{
    // 仅做速度控制：保留与 ttop_rc_control_task_5ms 相同的速度映射与斜坡策略。
    if (uart_receiver.finsh_flag == 0u) return;
    if (uart_receiver.state == 0u)
    {
        uart_receiver.finsh_flag = 0u;
        return;
    }

    // CH4 急停态下，持续屏蔽本周期控制输出。
    if (s_rc_emergency_active)
    {
        s_rc_speed_target_out = 0.0f;
        motor_set_target_counts_per_5ms(0);
        uart_receiver.finsh_flag = 0u;
        return;
    }

    {
        float speed_target_raw = (float)ttop_rc_map_ch1_to_speed_counts((float)uart_receiver.channel[1]);
        float delta = speed_target_raw - s_rc_speed_target_out;

        if (delta > TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS)
            delta = TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS;
        else if (delta < -TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS)
            delta = -TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS;

        s_rc_speed_target_out += delta;
        motor_set_target_counts_per_5ms((int16)roundf(s_rc_speed_target_out));
    }

    uart_receiver.finsh_flag = 0u;
}

void ttop_rc_control_yaw_only_task_5ms(void)
{
    // MODE3 运行开关：未开启时直接返回
    if (!menu_mode3_run_yaw_only_enabled()) return;
    // 回放开启时，避免与回放同时改写 g_yaw_target_deg
    if (menu_mode3_run_replay_enabled()) return;

    // 仅做方向（yaw）控制：保留与 ttop_rc_control_task_5ms 相同的 CH0 映射与速率限制。
    if (uart_receiver.finsh_flag == 0u) return;
    if (uart_receiver.state == 0u)
    {
        uart_receiver.finsh_flag = 0u;
        return;
    }

    // CH4 急停态下，持续屏蔽本周期方向输出。
    if (s_rc_emergency_active)
    {
        uart_receiver.finsh_flag = 0u;
        return;
    }

    {
        float yaw_unit_cmd = ttop_rc_map_ch0_to_unit_cmd((float)uart_receiver.channel[0]);
        float yaw_rate_lim = s_rc_cfg.yaw_rate_limit_deg_per_5ms;

        if (yaw_rate_lim < 0.0f) yaw_rate_lim = -yaw_rate_lim;

        // 低速转向衰减：根据当前速度目标降低舵量
        float current_speed_abs = s_rc_speed_target_out;
        if (current_speed_abs < 0) current_speed_abs = -current_speed_abs;

        float yaw_attenuation = 1.0f;
        if (current_speed_abs < ((float)TTOP_RC_LOW_SPEED_RUN_COUNTS_PER_5MS * 0.4f))
        {
            yaw_attenuation = 0.2f;
        }
        else if (current_speed_abs < (float)TTOP_RC_LOW_SPEED_RUN_COUNTS_PER_5MS)
        {
            yaw_attenuation = 0.5f;
        }

        g_yaw_target_deg -= yaw_unit_cmd * yaw_rate_lim * yaw_attenuation;
    }

    uart_receiver.finsh_flag = 0u;
}

void ttop_rc_control_yaw_direct_task_5ms(void)
{
    // 不做 MODE3 守卫，不做速度输出，纯 CH0→yaw
    if (uart_receiver.finsh_flag == 0u) return;
    if (uart_receiver.state == 0u)
    {
        uart_receiver.finsh_flag = 0u;
        return;
    }
    if (s_rc_emergency_active)
    {
        uart_receiver.finsh_flag = 0u;
        return;
    }
    {
        float yaw_unit_cmd = ttop_rc_map_ch0_to_unit_cmd((float)uart_receiver.channel[0]);
        float yaw_rate_lim = s_rc_cfg.yaw_rate_limit_deg_per_5ms;
        if (yaw_rate_lim < 0.0f) yaw_rate_lim = -yaw_rate_lim;
        // 录制模式：不做低速衰减，不做额外限制，忠实记录遥控操作
        g_yaw_target_deg -= yaw_unit_cmd * yaw_rate_lim;
    }
    uart_receiver.finsh_flag = 0u;
}

void ttop_rc_yaw_record_start(void)
{
    s_rc_yaw_record_active = 1u;
    s_rc_yaw_replay_active = 0u;
    s_rc_yaw_record_tick_5ms = 0u;
    s_rc_yaw_record_count = 0u;

    // 记录起始点
    s_rc_yaw_record_buf[s_rc_yaw_record_count++] = g_yaw_target_deg;
}

void ttop_rc_yaw_record_stop(void)
{
    s_rc_yaw_record_active = 0u;
}

void ttop_rc_yaw_record_task_5ms(void)
{
    // MODE3 运行开关：未开启时直接返回
    if (!menu_mode3_run_record_enabled()) return;

    if (!s_rc_yaw_record_active) return;
    if (s_rc_yaw_record_count >= (uint16)TTOP_RC_YAW_RECORD_MAX_POINTS)
    {
        s_rc_yaw_record_active = 0u;
        return;
    }

    s_rc_yaw_record_tick_5ms++;
    if (s_rc_yaw_record_tick_5ms >= (uint16)TTOP_RC_YAW_SAMPLE_TICKS_5MS)
    {
        s_rc_yaw_record_tick_5ms = 0u;
        s_rc_yaw_record_buf[s_rc_yaw_record_count++] = g_yaw_target_deg;
    }
}

void ttop_rc_yaw_replay_start(void)
{
    if (s_rc_yaw_record_count == 0u)
    {
        s_rc_yaw_replay_active = 0u;
        return;
    }

    s_rc_yaw_record_active = 0u;
    s_rc_yaw_replay_active = 1u;
    s_rc_yaw_replay_tick_5ms = 0u;
    s_rc_yaw_replay_index = 0u;

    // 回放起点立即生效
    g_yaw_target_deg = s_rc_yaw_record_buf[0u];
    s_rc_yaw_replay_index = 1u;
}

void ttop_rc_yaw_replay_stop(void)
{
    s_rc_yaw_replay_active = 0u;
}

void ttop_rc_yaw_replay_task_5ms(void)
{
    // MODE3 运行开关：未开启时直接返回
    if (!menu_mode3_run_replay_enabled()) return;

    if (!s_rc_yaw_replay_active) return;
    if (s_rc_yaw_replay_index >= s_rc_yaw_record_count)
    {
        s_rc_yaw_replay_active = 0u;
        return;
    }

    s_rc_yaw_replay_tick_5ms++;
    if (s_rc_yaw_replay_tick_5ms >= (uint16)TTOP_RC_YAW_SAMPLE_TICKS_5MS)
    {
        s_rc_yaw_replay_tick_5ms = 0u;
        g_yaw_target_deg = s_rc_yaw_record_buf[s_rc_yaw_replay_index++];
    }
}

uint8 ttop_rc_yaw_record_save_to_flash(uint8 slot)
{
    return flash_storage_save_rc_yaw_record(
        (const float *)s_rc_yaw_record_buf,
    s_rc_yaw_record_count,
    slot);
}

uint8 ttop_rc_yaw_record_load_from_flash(uint8 slot)
{
    uint16 count = 0u;
    if (!flash_storage_load_rc_yaw_record(
            (float *)s_rc_yaw_record_buf,
            (uint16)TTOP_RC_YAW_RECORD_MAX_POINTS,
        &count,
        slot))
    {
        return 0u;
    }

    s_rc_yaw_record_count = count;
    s_rc_yaw_record_active = 0u;
    s_rc_yaw_replay_active = 0u;
    s_rc_yaw_record_tick_5ms = 0u;
    s_rc_yaw_replay_tick_5ms = 0u;
    s_rc_yaw_replay_index = 0u;
    return 1u;
}

uint8 ttop_rc_yaw_record_is_active(void)
{
    return s_rc_yaw_record_active;
}

uint8 ttop_rc_yaw_replay_is_active(void)
{
    return s_rc_yaw_replay_active;
}

uint16 ttop_rc_yaw_record_get_count(void)
{
    return s_rc_yaw_record_count;
}

uint16 ttop_rc_yaw_replay_get_index(void)
{
    return s_rc_yaw_replay_index;
}
