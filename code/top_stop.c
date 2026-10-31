/*
 * top_stop.c
 *
 * 赛事功能：里程到点停机
 */

#include "zf_common_headfile.h"

// 可调：目标距离（米），超过后停止一切调控
#ifndef STOP_DISTANCE_M
#define STOP_DISTANCE_M              (20LL)
#endif

static volatile uint8 s_stop_all_ctrl = 0;

static inline void top_stop_apply_now(void)
{
    // 安全停机：立即清电机占空比，并让舵机回中
    small_driver_set_duty(0, 0);
    pwm_set_duty(PWM_CH1, SERVO_PWM(0));
}

void ttop_stop_init(void)
{
    s_stop_all_ctrl = 0;
}

uint8 ttop_stop_is_active(void)
{
    return s_stop_all_ctrl;
}


//=======停机任务，在哪调用都行=======//
void ttop_stop_check_and_apply(void)
{
    if (s_stop_all_ctrl) return;

    // 总计数与 delta 使用同一符号约定，取绝对值用于里程判断
    int64 total_cnt = encoder_get_demo5_count_total();
    if (total_cnt < 0) total_cnt = -total_cnt;

    int64 meters = 0;
    if (ENCODER_COUNTS_PER_M > 0)
    {
        meters = total_cnt / ENCODER_COUNTS_PER_M;
    }

    if (meters >= STOP_DISTANCE_M)
    {
        s_stop_all_ctrl = 1;
        top_stop_apply_now();
    }
}
