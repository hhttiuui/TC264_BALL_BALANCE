/*
 * liangan.c  —— 摄像头 UART 数据接收（UART3 原 GPS 接口）
 *
 * 兼容两种帧格式：
 *   旧: AA XH XL FF               (4字节, 115200)
 *   新: AA XH XL SpeH SpeL FF     (6字节, 460800)
 * XH|XL 合并为小球位置值（0~800）
 * SpeH|SpeL 合并为速度值（int16 ×10 → uint16）
 *
 * 调用：在 user/isr.c 的 uart3_rx_isr 中调用 liangan_uart_callback()
 */

#include "liangan.h"
#include "zf_driver_uart.h"
#include "IfxStm.h"
#include "top_8_turn_2.h"
#include "common_flie.h"
#include "zf_device_uart_receiver.h"
#include "flash_storage.h"

/* ---- RC 录制参数 ---- */
#define LIANGAN_RC_SAMPLE_TICKS_5MS   (2u)    /* 2×5ms = 10ms 采样 */
#define LIANGAN_RC_RECORD_MAX_POINTS  (2038u) /* 与 RC Yaw 共用 Flash 格式 */
static float s_liangan_rc_rec_buf[LIANGAN_RC_RECORD_MAX_POINTS];
static uint8_t  s_liangan_rc_rec_active  = 0u;
static uint8_t  s_liangan_rc_play_active = 0u;
static uint16_t s_liangan_rc_rec_count   = 0u;
static uint16_t s_liangan_rc_rec_tick    = 0u;
static uint16_t s_liangan_rc_play_idx    = 0u;
static uint16_t s_liangan_rc_play_tick   = 0u;

/* ---- 帧格式 ---- */
#define FRAME_HEAD  (0xAAu)
#define FRAME_TAIL  (0xFFu)

/* ---- STM 时钟频率（200MHz → 5ns/tick） ---- */
#define STM_FREQ_HZ  (200000000u)

/* ---- 接收状态机 ---- */
typedef enum {
    STATE_IDLE,         /* 等待 0xAA */
    STATE_GOT_AA,       /* 已收 0xAA，等待 XH */
    STATE_GOT_XH,       /* 已收 XH，等待 XL */
    STATE_GOT_XL,       /* 已收 XL，判断旧格式 FF 或新格式 SpeH */
    STATE_GOT_SH,       /* 新格式：已收 SpeH，等待 SpeL */
    STATE_GOT_SL        /* 新格式：已收 SpeL，等待 0xFF */
} rx_state_t;

static rx_state_t s_state = STATE_IDLE;
static uint8_t    s_xh   = 0u;
static uint8_t    s_sh   = 0u;

/* ---- STM 速度计算 ---- */
static uint64  s_stm_last      = 0u;
static float    s_pos_filt_last = 400.0f;
static uint8_t  s_has_last     = 0u;

static void calc_speed(void)
{
    uint64 t_now = IfxStm_get(IfxStm_getAddress(
        (IfxStm_Index)IfxCpu_getCoreId()));

    /* 一阶低通滤波：位置 */
    g_liangan_ball_filt = g_liangan_alpha_pos * g_liangan_ball_filt
                        + (1.0f - g_liangan_alpha_pos) * (float)g_liangan_ball_pos;

    if (s_has_last)
    {
        float  diff_filt = g_liangan_ball_filt - s_pos_filt_last;
        uint64 dt_ticks  = t_now - s_stm_last;
        float  dt_s      = (float)dt_ticks / (float)STM_FREQ_HZ;
        if (dt_s > 0.001f)
        {
            /* 速度由滤波后的位置差分得到，无需单独滤波 */
            g_liangan_speed_calc = diff_filt / dt_s;
        }
    }
    s_stm_last      = t_now;
    s_pos_filt_last = g_liangan_ball_filt;
    s_has_last      = 1u;
}

/* ---- 公共变量 ---- */
volatile uint16_t g_liangan_ball_pos  = 400u;
volatile uint16_t g_liangan_target    = 400u;
volatile int16_t  g_liangan_center_offset = -20; /* 中心偏离量，叠加在 400 上，默认 380 */
volatile int16_t  g_liangan_sweep_ofs1    = -120; /* 扫目标第一阶段偏移 */
volatile int16_t  g_liangan_sweep_ofs2    = 150;  /* 扫目标第二阶段偏移 */
volatile uint16_t g_liangan_sweep_time_ms = 3500; /* 扫目标总时长 ms */
volatile uint8_t  g_liangan_sweep_mode = LIANGAN_MODE_CLOSED;  /* 默认闭环 */
volatile int16_t  g_liangan_speed     = 0;
volatile float    g_liangan_speed_calc = 0.0f;
volatile float    g_liangan_ball_filt  = 400.0f;
volatile float    g_liangan_speed_filt = 0.0f;
volatile float    g_liangan_alpha_pos  = 0.00f;
volatile float    g_liangan_alpha_spd  = 0.90f;
volatile uint8_t  g_liangan_new_data   = 0u;
volatile uint32_t g_liangan_isr_count  = 0u;

/* ================================================================
 *  liangan_init — 初始化 UART3（460800 波特，兼容旧 115200）
 * ================================================================ */
void liangan_init(void)
{
    uart_init(UART_3, 460800, UART3_TX_P15_6, UART3_RX_P15_7);
    uart_rx_interrupt(UART_3, 1);
    g_liangan_target = (uint16_t)(400 + g_liangan_center_offset);
}

/* ================================================================
 *  liangan_uart_callback — 在 UART3 RX ISR 中调用
 *  支持 4 字节旧格式 + 6 字节新格式自动识别
 * ================================================================ */
void liangan_uart_callback(void)
{
    uint8_t dat;

    g_liangan_isr_count++;

    while (uart_query_byte(UART_3, &dat))
    {
        switch (s_state)
        {
        case STATE_IDLE:
            if (dat == FRAME_HEAD)
            {
                s_state = STATE_GOT_AA;
            }
            break;

        case STATE_GOT_AA:
            s_xh   = dat;
            s_state = STATE_GOT_XH;
            break;

        case STATE_GOT_XH:
            g_liangan_ball_pos = ((uint16_t)s_xh << 8) | dat;
            s_state = STATE_GOT_XL;
            break;

        case STATE_GOT_XL:
            if (dat == FRAME_TAIL)
            {
                /* 旧格式 4 字节帧：AA XH XL FF */
                g_liangan_new_data = 1u;
                calc_speed();
                s_state = STATE_IDLE;
            }
            else
            {
                /* 新格式 6 字节帧：继续收 SpeH */
                s_sh    = dat;
                s_state = STATE_GOT_SH;
            }
            break;

        case STATE_GOT_SH:
            {
                /* 合并速度值（uint16 → int16） */
                uint16_t raw = ((uint16_t)s_sh << 8) | dat;
                g_liangan_speed = (int16_t)raw;
                s_state = STATE_GOT_SL;
            }
            break;

        case STATE_GOT_SL:
            if (dat == FRAME_TAIL)
            {
                /* 新格式 6 字节帧：AA XH XL SpeH SpeL FF */
                g_liangan_new_data = 1u;
                calc_speed();
            }
            s_state = STATE_IDLE;
            break;
        }
    }
}

/* ================================================================
 *  开环扫目标（MODE3 用，g_liangan_sweep_closedloop==0）
 *  读取 EX1 段表：target_rel_deg=目标倾角(deg)，rate_dps=过渡速率(deg/s)
 * ================================================================ */
static uint8_t  s_sweep_ol_active   = 0u;
static uint16_t s_sweep_ol_seg_idx  = 0u;
static float    s_sweep_ol_angle    = 0.0f;

static void liangan_sweep_openloop_task_5ms(void)
{
    if (!s_sweep_ol_active) return;

    volatile ttop_eight2_seg_t *seg = &g_eight2_prog_ex1[s_sweep_ol_seg_idx];
    float target_deg = seg->target_rel_deg;
    float rate_dps   = seg->rate_dps;

    /* 结束条件：目标=0 且 速率=0 的段视为结束 */
    if (target_deg == 0.0f && rate_dps == 0.0f)
    {
        s_sweep_ol_active = 0u;
        return;
    }

    /* 每 5ms 一步：delta = rate * 0.005 */
    float step = rate_dps * 0.005f;
    float err  = target_deg - s_sweep_ol_angle;
    if (err > step)       s_sweep_ol_angle += step;
    else if (err < -step)  s_sweep_ol_angle -= step;
    else                  { s_sweep_ol_angle = target_deg; s_sweep_ol_seg_idx++; }

    /* 直接写角度目标到 pid_target（DAMPED 策略的角度环输入） */
    pid_target = s_sweep_ol_angle;
}

static void liangan_sweep_openloop_start(void)
{
    s_sweep_ol_active  = 1u;
    s_sweep_ol_seg_idx = 0u;
    s_sweep_ol_angle   = 0.0f;
}

static void liangan_sweep_openloop_reset(void)
{
    s_sweep_ol_active  = 0u;
    s_sweep_ol_seg_idx = 0u;
    s_sweep_ol_angle   = 0.0f;
    pid_target = 0.0f;  /* 目标倾角归零 */
}

/* ================================================================
 *  闭环扫目标（MODE3 用，g_liangan_sweep_closedloop==1）
 *  每 5ms 调用，总时长 g_liangan_sweep_time_ms：center → center+ofs1 → center+ofs2
 * ================================================================ */

volatile uint8_t g_liangan_sweep_active = 0u;
static uint16_t s_sweep_cnt = 0u;
static uint16_t s_sweep_half = 400u;  /* 运行时由 sweep_time_ms 算出 */

void liangan_auto_sweep_start(void)
{
    if (g_liangan_sweep_mode == LIANGAN_MODE_CLOSED)
    {
        g_liangan_sweep_active = 1u;
        s_sweep_cnt = 0u;
        s_sweep_half = g_liangan_sweep_time_ms / 10u;
        if (s_sweep_half < 1u) s_sweep_half = 1u;
        g_liangan_target = (uint16_t)(400 + g_liangan_center_offset);
    }
    else if (g_liangan_sweep_mode == LIANGAN_MODE_OPEN)
    {
        liangan_sweep_openloop_start();
    }
    else if (g_liangan_sweep_mode == LIANGAN_MODE_RC_REC)
    {
        liangan_rc_record_start();
    }
    else if (g_liangan_sweep_mode == LIANGAN_MODE_RC_PLAY)
    {
        liangan_rc_replay_start();
    }
    /* LIANGAN_MODE_RC_DIR: 无需 START，始终由 task 直驱 */
}

void liangan_auto_sweep_reset(void)
{
    g_liangan_sweep_active = 0u;
    s_sweep_cnt  = 0u;
    g_liangan_target = (uint16_t)(400 + g_liangan_center_offset);
    liangan_sweep_openloop_reset();
    liangan_rc_record_stop();
    liangan_rc_replay_stop();
    pid_target = 0.0f;  /* 目标倾角归零（梁水平） */
}

void liangan_auto_sweep_task_5ms(void)
{
    if (g_liangan_sweep_mode == LIANGAN_MODE_CLOSED)
    {
        if (!g_liangan_sweep_active) return;
        float center = 400.0f + (float)g_liangan_center_offset;
        float ofs1  = (float)g_liangan_sweep_ofs1;
        float ofs2  = (float)g_liangan_sweep_ofs2;
        float half  = (float)s_sweep_half;
        if (s_sweep_cnt < s_sweep_half)
        {
            float t = (float)s_sweep_cnt / half;
            float val = center + t * ofs1;
            g_liangan_target = (uint16_t)(val + 0.5f);
        }
        else
        {
            float t = (float)(s_sweep_cnt - s_sweep_half) / half;
            float val = (center + ofs1) + t * (ofs2 - ofs1);
            g_liangan_target = (uint16_t)(val + 0.5f);
        }
        s_sweep_cnt++;
        if (s_sweep_cnt >= (uint16_t)(s_sweep_half * 2u))
        {
            g_liangan_sweep_active = 0u;
        }
    }
    else if (g_liangan_sweep_mode == LIANGAN_MODE_OPEN)
    {
        liangan_sweep_openloop_task_5ms();
    }
    else if (g_liangan_sweep_mode == LIANGAN_MODE_RC_DIR)
    {
        liangan_rc_direct_task_5ms();
    }
    else if (g_liangan_sweep_mode == LIANGAN_MODE_RC_REC)
    {
        liangan_rc_direct_task_5ms();
        liangan_rc_record_task_5ms();
    }
    else if (g_liangan_sweep_mode == LIANGAN_MODE_RC_PLAY)
    {
        liangan_rc_replay_task_5ms();
    }
}

/* ================================================================
 *  RC 直驱 / 录制 / 回放（10ms 采样）
 *  CH1: 896±500 → ±3°
 * ================================================================ */
void liangan_rc_direct_task_5ms(void)
{
    /* 仅在 MODE3 且 funtion_ready_flag 时由上层调用，这里不检查 */
    float ch0 = (float)uart_receiver.channel[0];
    float angle = (ch0 - 896.0f) * 0.006f;  /* 3.0 / 500 = 0.006 */
    if (angle > 3.0f)  angle = 3.0f;
    if (angle < -3.0f) angle = -3.0f;
    pid_target = angle;
}

void liangan_rc_record_start(void)
{
    s_liangan_rc_rec_active = 1u;
    s_liangan_rc_play_active = 0u;
    s_liangan_rc_rec_tick   = 0u;
    s_liangan_rc_rec_count  = 0u;
    s_liangan_rc_rec_buf[s_liangan_rc_rec_count++] = pid_target;
}

void liangan_rc_record_stop(void)
{
    s_liangan_rc_rec_active = 0u;
}

void liangan_rc_record_task_5ms(void)
{
    if (!s_liangan_rc_rec_active) return;
    if (s_liangan_rc_rec_count >= (uint16_t)LIANGAN_RC_RECORD_MAX_POINTS)
    {
        s_liangan_rc_rec_active = 0u;
        return;
    }
    s_liangan_rc_rec_tick++;
    if (s_liangan_rc_rec_tick >= (uint16_t)LIANGAN_RC_SAMPLE_TICKS_5MS)
    {
        s_liangan_rc_rec_tick = 0u;
        s_liangan_rc_rec_buf[s_liangan_rc_rec_count++] = pid_target;
    }
}

void liangan_rc_replay_start(void)
{
    /* 如果 buffer 为空，尝试从 Flash slot 2 加载 */
    if (s_liangan_rc_rec_count == 0u)
    {
        liangan_rc_record_load_from_flash(2u);
    }
    if (s_liangan_rc_rec_count == 0u)
    {
        s_liangan_rc_play_active = 0u;
        return;
    }
    s_liangan_rc_rec_active  = 0u;
    s_liangan_rc_play_active = 1u;
    s_liangan_rc_play_tick   = 0u;
    s_liangan_rc_play_idx    = 0u;
    pid_target = s_liangan_rc_rec_buf[0u];
    s_liangan_rc_play_idx = 1u;
}

void liangan_rc_replay_stop(void)
{
    s_liangan_rc_play_active = 0u;
}

void liangan_rc_replay_task_5ms(void)
{
    if (!s_liangan_rc_play_active) return;
    if (s_liangan_rc_play_idx >= s_liangan_rc_rec_count)
    {
        s_liangan_rc_play_active = 0u;
        return;
    }
    s_liangan_rc_play_tick++;
    if (s_liangan_rc_play_tick >= (uint16_t)LIANGAN_RC_SAMPLE_TICKS_5MS)
    {
        s_liangan_rc_play_tick = 0u;
        pid_target = s_liangan_rc_rec_buf[s_liangan_rc_play_idx++];
    }
}

uint8_t liangan_rc_record_is_active(void)
{
    return s_liangan_rc_rec_active;
}

uint8_t liangan_rc_replay_is_active(void)
{
    return s_liangan_rc_play_active;
}

uint16_t liangan_rc_record_get_count(void)
{
    return s_liangan_rc_rec_count;
}

/* ================================================================
 *  Flash 存取（复用 RC Yaw 的 Flash 存储格式）
 * ================================================================ */

uint8_t liangan_rc_record_save_to_flash(uint8_t slot)
{
    return flash_storage_save_rc_yaw_record(
        (const float *)s_liangan_rc_rec_buf,
        s_liangan_rc_rec_count,
        slot);
}

uint8_t liangan_rc_record_load_from_flash(uint8_t slot)
{
    uint16_t count = 0u;
    if (!flash_storage_load_rc_yaw_record(
            (float *)s_liangan_rc_rec_buf,
            (uint16_t)LIANGAN_RC_RECORD_MAX_POINTS,
            &count,
            slot))
    {
        return 0u;
    }
    s_liangan_rc_rec_count = count;
    return 1u;
}
