/*
 * liangan.h  —— 摄像头 UART 数据接收
 *
 * 帧格式：AA XH XL FF（4字节定长）
 * XH|XL 合并为 0~800 的小球位置值
 */

#ifndef CODE_LIANGAN_H_
#define CODE_LIANGAN_H_

#include <stdint.h>

/* 小球位置（0~800），由 UART 中断更新，PID 直接读取 */
extern volatile uint16_t g_liangan_ball_pos;

/* 小球目标位置（0~800），菜单可调，默认 400 */
extern volatile uint16_t g_liangan_target;

/* 小球速度（×10，有符号），仅新格式 6 字节帧有 */
extern volatile int16_t  g_liangan_speed;

/* 小球速度（位置差/时间差，单位：位置值/秒），MCU 自算 */
extern volatile float    g_liangan_speed_calc;

/* 滤波后的小球位置 */
extern volatile float    g_liangan_ball_filt;

/* 滤波后的小球速度 */
extern volatile float    g_liangan_speed_filt;

/* 中心偏离量（-200~+200），叠加在默认中心 400 上，菜单可调 */
extern volatile int16_t  g_liangan_center_offset;

/* 自动扫目标偏移量：第一阶段目标 = center + sweep_ofs1，第二阶段 = center + sweep_ofs2 */
extern volatile int16_t  g_liangan_sweep_ofs1;
extern volatile int16_t  g_liangan_sweep_ofs2;

/* 自动扫目标总时长（ms），默认 4000，每 5ms 一步 */
extern volatile uint16_t g_liangan_sweep_time_ms;

/* 扫目标运行模式 */
typedef enum {
    LIANGAN_MODE_CLOSED = 0,  /* 位置闭环（公式扫目标） */
    LIANGAN_MODE_OPEN   = 1,  /* 段表开环（EX1 直接控制角度） */
    LIANGAN_MODE_RC_DIR = 2,  /* RC CH0 直驱角度（896±500 → ±3°） */
    LIANGAN_MODE_RC_REC = 3,  /* RC 直驱 + 录制到 buffer */
    LIANGAN_MODE_RC_PLAY= 4,  /* 从 buffer 回放 */
} liangan_mode_t;

extern volatile uint8_t  g_liangan_sweep_mode;  /* 默认 LIANGAN_MODE_CLOSED */

/* RC 轨迹录制/回放（10ms 采样，复用 RC Yaw Flash 存储） */
void liangan_rc_direct_task_5ms(void);
void liangan_rc_record_start(void);
void liangan_rc_record_stop(void);
void liangan_rc_record_task_5ms(void);
void liangan_rc_replay_start(void);
void liangan_rc_replay_stop(void);
void liangan_rc_replay_task_5ms(void);
uint8_t liangan_rc_record_is_active(void);
uint8_t liangan_rc_replay_is_active(void);
uint16_t liangan_rc_record_get_count(void);

/* Flash 存取（复用 RC Yaw 存储格式，slot 1~2） */
uint8_t liangan_rc_record_save_to_flash(uint8_t slot);
uint8_t liangan_rc_record_load_from_flash(uint8_t slot);

/* 滤波系数（0.80~0.99，菜单可调），默认 0.95 / 0.90 */
extern volatile float    g_liangan_alpha_pos;
extern volatile float    g_liangan_alpha_spd;

/* 新数据标志：ISR 置位，PID 消费后清零 */
extern volatile uint8_t  g_liangan_new_data;

/* ISR 触发计数：每进一次 UART3 RX 中断自加，调试用 */
extern volatile uint32_t g_liangan_isr_count;

/* UART RX 中断回调：放入 uart3_rx_isr 中调用 */
void liangan_uart_callback(void);

/* 初始化 UART3（115200, P15.6 RX），替代被注释的 gnss_init() */
void liangan_init(void);
/* 自动扫目标：MODE3 下每 5ms 调用，4s 内均匀扫 0↔800 */
void liangan_auto_sweep_task_5ms(void);
void liangan_auto_sweep_start(void);
void liangan_auto_sweep_reset(void);
extern volatile uint8_t g_liangan_sweep_active;
#endif /* CODE_LIANGAN_H_ */
