/*
 * common_flie.h
 *
 *  Created on: 2025-12-23
 *      Author: hhttiuui
 */

#ifndef CODE_COMMON_FLIE_H_
#define CODE_COMMON_FLIE_H_

#define PWM_CH1                 (ATOM0_CH1_P33_9)
#define SERVO_PERIOD_US         (3333u)    // 300Hz -> 3.33ms  -> 3333us
#define SERVO_PULSE_MIN_US      (1000u)
#define SERVO_PULSE_MAX_US      (2000u)

// pos: 0~1000  ->  1000~2000us
#define SERVO_PWM(pos)          ((uint32)(((uint64)PWM_DUTY_MAX * (SERVO_PULSE_MIN_US + (uint32)(pos))) / SERVO_PERIOD_US))

void imu_task_handler_2ms(void);

// ============================================================================
// 统一 PID 模式（common_flie.c 末尾新增）
// 说明：
// - 这是“新追加的可选接口”，用于统一不同 PID_INIT*/handler 的重复代码。
// - 旧接口全部保留；你可以逐步把外部调用切到该接口。
// ============================================================================

typedef enum
{
	PID_MODE_CASCADE = 0,     // 原：PID_INIT + pid_timer_handler_10ms（角度环->角速度环）
	PID_MODE_ANGLE = 1,       // 原：PID_INIT_NO_RATE + pid_timer_handler_10ms_no_rate
	PID_MODE_DAMPED_TWO = 2,  // 原：PID_INIT_NO_RATE_TWO + pid_timer_handler_10ms_no_rate_two
	PID_MODE_DAMPED_THREE = 3, // 原：PID_INIT_NO_RATE_THREE + pid_timer_handler_10ms_no_rate_three
	PID_MODE_DAMPED_FOUR = 4,
	PID_MODE_CASCADE_EX1 = 5, // 新增：CASCADE 参数组1（默认复制 PID_MODE_CASCADE）
	PID_MODE_CASCADE_EX2 = 6, // 新增：CASCADE 参数组2（默认复制 PID_MODE_CASCADE）
	PID_MODE_CASCADE_EX3 = 7,
	PID_MODE_CASCADE_EX4 = 8,
	PID_MODE_CASCADE_EX5 = 9,
	PID_MODE_CASCADE_EX6 = 10,
	PID_MODE_CASCADE_EX7 = 11,
	PID_MODE_CASCADE_EX8 = 12,
	PID_MODE_CASCADE_EX9 = 13,
	PID_MODE_CASCADE_EX10 = 14,
	PID_MODE_CASCADE_EX11 = 15,
} pid_mode_t;

// ============================================================================
// PID 模式配置表（对外暴露，便于屏幕/上层逻辑直接修改参数）
// 说明：
// - `s_pid_mode_cfg[mode]` 与 `pid_mode_t` 的枚举值一一对应（下标即 mode）。
// - 修改配置表后，需要触发一次初始化（例如调用 `ttop_funtion_mode_once_apply()`
//   间接调用 `PID_INIT_BY_MODE(mode)`），让参数拷贝到运行时 PID 实例中才会生效。
// - 该结构体定义位于头文件是为了让外部可直接访问字段；请谨慎修改字段布局。
// ============================================================================

typedef enum
{
	PID_STRAT_CASCADE = 0,
	PID_STRAT_ANGLE_ONLY,
	PID_STRAT_ANGLE_DAMPED,
} pid_strategy_t;

typedef struct
{
	float kp;
	float ki;
	float kd;
	float out_min;
	float out_max;
	float i_min;
	float i_max;
} pid_gains_cfg_t;

typedef struct
{
	pid_strategy_t strategy;

	// AHRS Mahony
	float ahrs_kp;
	float ahrs_ki;

	// yaw gate（pitch 角度门控）
	float yaw_gate_off_abs_pitch_deg; // |pitch| 大于该值 -> 关闭 yaw
	float yaw_gate_on_abs_pitch_deg;  // |pitch| 小于该值 -> 打开 yaw

	// PID 参数
	pid_gains_cfg_t yaw;
	pid_gains_cfg_t angle;
	pid_gains_cfg_t rate; // 仅 cascade 使用

	// 阻尼（仅 angle_damped 使用）
	float damp_k;
	float damp_max;

	// 其它初始化动作
	float imu_filter_alpha;
	float servo_init_out;
	int16 motor_target_counts_5ms;
} pid_mode_cfg_t;


// 配置表条目数（与 pid_mode_t 的最后一个枚举值保持一致）
#define PID_MODE_CFG_COUNT   (16u)

extern pid_mode_cfg_t s_pid_mode_cfg[PID_MODE_CFG_COUNT];

// 按模式一次性初始化 PID/AHRS/滤波/初值等
void PID_INIT_BY_MODE(pid_mode_t mode);

// 仅清理运行时积分状态（不修改 kp/ki/kd 与限幅配置）
// - 清零各 PID 实例的 integrator / prev_error / prev_measurement
// - 清零 AHRS Mahony 的积分反馈项
void PID_CLEAR_INTEGRATORS(void);

// 按模式热切换参数（不清零 PID 历史项）
// 说明：
// - 仅更新增益/限幅与电机目标速度，不执行 PID 实例重置。
// - 适合同一控制策略下（例如 CASCADE 组内）按速度切参。
void PID_HOT_SWITCH_BY_MODE(pid_mode_t mode);

// 动态 PID（cascade）参数接收接口：由外部传入实时 kp。
void pid_dynamic_cascade_set_kp(float angle_kp, float rate_kp, float yaw_kp);

// 动态 PID 专用处理：固定走 位置+角度+角速度（cascade）链路。
// 适用于外部按速度实时插值 kp 的场景。
void pid_timer_handler_dynamic_cascade(pid_mode_t mode);

// 多速率 PID 调度：外部按 2ms 调用
// - 角速度环：2ms
// - 角度环：4ms
// - yaw 外环：8ms
void pid_timer_handler_multirate_2ms(void);

// 舵机清零/回中：对外接口
// 说明：本项目舵机输出使用 PWM_CH1 + SERVO_PWM(pos) 映射。
// 这里的“清零”定义为输出回到 CENTER（与 common_flie.c 内部 servo_output_cb 的 CENTER=500 对齐）。
void servo_zero(void);


// 热更新 yaw 外环 kd（小球位置环 D 项），无需重新初始化
void pid_yaw_set_kd(float kd);

extern volatile float g_yaw_pid_out_deg;
extern volatile float g_yaw_target_deg;extern volatile float pid_target;             /* 角度环目标值（deg），开环扫目标直接写入 */extern volatile float g_yaw_zero_offset_deg;  // yaw 零点偏置（度），菜单可调
extern volatile float g_pitch_zero_offset_deg;
extern volatile float g_servo_zero_offset_deg; // pitch 零点偏置（度），菜单可调
extern volatile float g_pitch_angle_lpf_alpha; // pitch 角度低通系数（0=关闭滤波）
extern int count_pid;

#endif /* CODE_COMMON_FLIE_H_ */
