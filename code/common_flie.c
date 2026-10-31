/*
 * common_flie.c
 *
 *  Created on: 2025-12-23
 *      Author: hhttiuui
 *
 *  说明：
 *  - 本文件包含与舵机相关的通用函数示例，包含一个简单的 PID 控制实现。
 */


#include "zf_common_headfile.h"
#include "imu_filter.h"
#include "quaternion_ahrs.h"
#include "liangan.h"

extern volatile uint8_t g_liangan_sweep_mode;  /* liangan_mode_t */

int count_pid = 0;
volatile uint8 pid_flag = 0;

// yaw 零点偏置（度），后续可通过菜单修改并存入 Flash
volatile float g_yaw_zero_offset_deg = 0.0f;
// pitch 零点偏置（度），可在 SPCH_DBG 页面调节
volatile float g_pitch_zero_offset_deg = 0.0f;
volatile float g_servo_zero_offset_deg = 0.0f;
// pitch 角度低通系数（0=关闭滤波，取值建议 0~0.99）
volatile float g_pitch_angle_lpf_alpha = 0.0f;

// PID/AHRS 调用周期（秒）：当前用5ms触发 
#define PID_TASK_DT_S   (0.005f)

// 新增多速率调度函数使用的独立 dt（外部每 2ms 调一次）
#define PID_MR_RATE_DT_S    (0.002f)   // 角速度环：2ms
#define PID_MR_ANGLE_DT_S   (0.004f)   // 角度环：4ms
#define PID_MR_YAW_DT_S     (0.008f)   // yaw 外环：8ms

// 多速率调度：yaw 环开关（1=启用，0=关闭）
#ifndef PID_MR_ENABLE_YAW
#define PID_MR_ENABLE_YAW   (0u)
#endif

// IMU/AHRS 调用周期（秒）：当前用2ms触发 
#ifndef IMU_TASK_DT_S
#define IMU_TASK_DT_S   (0.001f)
#endif

volatile uint8 imu_flag_2ms = 0;

/*
 * 舵机 PID 控制模块（放在 common_flie.c）
 * 说明：
 * - 本模块提供一个简单的 PID 控制器实现，并提供一个 5 ms 定时处理入口函数。
 * - 测量值来源于 imu660rb，通过 imu660rb_acc_transition() 
 * - 函数的命名有点小问题，无伤大雅嘿嘿
 */

// PID 状态与参数
typedef struct {
	float kp;
	float ki;
	float kd;
	float integrator;
	float prev_error;
	float prev_measurement;
	float out_min;
	float out_max;
	float integrator_min;
	float integrator_max;
} pid_t;

// 串级 PID：外环（角度）-> 输出角速度目标；内环（角速度）-> 输出舵机控制量
static pid_t g_pid_angle;
static pid_t g_pid_rate;
static pid_t g_pid_yaw;
// 无角速度环（非线性/增益调度）版本的角度 PID
static pid_t g_pid_angle_two;
//目标值
volatile float pid_target = 0.0f;

// yaw 外环输出（deg），作为 pitch 角度环目标 pid_target
// 注意：赛事功能模块（Ttop_funtion）需要直接修改该值来实现“转弯规划”
volatile float g_yaw_pid_out_deg = 0.0f;
volatile float g_yaw_target_deg = 0.0f;
volatile float g_lg_angle_out = 0.0f;          /* LG_DBG: 角度环输出 */
volatile float g_lg_pos_err   = 0.0f;          /* LG_DBG: 小球位置误差 */

// 当前活动 PID 模式：由 PID_INIT_BY_MODE 设置，多速率调度函数按该模式执行。
static pid_mode_t g_pid_active_mode = PID_MODE_ANGLE;   /* MODE1: 小球位置+速度+舵机角 */

// Mahony 四元数姿态解算状态
static ahrs_mahony_t g_ahrs;

// ================= IMU 任务输出缓存（由 IMU 任务更新，PID 任务只读） =================
static volatile float g_roll_deg_cache = 0.0f;
static volatile float g_pitch_deg_cache = 0.0f;
static volatile float g_yaw_deg_cache = 0.0f;
static volatile float g_yaw_rel_deg_cache = 0.0f;
static volatile float g_pitch_rate_dps_f_cache = 0.0f;
static float g_pitch_deg_lpf_state = 0.0f;
static uint8 g_pitch_deg_lpf_inited = 0u;

// 角速度死区（单位：deg/s，用于抑制零点附近噪声，建议0-2）
#ifndef GYRO_RATE_DEADZONE_DPS
#define GYRO_RATE_DEADZONE_DPS   (0.0f)
#endif

// 舵机最终输出死区（单位：与 out 相同，用于抑制舵机零点附近抖动建议0-10）
#ifndef SERVO_OUT_DEADZONE
#define SERVO_OUT_DEADZONE   (2.0f)
#endif

// yaw 门控：当 |pitch| 超过该值时关闭 yaw 环（deg）
#ifndef YAW_GATE_ABS_PITCH_MAX_DEG
#define YAW_GATE_ABS_PITCH_MAX_DEG   (30.0f)
#endif

// 死区函数（带重映射）：|x|<=dz 输出0；超过死区后减去dz，避免跨越边界跳变。
static inline float apply_deadzone(float x, float dz)
{
	if (dz <= 0.0f) return x;
	if (x > dz) return x - dz;
	if (x < -dz) return x + dz;
	return 0.0f;
}

static void servo_output_cb(float out)
{
	// 假设 out 范围已通过 pid_set_limits() 限幅到 e.g. -400..+400
	// 中心值（舵机空位）：
    const int CENTER = 500;
	float adj = out + g_servo_zero_offset_deg;
	int i = (int)roundf((float)CENTER - adj);
    pwm_set_duty(PWM_CH1, SERVO_PWM(i));
}

// 对外：舵机清零/回中
// 与 servo_output_cb 的 CENTER 定义保持一致：CENTER=500
void servo_zero(void)
{
	const int CENTER = 500;
	pwm_set_duty(PWM_CH1, SERVO_PWM(CENTER));
}

// 默认 PID 参数
static void pid_init_default(pid_t *pid)
{
	pid->kp = 1.0f;
	pid->ki = 0.0f;
	pid->kd = 0.0f;
	pid->integrator = 0.0f;
	pid->prev_error = 0.0f;
	pid->prev_measurement = 0.0f;
	pid->out_min = -1000.0f; // 根据实际需要调整最小输出
	pid->out_max = 1000.0f;  // 根据实际需要调整最大输出
	// 默认积分限幅：与输出限幅一致（可通过 pid_set_integrator_limits 调整）
	pid->integrator_min = pid->out_min;
	pid->integrator_max = pid->out_max;
}

// 积分项限幅
static void pid_set_integrator_limits(pid_t *pid, float imin, float imax)
{
	pid->integrator_min = imin;
	pid->integrator_max = imax;
}

static void pid_set_gains(pid_t *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

static void pid_set_limits(pid_t *pid, float out_min, float out_max)
{
	pid->out_min = out_min;
	pid->out_max = out_max;
}

// PID 计算，输入测量值 measurement，时间间隔 dt（秒）
static float pid_compute(pid_t *pid, float setpoint, float measurement, float dt)
{
	float error = setpoint - measurement;

	// P 项
	float p = pid->kp * error;

	// I 项（梯形积分）
	pid->integrator += 0.5f * pid->ki * dt * (error + pid->prev_error);

	// 防积分饱和：限制到单独的积分限幅（anti-windup）
	if (pid->integrator > pid->integrator_max) pid->integrator = pid->integrator_max;
	if (pid->integrator < pid->integrator_min) pid->integrator = pid->integrator_min;

	// D 项（基于测量，减少噪声敏感度）
	float d = 0.0f;
	if (dt > 0.0f)
	{
		d = - pid->kd * (measurement - pid->prev_measurement) / dt; // '-' because derivative on measurement
	}

	float out = p + pid->integrator + d;

	// 输出限幅
	if (out > pid->out_max) out = pid->out_max;
	if (out < pid->out_min) out = pid->out_min;

	// 保存历史量
	pid->prev_error = error;
	pid->prev_measurement = measurement;

	return out;
}

// =======IMU 周期任务处理函数：在 2ms 定时器中调用=======
void imu_task_handler_2ms(void)
{
	gpio_toggle_level(P22_1);
	// 读取 IMU 数据
	imu660rx_get_acc();
	imu660rx_get_gyro();

	// --- 四元数姿态解算（Mahony，6轴）---
	float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
	ahrs_mahony_update_imu660rx_euler_deg(&g_ahrs, IMU_TASK_DT_S, &roll_deg, &pitch_deg, &yaw_deg);
	//yaw 为 6轴不可观量，会随时间漂移；工程上常用“相对yaw”（启动稳定后自动置零）
	float yaw_rel_deg = ahrs_yaw_get_rel_deg(yaw_deg);
	yaw_rel_deg = ahrs_yaw_rel_unwrap_720_update_deg(yaw_rel_deg);

	// 角速度（deg/s）：按当前安装方向，暂时沿用 gyro_y，并保留你现有的符号约定
	float pitch_rate_dps_f = -imu660rx_gyro_transition_filtered(imu660rx_gyro_y);
	pitch_rate_dps_f = apply_deadzone(pitch_rate_dps_f, GYRO_RATE_DEADZONE_DPS);
	float pitch_corr_deg = pitch_deg - g_pitch_zero_offset_deg;
	float pitch_out_deg = pitch_corr_deg;

	if (g_pitch_angle_lpf_alpha > 0.0f)
	{
		float a = g_pitch_angle_lpf_alpha;
		if (a > 0.99f) a = 0.99f;
		if (!g_pitch_deg_lpf_inited)
		{
			g_pitch_deg_lpf_state = pitch_corr_deg;
			g_pitch_deg_lpf_inited = 1u;
		}
		g_pitch_deg_lpf_state = a * g_pitch_deg_lpf_state + (1.0f - a) * pitch_corr_deg;
		pitch_out_deg = g_pitch_deg_lpf_state;
	}
	else
	{
		g_pitch_deg_lpf_state = pitch_corr_deg;
		g_pitch_deg_lpf_inited = 0u;
	}

	// 写入缓存（PID 任务仅读取这些缓存）
	g_roll_deg_cache = roll_deg;
	g_pitch_deg_cache = pitch_out_deg;
	g_yaw_deg_cache = yaw_deg;
	g_yaw_rel_deg_cache = yaw_rel_deg;
	g_pitch_rate_dps_f_cache = pitch_rate_dps_f;

	// 兼容你现有的调试显示变量（如果 UI 侧在用）
	show_roll_deg = roll_deg;
	show_pitch_deg = pitch_out_deg;
	show_yaw_deg = yaw_rel_deg;
	show_pitch_rate_dps_f = pitch_rate_dps_f;
	gpio_toggle_level(P22_1);
}

// ============================================================================
//
// 统一 PID 模式：配置表 + 通用 init + 通用 handler
//
// ============================================================================

// 配置表与结构体类型在 common_flie.h 中对外暴露，便于屏幕/上层逻辑直接修改。

pid_mode_cfg_t s_pid_mode_cfg[PID_MODE_CFG_COUNT] =
{
	// PID_MODE_CASCADE
	{
		.strategy = PID_STRAT_CASCADE,
		.ahrs_kp = 4.5f,
		.ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 6.0f,
		.yaw_gate_on_abs_pitch_deg = 4.0f,
		.yaw = { .kp = 0.7f, .ki = 0.0f, .kd = 0.0012f, .out_min = -20.0f, .out_max = 20.0f, .i_min = -10.0f, .i_max = 10.0f },
		.angle = { .kp = 18.0f, .ki = 0.0f, .kd = 0.0008f, .out_min = -700.0f, .out_max = 700.0f, .i_min = -150.0f, .i_max = 150.0f },
		.rate = { .kp = 1.8f, .ki = 0.0f, .kd = 0.0035f, .out_min = -600.0f, .out_max = 600.0f, .i_min = -100.0f, .i_max = 100.0f },
		.damp_k = 0.0f,
		.damp_max = 0.0f,
		.imu_filter_alpha = 0.92f,
		.servo_init_out = 0.0f,
		.motor_target_counts_5ms = 200,
	},

	// PID_MODE_ANGLE
	{
		.strategy = PID_STRAT_ANGLE_ONLY,
		.ahrs_kp = 4.5f,
		.ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 6.0f,
		.yaw_gate_on_abs_pitch_deg = 4.0f,
		.yaw = { .kp = 0.25f, .ki = 0.0f, .kd = 0.0012f, .out_min = -5.0f, .out_max = 5.0f, .i_min = -10.0f, .i_max = 10.0f },
		.angle = { .kp = 90.0f, .ki = 0.0f, .kd = 0.0028f, .out_min = -600.0f, .out_max = 600.0f, .i_min = -150.0f, .i_max = 150.0f },
		.rate = { 0 },
		.damp_k = 0.0f,
		.damp_max = 0.0f,
		.imu_filter_alpha = 0.92f,
		.servo_init_out = 0.0f,
		.motor_target_counts_5ms = 100,
	},

	// PID_MODE_DAMPED_TWO
	{
		.strategy = PID_STRAT_ANGLE_DAMPED,
		.ahrs_kp = 1.0f,
		.ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 1.5f,
		.yaw_gate_on_abs_pitch_deg = 0.5f,
		.yaw = { .kp = 0.25f, .ki = 0.0f, .kd = 0.0002f, .out_min = -3.5f, .out_max = 3.5f, .i_min = -5.0f, .i_max = 5.0f },
		.angle = { .kp = 195.0f, .ki = 0.0f, .kd = 0.0028f, .out_min = -700.0f, .out_max = 700.0f, .i_min = -550.0f, .i_max = 550.0f },
		.rate = { 0 },
		.damp_k = 4.5f,
		.damp_max = 120.0f,
		.imu_filter_alpha = 0.92f,
		.servo_init_out = 30.0f,
		.motor_target_counts_5ms = 55,
	},

	// PID_MODE_DAMPED_THREE
	{
		.strategy = PID_STRAT_ANGLE_DAMPED,
		.ahrs_kp = 0.35f,
		.ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 1.5f,
		.yaw_gate_on_abs_pitch_deg = 0.5f,
		.yaw = { .kp = 0.25f, .ki = 0.0f, .kd = 0.0002f, .out_min = -5.5f, .out_max = 5.5f, .i_min = -1.0f, .i_max = 1.0f },
		.angle = { .kp = 235.0f, .ki = 0.0f, .kd = 0.38f, .out_min = -900.0f, .out_max = 900.0f, .i_min = -550.0f, .i_max = 550.0f },
		.rate = { 0 },
		.damp_k = 12.0f,
		.damp_max = 180.0f,
		.imu_filter_alpha = 0.92f,
		.servo_init_out = 30.0f,
		.motor_target_counts_5ms = 45,
	},

	// PID_MODE_DAMPED_FOUR
	{
		.strategy = PID_STRAT_ANGLE_DAMPED,
		.ahrs_kp = 0.35f,
		.ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 1.5f,
		.yaw_gate_on_abs_pitch_deg = 0.5f,
		.yaw = { .kp = 0.25f, .ki = 0.0f, .kd = 0.0002f, .out_min = -5.5f, .out_max = 5.5f, .i_min = -1.0f, .i_max = 1.0f },
		.angle = { .kp = 235.0f, .ki = 0.0f, .kd = 0.38f, .out_min = -900.0f, .out_max = 900.0f, .i_min = -550.0f, .i_max = 550.0f },
		.rate = { 0 },
		.damp_k = 12.0f,
		.damp_max = 180.0f,
		.imu_filter_alpha = 0.92f,
		.servo_init_out = 30.0f,
		.motor_target_counts_5ms = 45,
	},

	// PID_MODE_CASCADE_EX1（默认复制 PID_MODE_CASCADE）
	{
		.strategy = PID_STRAT_CASCADE,
		.ahrs_kp = 4.5f,
		.ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 6.0f,
		.yaw_gate_on_abs_pitch_deg = 4.0f,
		.yaw = { .kp = 0.7f, .ki = 0.0f, .kd = 0.0012f, .out_min = -20.0f, .out_max = 20.0f, .i_min = -10.0f, .i_max = 10.0f },
		.angle = { .kp = 18.0f, .ki = 0.0f, .kd = 0.0008f, .out_min = -700.0f, .out_max = 700.0f, .i_min = -150.0f, .i_max = 150.0f },
		.rate = { .kp = 1.8f, .ki = 0.0f, .kd = 0.0035f, .out_min = -600.0f, .out_max = 600.0f, .i_min = -100.0f, .i_max = 100.0f },
		.damp_k = 0.0f,
		.damp_max = 0.0f,
		.imu_filter_alpha = 0.92f,
		.servo_init_out = 0.0f,
		.motor_target_counts_5ms = 200,
	},

	// PID_MODE_CASCADE_EX2（默认复制 PID_MODE_CASCADE）
	{
		.strategy = PID_STRAT_CASCADE,
		.ahrs_kp = 4.5f,
		.ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 6.0f,
		.yaw_gate_on_abs_pitch_deg = 4.0f,
		.yaw = { .kp = 0.7f, .ki = 0.0f, .kd = 0.0012f, .out_min = -20.0f, .out_max = 20.0f, .i_min = -10.0f, .i_max = 10.0f },
		.angle = { .kp = 18.0f, .ki = 0.0f, .kd = 0.0008f, .out_min = -700.0f, .out_max = 700.0f, .i_min = -150.0f, .i_max = 150.0f },
		.rate = { .kp = 1.8f, .ki = 0.0f, .kd = 0.0035f, .out_min = -600.0f, .out_max = 600.0f, .i_min = -100.0f, .i_max = 100.0f },
		.damp_k = 0.0f,
		.damp_max = 0.0f,
		.imu_filter_alpha = 0.92f,
		.servo_init_out = 0.0f,
		.motor_target_counts_5ms = 200,
	},

	// PID_MODE_CASCADE_EX3（默认复制 PID_MODE_CASCADE）
	{
		.strategy = PID_STRAT_CASCADE,
		.ahrs_kp = 4.5f,
		.ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 6.0f,
		.yaw_gate_on_abs_pitch_deg = 4.0f,
		.yaw = { .kp = 0.7f, .ki = 0.0f, .kd = 0.0012f, .out_min = -20.0f, .out_max = 20.0f, .i_min = -10.0f, .i_max = 10.0f },
		.angle = { .kp = 18.0f, .ki = 0.0f, .kd = 0.0008f, .out_min = -700.0f, .out_max = 700.0f, .i_min = -150.0f, .i_max = 150.0f },
		.rate = { .kp = 1.8f, .ki = 0.0f, .kd = 0.0035f, .out_min = -600.0f, .out_max = 600.0f, .i_min = -100.0f, .i_max = 100.0f },
		.damp_k = 0.0f,
		.damp_max = 0.0f,
		.imu_filter_alpha = 0.92f,
		.servo_init_out = 0.0f,
		.motor_target_counts_5ms = 200,
	},

	// PID_MODE_CASCADE_EX4（默认复制 PID_MODE_CASCADE）
	{
		.strategy = PID_STRAT_CASCADE,
		.ahrs_kp = 4.5f,
		.ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 6.0f,
		.yaw_gate_on_abs_pitch_deg = 4.0f,
		.yaw = { .kp = 0.7f, .ki = 0.0f, .kd = 0.0012f, .out_min = -20.0f, .out_max = 20.0f, .i_min = -10.0f, .i_max = 10.0f },
		.angle = { .kp = 18.0f, .ki = 0.0f, .kd = 0.0008f, .out_min = -700.0f, .out_max = 700.0f, .i_min = -150.0f, .i_max = 150.0f },
		.rate = { .kp = 1.8f, .ki = 0.0f, .kd = 0.0035f, .out_min = -600.0f, .out_max = 600.0f, .i_min = -100.0f, .i_max = 100.0f },
		.damp_k = 0.0f,
		.damp_max = 0.0f,
		.imu_filter_alpha = 0.92f,
		.servo_init_out = 0.0f,
		.motor_target_counts_5ms = 200,
	},

	// PID_MODE_CASCADE_EX5（默认复制 PID_MODE_CASCADE）
	{
		.strategy = PID_STRAT_CASCADE,
		.ahrs_kp = 4.5f,
		.ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 6.0f,
		.yaw_gate_on_abs_pitch_deg = 4.0f,
		.yaw = { .kp = 0.7f, .ki = 0.0f, .kd = 0.0012f, .out_min = -20.0f, .out_max = 20.0f, .i_min = -10.0f, .i_max = 10.0f },
		.angle = { .kp = 18.0f, .ki = 0.0f, .kd = 0.0008f, .out_min = -700.0f, .out_max = 700.0f, .i_min = -150.0f, .i_max = 150.0f },
		.rate = { .kp = 1.8f, .ki = 0.0f, .kd = 0.0035f, .out_min = -600.0f, .out_max = 600.0f, .i_min = -100.0f, .i_max = 100.0f },
		.damp_k = 0.0f,
		.damp_max = 0.0f,
		.imu_filter_alpha = 0.92f,
		.servo_init_out = 0.0f,
		.motor_target_counts_5ms = 200,
	},

	// PID_MODE_CASCADE_EX6（默认复制 PID_MODE_CASCADE）
	{
		.strategy = PID_STRAT_CASCADE,
		.ahrs_kp = 4.5f,
		.ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 6.0f,
		.yaw_gate_on_abs_pitch_deg = 4.0f,
		.yaw = { .kp = 0.7f, .ki = 0.0f, .kd = 0.0012f, .out_min = -20.0f, .out_max = 20.0f, .i_min = -10.0f, .i_max = 10.0f },
		.angle = { .kp = 18.0f, .ki = 0.0f, .kd = 0.0008f, .out_min = -700.0f, .out_max = 700.0f, .i_min = -150.0f, .i_max = 150.0f },
		.rate = { .kp = 1.8f, .ki = 0.0f, .kd = 0.0035f, .out_min = -600.0f, .out_max = 600.0f, .i_min = -100.0f, .i_max = 100.0f },
		.damp_k = 0.0f,
		.damp_max = 0.0f,
		.imu_filter_alpha = 0.92f,
		.servo_init_out = 0.0f,
		.motor_target_counts_5ms = 200,
	},

	// PID_MODE_CASCADE_EX7（默认复制 PID_MODE_CASCADE）
	{
		.strategy = PID_STRAT_CASCADE,
		.ahrs_kp = 4.5f,
		.ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 6.0f,
		.yaw_gate_on_abs_pitch_deg = 4.0f,
		.yaw = { .kp = 0.7f, .ki = 0.0f, .kd = 0.0012f, .out_min = -20.0f, .out_max = 20.0f, .i_min = -10.0f, .i_max = 10.0f },
		.angle = { .kp = 18.0f, .ki = 0.0f, .kd = 0.0008f, .out_min = -700.0f, .out_max = 700.0f, .i_min = -150.0f, .i_max = 150.0f },
		.rate = { .kp = 1.8f, .ki = 0.0f, .kd = 0.0035f, .out_min = -600.0f, .out_max = 600.0f, .i_min = -100.0f, .i_max = 100.0f },
		.damp_k = 0.0f,
		.damp_max = 0.0f,
		.imu_filter_alpha = 0.92f,
		.servo_init_out = 0.0f,
		.motor_target_counts_5ms = 200,
	},

	// PID_MODE_CASCADE_EX8
	{
		.strategy = PID_STRAT_CASCADE,
		.ahrs_kp = 4.5f, .ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 6.0f, .yaw_gate_on_abs_pitch_deg = 4.0f,
		.yaw = { .kp = 0.7f, .ki = 0.0f, .kd = 0.0012f, .out_min = -20.0f, .out_max = 20.0f, .i_min = -10.0f, .i_max = 10.0f },
		.angle = { .kp = 18.0f, .ki = 0.0f, .kd = 0.0008f, .out_min = -700.0f, .out_max = 700.0f, .i_min = -150.0f, .i_max = 150.0f },
		.rate = { .kp = 1.8f, .ki = 0.0f, .kd = 0.0035f, .out_min = -600.0f, .out_max = 600.0f, .i_min = -100.0f, .i_max = 100.0f },
		.damp_k = 0.0f, .damp_max = 0.0f,
		.imu_filter_alpha = 0.92f, .servo_init_out = 0.0f,
		.motor_target_counts_5ms = 200,
	},

	// PID_MODE_CASCADE_EX9
	{
		.strategy = PID_STRAT_CASCADE,
		.ahrs_kp = 4.5f, .ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 6.0f, .yaw_gate_on_abs_pitch_deg = 4.0f,
		.yaw = { .kp = 0.7f, .ki = 0.0f, .kd = 0.0012f, .out_min = -20.0f, .out_max = 20.0f, .i_min = -10.0f, .i_max = 10.0f },
		.angle = { .kp = 18.0f, .ki = 0.0f, .kd = 0.0008f, .out_min = -700.0f, .out_max = 700.0f, .i_min = -150.0f, .i_max = 150.0f },
		.rate = { .kp = 1.8f, .ki = 0.0f, .kd = 0.0035f, .out_min = -600.0f, .out_max = 600.0f, .i_min = -100.0f, .i_max = 100.0f },
		.damp_k = 0.0f, .damp_max = 0.0f,
		.imu_filter_alpha = 0.92f, .servo_init_out = 0.0f,
		.motor_target_counts_5ms = 200,
	},

	// PID_MODE_CASCADE_EX10
	{
		.strategy = PID_STRAT_CASCADE,
		.ahrs_kp = 4.5f, .ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 6.0f, .yaw_gate_on_abs_pitch_deg = 4.0f,
		.yaw = { .kp = 0.7f, .ki = 0.0f, .kd = 0.0012f, .out_min = -20.0f, .out_max = 20.0f, .i_min = -10.0f, .i_max = 10.0f },
		.angle = { .kp = 18.0f, .ki = 0.0f, .kd = 0.0008f, .out_min = -700.0f, .out_max = 700.0f, .i_min = -150.0f, .i_max = 150.0f },
		.rate = { .kp = 1.8f, .ki = 0.0f, .kd = 0.0035f, .out_min = -600.0f, .out_max = 600.0f, .i_min = -100.0f, .i_max = 100.0f },
		.damp_k = 0.0f, .damp_max = 0.0f,
		.imu_filter_alpha = 0.92f, .servo_init_out = 0.0f,
		.motor_target_counts_5ms = 200,
	},

	// PID_MODE_CASCADE_EX11
	{
		.strategy = PID_STRAT_CASCADE,
		.ahrs_kp = 4.5f, .ahrs_ki = 0.0f,
		.yaw_gate_off_abs_pitch_deg = 6.0f, .yaw_gate_on_abs_pitch_deg = 4.0f,
		.yaw = { .kp = 0.7f, .ki = 0.0f, .kd = 0.0012f, .out_min = -20.0f, .out_max = 20.0f, .i_min = -10.0f, .i_max = 10.0f },
		.angle = { .kp = 18.0f, .ki = 0.0f, .kd = 0.0008f, .out_min = -700.0f, .out_max = 700.0f, .i_min = -150.0f, .i_max = 150.0f },
		.rate = { .kp = 1.8f, .ki = 0.0f, .kd = 0.0035f, .out_min = -600.0f, .out_max = 600.0f, .i_min = -100.0f, .i_max = 100.0f },
		.damp_k = 0.0f, .damp_max = 0.0f,
		.imu_filter_alpha = 0.92f, .servo_init_out = 0.0f,
		.motor_target_counts_5ms = 200,
	},

};

static inline const pid_mode_cfg_t *pid_mode_cfg_get(pid_mode_t mode)
{
	uint16 m = (uint16)mode;
	uint16 n = (uint16)(sizeof(s_pid_mode_cfg) / sizeof(s_pid_mode_cfg[0]));
	if (m >= n) m = 0u;
	return &s_pid_mode_cfg[m];
}

void pid_dynamic_cascade_set_kp(float angle_kp, float rate_kp, float yaw_kp)
{
	pid_set_gains(&g_pid_angle, angle_kp, g_pid_angle.ki, g_pid_angle.kd);
	pid_set_gains(&g_pid_rate, rate_kp, g_pid_rate.ki, g_pid_rate.kd);
	pid_set_gains(&g_pid_yaw, yaw_kp / 100.0f, g_pid_yaw.ki, g_pid_yaw.kd);
}

void pid_yaw_set_kd(float kd)
{
	g_pid_yaw.kd = kd;
}

static void pid_apply_gains(pid_t *pid, const pid_gains_cfg_t *cfg)
{
	pid_init_default(pid);
	pid_set_gains(pid, cfg->kp, cfg->ki, cfg->kd);
	pid_set_limits(pid, cfg->out_min, cfg->out_max);
	pid_set_integrator_limits(pid, cfg->i_min, cfg->i_max);
}

static void pid_apply_gains_hot(pid_t *pid, const pid_gains_cfg_t *cfg)
{
	pid_set_gains(pid, cfg->kp, cfg->ki, cfg->kd);
	pid_set_limits(pid, cfg->out_min, cfg->out_max);
	pid_set_integrator_limits(pid, cfg->i_min, cfg->i_max);

	// 热切换时保留历史项，但把积分值夹到新的积分限幅内。
	if (pid->integrator > pid->integrator_max) pid->integrator = pid->integrator_max;
	if (pid->integrator < pid->integrator_min) pid->integrator = pid->integrator_min;
}

static void pid_clear_integrator_state(pid_t *pid)
{
	pid->integrator = 0.0f;
	pid->prev_error = 0.0f;
	pid->prev_measurement = 0.0f;
}

void PID_CLEAR_INTEGRATORS(void)
{
	pid_clear_integrator_state(&g_pid_yaw);
	pid_clear_integrator_state(&g_pid_angle);
	pid_clear_integrator_state(&g_pid_rate);
	pid_clear_integrator_state(&g_pid_angle_two);

	// 仅重置 Mahony 积分反馈项，姿态四元数保持不变。
	g_ahrs.integralFBx = 0.0f;
	g_ahrs.integralFBy = 0.0f;
	g_ahrs.integralFBz = 0.0f;
}

void PID_INIT_BY_MODE(pid_mode_t mode)
{
	uint16 mode_idx = (uint16)mode;
	if (mode_idx >= (uint16)PID_MODE_CFG_COUNT)
	{
		mode_idx = 0u;
	}
	g_pid_active_mode = (pid_mode_t)mode_idx;

	const pid_mode_cfg_t *cfg = pid_mode_cfg_get(g_pid_active_mode);

	// 姿态解算初始化
	ahrs_mahony_init(&g_ahrs, cfg->ahrs_kp, cfg->ahrs_ki);

	// yaw PID：所有模式都会用（kp/ki ÷100 适配摄像头 0~800 数据域）
	{
		pid_gains_cfg_t yaw_scaled = cfg->yaw;
		yaw_scaled.kp /= 100.0f;
		yaw_scaled.ki /= 100.0f;
		pid_apply_gains(&g_pid_yaw, &yaw_scaled);
	}

	// 角度 PID：根据模式决定用哪个实例
	if (cfg->strategy == PID_STRAT_CASCADE || cfg->strategy == PID_STRAT_ANGLE_ONLY)
	{
		pid_apply_gains(&g_pid_angle, &cfg->angle);
	}
	else
	{
		pid_apply_gains(&g_pid_angle_two, &cfg->angle);
	}

	// 角速度 PID：仅 cascade
	if (cfg->strategy == PID_STRAT_CASCADE)
	{
		pid_apply_gains(&g_pid_rate, &cfg->rate);
	}

	// 阻尼参数：完全由 s_pid_mode_cfg 决定（运行时不再被 NO_RATE_* 覆盖）

	imu_filter_init(cfg->imu_filter_alpha);

	// 初始化输出/目标
	g_yaw_pid_out_deg = 0.0f;
	pid_target = 0.0f;
	servo_output_cb(cfg->servo_init_out);
	motor_set_target_counts_per_5ms(cfg->motor_target_counts_5ms);
}

void PID_HOT_SWITCH_BY_MODE(pid_mode_t mode)
{
	const pid_mode_cfg_t *cfg = pid_mode_cfg_get(mode);

	// AHRS 增益热切换：不重置四元数与历史积分，避免姿态估计跳变。
	ahrs_mahony_set_gains(&g_ahrs, cfg->ahrs_kp, cfg->ahrs_ki, 0u);

	// yaw PID：所有模式都会用
	{
		pid_gains_cfg_t yaw_scaled = cfg->yaw;
		yaw_scaled.kp /= 100.0f;
		yaw_scaled.ki /= 100.0f;
		pid_apply_gains_hot(&g_pid_yaw, &yaw_scaled);
	}

	// 角度 PID：根据策略更新当前使用的实例
	if (cfg->strategy == PID_STRAT_CASCADE || cfg->strategy == PID_STRAT_ANGLE_ONLY)
	{
		pid_apply_gains_hot(&g_pid_angle, &cfg->angle);
	}
	else
	{
		pid_apply_gains_hot(&g_pid_angle_two, &cfg->angle);
	}

	// 角速度 PID：仅 cascade 使用
	if (cfg->strategy == PID_STRAT_CASCADE)
	{
		pid_apply_gains_hot(&g_pid_rate, &cfg->rate);
	}

	// 与速度分档相关：热切换时同步更新目标速度。
	//motor_set_target_counts_per_5ms(cfg->motor_target_counts_5ms);
}

static float pid_compute_yaw_target_with_gate(const pid_mode_cfg_t *cfg, float pitch_deg, float yaw_rel_deg)
{
	static uint8 s_yaw_enable = 1u;
	float abs_pitch = fabsf(pitch_deg);

	if (s_yaw_enable)
	{
		if (abs_pitch > cfg->yaw_gate_off_abs_pitch_deg) s_yaw_enable = 0u;
	}
	else
	{
		if (abs_pitch < cfg->yaw_gate_on_abs_pitch_deg) s_yaw_enable = 1u;
	}

	// 门控关闭时：保持当前目标（等效不更新 yaw 外环输出）；这里保持原行为：仍计算 yaw pid
	(void)s_yaw_enable;
	return pid_compute(&g_pid_yaw, g_yaw_target_deg, yaw_rel_deg, PID_TASK_DT_S);
}

static float pid_compute_yaw_target_with_gate_dt(const pid_mode_cfg_t *cfg, float pitch_deg, float yaw_rel_deg, float yaw_dt_s)
{
	static uint8 s_yaw_enable_dt = 1u;
	float abs_pitch = fabsf(pitch_deg);

	if (s_yaw_enable_dt)
	{
		if (abs_pitch > cfg->yaw_gate_off_abs_pitch_deg) s_yaw_enable_dt = 0u;
	}
	else
	{
		if (abs_pitch < cfg->yaw_gate_on_abs_pitch_deg) s_yaw_enable_dt = 1u;
	}

	(void)s_yaw_enable_dt;
	return pid_compute(&g_pid_yaw, g_yaw_target_deg, yaw_rel_deg, yaw_dt_s);
}

void pid_timer_handler_dynamic_cascade(pid_mode_t mode)
{
	// 动态 PID 专用：固定 cascade 控制链，kp 可由外部实时插值更新。
	const pid_mode_cfg_t *cfg = pid_mode_cfg_get(mode);

	float pitch_deg = (float)g_pitch_deg_cache;
	float pitch_rate_dps_f = (float)g_pitch_rate_dps_f_cache;
	float yaw_rel_deg = (float)g_yaw_rel_deg_cache;

	// yaw PID -> pid_target
	g_yaw_pid_out_deg = pid_compute_yaw_target_with_gate(cfg, pitch_deg, yaw_rel_deg);
	pid_target = g_yaw_pid_out_deg;

	float rate_sp_dps = pid_compute(&g_pid_angle, pid_target, pitch_deg, PID_TASK_DT_S);
	float out = pid_compute(&g_pid_rate, rate_sp_dps, pitch_rate_dps_f, PID_TASK_DT_S);
	out = apply_deadzone(out, SERVO_OUT_DEADZONE);

	ips_show_pid_out = out;
	servo_output_cb(out);
	count_pid++;
}

void pid_timer_handler_multirate_2ms(void)
{
	gpio_toggle_level(P22_2);
	// 新框架：外部固定 2ms 调用本函数
	// - 角速度环：每次调用执行（2ms）
	// - 角度环：每 2 次执行（4ms）
	// - yaw 外环：每 4 次执行（8ms）
	const pid_mode_cfg_t *cfg = pid_mode_cfg_get(g_pid_active_mode);
	static uint8 s_sched_cnt = 0u;
	static float s_yaw_target_hold = 0.0f;
	static float s_angle_out_hold = 0.0f;
	static float s_rate_sp_hold = 0.0f;

	float pitch_deg = (float)g_pitch_deg_cache;
	float pitch_rate_dps_f = (float)g_pitch_rate_dps_f_cache;
	float yaw_rel_deg = (float)g_yaw_rel_deg_cache - g_yaw_zero_offset_deg;
	uint8 run_angle = 0u;
	uint8 run_yaw = 0u;

	if ((s_sched_cnt & 0x01u) == 0u) run_angle = 1u;
	if ((s_sched_cnt & 0x03u) == 0u) run_yaw = 1u;

	/* [liangan] Yaw 环：仅闭环模式下使用摄像头位置反馈 */
	if (g_liangan_sweep_mode == 0u)   /* LIANGAN_MODE_CLOSED */
	{
		if (run_yaw)
		{
			s_yaw_target_hold = pid_compute(&g_pid_yaw,
				(float)g_liangan_target,        // 目标：小球位置（菜单可调）
				g_liangan_ball_filt,            // 反馈：滤波后位置
				PID_MR_YAW_DT_S);
		}

		g_yaw_pid_out_deg = s_yaw_target_hold;
		pid_target = s_yaw_target_hold;
	}
	/* 其他模式（开环/RC直驱/录制/回放）由 liangan 模块写入 pid_target */

	if (cfg->strategy == PID_STRAT_CASCADE)
	{
		if (run_angle)
		{
			s_rate_sp_hold = pid_compute(&g_pid_angle, pid_target, pitch_deg, PID_MR_ANGLE_DT_S);
		}

		float out = pid_compute(&g_pid_rate, s_rate_sp_hold, pitch_rate_dps_f, PID_MR_RATE_DT_S);
		out = apply_deadzone(out, SERVO_OUT_DEADZONE);
		ips_show_pid_out = out;
		servo_output_cb(out);
	}
	else if (cfg->strategy == PID_STRAT_ANGLE_ONLY)
	{
		/* MODE1: 小球位置(滤波)→俯仰目标 + 小球速度→前馈 + 舵机角度闭环 */
		if (run_angle)
		{
			float ball_err = (float)g_liangan_target - g_liangan_ball_filt;
			float angle_sp = ball_err * 0.02f;                  /* 位置→角度目标 */
			g_lg_pos_err = ball_err;
			s_angle_out_hold = pid_compute(&g_pid_angle, angle_sp, pitch_deg, PID_MR_ANGLE_DT_S);
			g_lg_angle_out = s_angle_out_hold;
		}

		float out = s_angle_out_hold
		          - g_liangan_speed_calc * 0.01f;               /* 速度阻尼（反方向） */
		out = apply_deadzone(out, SERVO_OUT_DEADZONE);
		ips_show_pid_out = out;
		servo_output_cb(out);
	}
	else
	{
		if (run_angle)
		{
			s_angle_out_hold = pid_compute(&g_pid_angle_two, pid_target, pitch_deg, PID_MR_ANGLE_DT_S);
		}

		float out = s_angle_out_hold;
		float damp = cfg->damp_k * pitch_rate_dps_f;
		if (damp > cfg->damp_max) damp = cfg->damp_max;
		if (damp < -cfg->damp_max) damp = -cfg->damp_max;
		out += damp;

		if (out > g_pid_angle_two.out_max) out = g_pid_angle_two.out_max;
		if (out < g_pid_angle_two.out_min) out = g_pid_angle_two.out_min;

		out = apply_deadzone(out, SERVO_OUT_DEADZONE);
		ips_show_pid_out = out;
		servo_output_cb(out);
	}

	count_pid++;
	s_sched_cnt++;
	if (s_sched_cnt >= 4u)
	{
		s_sched_cnt = 0u;
	}
	gpio_toggle_level(P22_2);
}
