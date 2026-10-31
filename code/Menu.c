/*
 * Menu.c
 *
 *  Created on: 2026-01-17
 *      Author: hhttiuui
 *
 *  菜单/界面显示模块：
 *  - IPS200 屏幕菜单显示
 *  - 按键切换菜单层级与选项
 *  - 显示 IMU / PID / 速度等调试信息
 */

#include "zf_common_headfile.h"
#include "Menu.h"
#include "Key.h"
#include "motor_controller.h"
#include "Ttop_funtion.h"
#include "top_8_turn_2.h"
#include "flash_storage.h"
#include "common_flie.h"
#include "ttop_speed_change.h"
#include "ttop_kemu1.h"
#include "ttop_kemu3.h"
#include "roll_slope_speed.h"
#include "ins_dead_reckoning.h"
#include "ins_path_follow.h"
#include "gps_display.h"
#include "gps_nav.h"
#include "gps_nav_circle.h"
#include "gps_storage.h"
#include "small_driver_uart_control.h"
#include "ttop_rc_control.h"
#include "liangan.h"

extern float g_lg_pos_err;
extern float g_lg_angle_out;

// =========================
// 菜单：扣扣飞车
// =========================
typedef enum
{
    UI_PAGE_LEGACY = 0,          // 保留原来的 main/second 菜单
    UI_PAGE_KKF_MAIN = 1,        // 界面1：扣扣飞车
    UI_PAGE_KKF_PARAM = 2,       // 界面1_1：参数调试（含模式选择入口+参数列表）
    UI_PAGE_KKF_MODE_SELECT = 3, // 界面1_1_2：模式选择
    UI_PAGE_KKF_EIGHT_TURN = 4,  // 界面1_1_3：8字段表（只读显示）

    UI_PAGE_KKF_INS_PATH_VIEW = 13, // 界面1：航点查看（g_ins_path 分页显示）

    UI_PAGE_KKF_EIGHT2_PROG_SELECT = 10, // 界面1_1_3_1：段表选择（main/ex1/ex2）

    UI_PAGE_KKF_PID_MODE_SELECT = 14, // 界面1_1：PID_MODE 选择页（选择要编辑哪套 PID 参数）
    UI_PAGE_KKF_GPS_DEBUG = 15,      // 界面1_1_9：GPS 调试入口（四项空菜单）
    UI_PAGE_KKF_GPS_ROUTE = 16,      // 专门界面：显示已保存路线（点/线）
    UI_PAGE_KKF_GPS_HEADINGS = 17,   // 专门界面：显示多段航向角
    UI_PAGE_KKF_CON_DBG = 18,        // 界面1_1_10：遥控器通道调试
    UI_PAGE_KKF_DYN_PID = 19,        // 界面1_1_11：动态 PID 参数显示
    UI_PAGE_KKF_KEY_DBG = 20,        // 界面1_1_12：按键状态调试（KEY1/2/3）
    UI_PAGE_KKF_KEMU1_DBG = 21,      // 界面1_1_13：kemu1 段表调试
    UI_PAGE_KKF_KEMU3_DBG = 30,      // 界面1_1_15：kemu3 子菜单
    UI_PAGE_KKF_KEMU3_PATH = 31,     // kemu3 路径参数
    UI_PAGE_KKF_SLOPE_DBG = 32,      // 坡道速度参数
    UI_PAGE_KKF_MOTOR_PID = 33,      // 电机 PID 参数调试
    UI_PAGE_KKF_RC_YAW_CFG = 34,     // MODE3: RC yaw 功能开关
    UI_PAGE_KKF_SPCH_DBG = 27,       // 界面1_1_14：速度变化调试
    UI_PAGE_KKF_GPS_CIRCLE = 28,     // 界面：单点绕圈 GPS 调试
    UI_PAGE_KKF_GPS_STORAGE = 29,    // 界面：GPS 实时存点轨迹显示
    UI_PAGE_KKF_MOTOR_DUTY = 35,     // 界面：左右电机占空比直接调试
    UI_PAGE_KKF_LG_DBG = 40,         // 界面：liangan 摄像头数据调试

    // 界面1_1_4 ~ 1_1_8：PID 参数（直接操作 s_pid_mode_cfg）
    UI_PAGE_KKF_PID_CASCADE = 5,
    UI_PAGE_KKF_PID_CASCADE2 = 11,
    UI_PAGE_KKF_PID_CASCADE3 = 12,
    UI_PAGE_KKF_PID_CASCADE4 = 22,
    UI_PAGE_KKF_PID_CASCADE5 = 23,
    UI_PAGE_KKF_PID_CASCADE6 = 24,
    UI_PAGE_KKF_PID_CASCADE7 = 25,
    UI_PAGE_KKF_PID_CASCADE8 = 26,
    UI_PAGE_KKF_PID_CASCADE9 = 36,
    UI_PAGE_KKF_PID_CASCADE10 = 37,
    UI_PAGE_KKF_PID_CASCADE11 = 38,
    UI_PAGE_KKF_PID_CASCADE12 = 39,
    UI_PAGE_KKF_PID_ANGLE = 6,
    UI_PAGE_KKF_PID_DAMPED_TWO = 7,
    UI_PAGE_KKF_PID_DAMPED_THREE = 8,
    UI_PAGE_KKF_PID_DAMPED_FOUR = 9,
} ui_page_t;

// 上电默认进入“扣扣飞车”主界面
static volatile ui_page_t s_ui_page = UI_PAGE_KKF_MAIN;

// 扣扣飞车：主界面光标（行号从1开始，对齐你当前的“return在第1行”的习惯）
static volatile uint8 s_kkf_main_item = 1u;
static volatile uint8 s_kkf_param_item = 1u;
static volatile uint8 s_kkf_mode_item = 1u;
static volatile uint8 s_kkf_eight_turn_item = 1u;
static volatile uint8 s_kkf_pid_item = 1u;
static volatile uint8 s_kkf_spch_item = 1u;

static volatile int16 s_kkf_motor_duty = 0;
static volatile uint8 s_kkf_motor_duty_enable = 0u;
static volatile uint8 s_kkf_lg_edit = 0u;   /* 0=TGT, 1=ALPHA, 2=CENTER, 3=SWP1, 4=SWP2, 5=TIME, 6=MODE */
#define KKF_MOTOR_DUTY_LONG_PRESS_STEP (100u)

// MODE3 RC Yaw 配置页状态
static volatile uint8 s_kkf_rc_yaw_cfg_item = 1u;
static volatile uint8 s_kkf_rc_yaw_cfg_edit = 0u; // 0=select, 1=edit

// MODE3 RC Yaw 菜单配置开关（可在菜单中编辑）
static volatile uint8 s_mode3_cfg_yaw_only = 1u;
static volatile uint8 s_mode3_cfg_record = 0u;
static volatile uint8 s_mode3_cfg_replay = 0u;
static volatile uint8 s_mode3_cfg_flash_slot = 1u;

// MODE3 RC Yaw 运行开关（START 时由 cfg 锁存）
static volatile uint8 s_mode3_run_yaw_only = 0u;
static volatile uint8 s_mode3_run_record = 0u;
static volatile uint8 s_mode3_run_replay = 0u;

// GPS 调试子菜单当前选项（1..5），用于 kkf_gps_debug_draw / Menu_Select
static volatile uint8 s_kkf_gps_item = 1u;
// GPS 圆绕调试页选项（1..5）
static volatile uint8 s_kkf_gps_circle_item = 1u;

// 菜单中用于选择 motor_request_start 的临时状态（已移除）

// PID 选择：当前选中的 PID 模式（用于 PID_CHANGE 进入编辑）
static volatile pid_mode_t s_kkf_pid_mode_id = PID_MODE_CASCADE;
static volatile uint8 s_kkf_pid_mode_item = 1u; // 1..16

// 航点查看页：当前页（从 0 开始）
static volatile uint16 s_kkf_ins_path_page = 0u;

// eight2 段表选择：当前选择（用于显示/编辑哪一套段表）
static volatile ttop_eight2_prog_id_t s_kkf_eight2_prog_id = TTOP_EIGHT2_PROG_MAIN;
static volatile uint8 s_kkf_eight2_prog_item = 1u; // 1..8

static volatile ttop_eight2_seg_t *kkf_eight2_prog_ptr(ttop_eight2_prog_id_t id)
{
    switch (id)
    {
        case TTOP_EIGHT2_PROG_EX1: return (volatile ttop_eight2_seg_t *)g_eight2_prog_ex1;
        case TTOP_EIGHT2_PROG_EX2: return (volatile ttop_eight2_seg_t *)g_eight2_prog_ex2;
        case TTOP_EIGHT2_PROG_EX3: return (volatile ttop_eight2_seg_t *)g_eight2_prog_ex3;
        case TTOP_EIGHT2_PROG_EX4: return (volatile ttop_eight2_seg_t *)g_eight2_prog_ex4;
        case TTOP_EIGHT2_PROG_EX5: return (volatile ttop_eight2_seg_t *)g_eight2_prog_ex5;
        case TTOP_EIGHT2_PROG_EX6: return (volatile ttop_eight2_seg_t *)g_eight2_prog_ex6;
        case TTOP_EIGHT2_PROG_EX7: return (volatile ttop_eight2_seg_t *)g_eight2_prog_ex7;
        case TTOP_EIGHT2_PROG_MAIN:
        default: return (volatile ttop_eight2_seg_t *)g_eight2_prog;
    }
}

static const char *kkf_eight2_prog_str(ttop_eight2_prog_id_t id)
{
    switch (id)
    {
        case TTOP_EIGHT2_PROG_EX1: return "EX1";
        case TTOP_EIGHT2_PROG_EX2: return "EX2";
        case TTOP_EIGHT2_PROG_EX3: return "EX3";
        case TTOP_EIGHT2_PROG_EX4: return "EX4";
        case TTOP_EIGHT2_PROG_EX5: return "EX5";
        case TTOP_EIGHT2_PROG_EX6: return "EX6";
        case TTOP_EIGHT2_PROG_EX7: return "EX7";
        case TTOP_EIGHT2_PROG_MAIN:
        default: return "MAIN";
    }
}

// MT_ST 显示字符串：当被选中时返回带 '#' 的高亮样式
// moto start menu removed

static const char *kkf_pid_mode_str(pid_mode_t mode)
{
    switch (mode)
    {
        case PID_MODE_CASCADE: return "CASCADE";
        case PID_MODE_CASCADE_EX1: return "CASCADE2";
        case PID_MODE_CASCADE_EX2: return "CASCADE3";
        case PID_MODE_CASCADE_EX3: return "CASCADE4";
        case PID_MODE_CASCADE_EX4: return "CASCADE5";
        case PID_MODE_CASCADE_EX5: return "CASCADE6";
        case PID_MODE_CASCADE_EX6: return "CASCADE7";
        case PID_MODE_CASCADE_EX7: return "CASCADE8";
        case PID_MODE_CASCADE_EX8: return "CASCADE9";
        case PID_MODE_CASCADE_EX9: return "CASCADE10";
        case PID_MODE_CASCADE_EX10: return "CASCADE11";
        case PID_MODE_CASCADE_EX11: return "CASCADE12";
        case PID_MODE_ANGLE: return "ANGLE";
        case PID_MODE_DAMPED_TWO: return "DAMPED2";
        case PID_MODE_DAMPED_THREE: return "DAMPED3";
        case PID_MODE_DAMPED_FOUR: return "DAMPED4";
        default: return "CASCADE";
    }
}

static uint8 kkf_pid_mode_to_item(pid_mode_t mode)
{
    switch (mode)
    {
        case PID_MODE_CASCADE: return 1u;
        case PID_MODE_CASCADE_EX1: return 2u;
        case PID_MODE_CASCADE_EX2: return 3u;
        case PID_MODE_CASCADE_EX3: return 4u;
        case PID_MODE_CASCADE_EX4: return 5u;
        case PID_MODE_CASCADE_EX5: return 6u;
        case PID_MODE_CASCADE_EX6: return 7u;
        case PID_MODE_CASCADE_EX7: return 8u;
        case PID_MODE_CASCADE_EX8: return 9u;
        case PID_MODE_CASCADE_EX9: return 10u;
        case PID_MODE_CASCADE_EX10: return 11u;
        case PID_MODE_CASCADE_EX11: return 12u;
        case PID_MODE_ANGLE: return 13u;
        case PID_MODE_DAMPED_TWO: return 14u;
        case PID_MODE_DAMPED_THREE: return 15u;
        case PID_MODE_DAMPED_FOUR: return 16u;
        default: return 1u;
    }
}

static pid_mode_t kkf_item_to_pid_mode(uint8 item)
{
    switch (item)
    {
        case 2u: return PID_MODE_CASCADE_EX1;
        case 3u: return PID_MODE_CASCADE_EX2;
        case 4u: return PID_MODE_CASCADE_EX3;
        case 5u: return PID_MODE_CASCADE_EX4;
        case 6u: return PID_MODE_CASCADE_EX5;
        case 7u: return PID_MODE_CASCADE_EX6;
        case 8u: return PID_MODE_CASCADE_EX7;
        case 9u: return PID_MODE_CASCADE_EX8;
        case 10u: return PID_MODE_CASCADE_EX9;
        case 11u: return PID_MODE_CASCADE_EX10;
        case 12u: return PID_MODE_CASCADE_EX11;
        case 13u: return PID_MODE_ANGLE;
        case 14u: return PID_MODE_DAMPED_TWO;
        case 15u: return PID_MODE_DAMPED_THREE;
        case 16u: return PID_MODE_DAMPED_FOUR;
        case 1u:
        default: return PID_MODE_CASCADE;
    }
}

static ui_page_t kkf_pid_mode_to_page(pid_mode_t mode)
{
    switch (mode)
    {
        case PID_MODE_CASCADE: return UI_PAGE_KKF_PID_CASCADE;
        case PID_MODE_CASCADE_EX1: return UI_PAGE_KKF_PID_CASCADE2;
        case PID_MODE_CASCADE_EX2: return UI_PAGE_KKF_PID_CASCADE3;
        case PID_MODE_CASCADE_EX3: return UI_PAGE_KKF_PID_CASCADE4;
        case PID_MODE_CASCADE_EX4: return UI_PAGE_KKF_PID_CASCADE5;
        case PID_MODE_CASCADE_EX5: return UI_PAGE_KKF_PID_CASCADE6;
        case PID_MODE_CASCADE_EX6: return UI_PAGE_KKF_PID_CASCADE7;
        case PID_MODE_CASCADE_EX7: return UI_PAGE_KKF_PID_CASCADE8;
        case PID_MODE_CASCADE_EX8: return UI_PAGE_KKF_PID_CASCADE9;
        case PID_MODE_CASCADE_EX9: return UI_PAGE_KKF_PID_CASCADE10;
        case PID_MODE_CASCADE_EX10: return UI_PAGE_KKF_PID_CASCADE11;
        case PID_MODE_CASCADE_EX11: return UI_PAGE_KKF_PID_CASCADE12;
        case PID_MODE_ANGLE: return UI_PAGE_KKF_PID_ANGLE;
        case PID_MODE_DAMPED_TWO: return UI_PAGE_KKF_PID_DAMPED_TWO;
        case PID_MODE_DAMPED_THREE: return UI_PAGE_KKF_PID_DAMPED_THREE;
        case PID_MODE_DAMPED_FOUR: return UI_PAGE_KKF_PID_DAMPED_FOUR;
        default: return UI_PAGE_KKF_PID_CASCADE;
    }
}

typedef enum
{
    KKF_EIGHT_TURN_SEL = 0u,   // '>' 选择行
    KKF_EIGHT_TURN_EDIT_DEG,  // '-' 编辑偏移量
    KKF_EIGHT_TURN_EDIT_DPS,  // '*' 编辑限速
} kkf_eight_turn_state_t;

static volatile kkf_eight_turn_state_t s_kkf_eight_turn_state = KKF_EIGHT_TURN_SEL;

typedef enum
{
    KKF_KEMU1_SEL = 0u,          // '>' 选择行
    KKF_KEMU1_EDIT_METERS = 1u,  // '-' 编辑左列（计数/距离）
    KKF_KEMU1_EDIT_SPEED = 2u,   // '*' 编辑右列（速度）
} kkf_kemu1_state_t;

static volatile kkf_kemu1_state_t s_kkf_kemu1_state = KKF_KEMU1_SEL;
static volatile uint8 s_kkf_kemu1_item = 0u; // 0=counts_per_meter, 1..7=seg0..6

typedef enum
{
    KKF_KEMU3_SEL = 0u,          // '>' 选择行
    KKF_KEMU3_EDIT_METERS = 1u,  // '-' 编辑左列（距离）
    KKF_KEMU3_EDIT_SPEED = 2u,   // '*' 编辑右列（速度）
} kkf_kemu3_state_t;

static volatile kkf_kemu3_state_t s_kkf_kemu3_state = KKF_KEMU3_SEL;
static volatile uint8 s_kkf_kemu3_item = 0u; // 0=counts_per_meter, 1..8=seg0..7
static volatile uint8 s_kkf_kemu3_menu_item = 1u; // 1..9
static volatile uint8 s_kkf_kemu3_dbg_edit = 0u;  // 0=select, 1=edit end deg, 2=edit end dps, 3=edit EX2 loop
static volatile uint8 s_kemu3_cfg_record = 0u;
static volatile uint8 s_kemu3_cfg_replay = 0u;
static volatile uint8 s_kemu3_cfg_flash_slot = 1u;

typedef enum
{
    KKF_SLOPE_FIELD_ENABLE = 1u,
    KKF_SLOPE_FIELD_DIST,
    KKF_SLOPE_FIELD_UP_START,
    KKF_SLOPE_FIELD_UP_END,
    KKF_SLOPE_FIELD_DOWN_START,
    KKF_SLOPE_FIELD_DOWN_END,
    KKF_SLOPE_FIELD_UP_DELTA,
    KKF_SLOPE_FIELD_PRE_UP_DECEL,
    KKF_SLOPE_FIELD_CREST_DELTA,
    KKF_SLOPE_FIELD_FLAT_DELTA,
    KKF_SLOPE_FIELD_COUNT = KKF_SLOPE_FIELD_FLAT_DELTA,
} kkf_slope_field_t;

typedef enum
{
    KKF_SLOPE_STATE_SEL = 0u,
    KKF_SLOPE_STATE_EDIT = 1u,
} kkf_slope_state_t;

static volatile kkf_slope_state_t s_kkf_slope_state = KKF_SLOPE_STATE_SEL;
static volatile uint8 s_kkf_slope_item = 1u;

typedef enum
{
    KKF_SPCH_FIELD_FIT_MODE = 1u,
    KKF_SPCH_FIELD_LPF_ALPHA,
    KKF_SPCH_FIELD_ANG_LPF,
    KKF_SPCH_FIELD_PITCH_ZERO,
    KKF_SPCH_FIELD_SERVO_ZERO,
    KKF_SPCH_FIELD_COUNT = KKF_SPCH_FIELD_SERVO_ZERO,
} kkf_spch_field_t;

typedef enum
{
    KKF_SPCH_STATE_SEL = 0u,
    KKF_SPCH_STATE_EDIT = 1u,
} kkf_spch_state_t;

static volatile kkf_spch_state_t s_kkf_spch_state = KKF_SPCH_STATE_SEL;

typedef enum
{
    KKF_MOTOR_PID_FIELD_KP = 1u,
    KKF_MOTOR_PID_FIELD_KI,
    KKF_MOTOR_PID_FIELD_KD,
    KKF_MOTOR_PID_FIELD_BT_MODE,
    KKF_MOTOR_PID_FIELD_DUTY_KD,
    KKF_MOTOR_PID_FIELD_PITCH_KD,
    KKF_MOTOR_PID_FIELD_ACC_KD,
    KKF_MOTOR_PID_FIELD_BOOST_MAX,
    KKF_MOTOR_PID_FIELD_COUNT = KKF_MOTOR_PID_FIELD_BOOST_MAX,
} kkf_motor_pid_field_t;

typedef enum
{
    KKF_MOTOR_PID_STATE_SEL = 0u,
    KKF_MOTOR_PID_STATE_EDIT = 1u,
} kkf_motor_pid_state_t;

static volatile kkf_motor_pid_state_t s_kkf_motor_pid_state = KKF_MOTOR_PID_STATE_SEL;
static volatile uint8 s_kkf_motor_pid_item = 1u;

// 模式：1~12（与顶层 funtion_kemu 对齐）
static volatile uint8 s_kkf_mode = 1u;
// GPS 导航使能：ON 时 START 会启用 gps_nav；OFF 时 START 不启用
static volatile uint8 s_kkf_gps_nav_enable = 0u;

// 与顶层调度融合：菜单直接接管模式/ready/清零等动作
extern volatile uint8 eight_flag;
extern volatile uint16 eight_count;
extern volatile uint32 g_uart1_rx_isr_count;
void ttop_funtion_mode_once_apply(void);

static const char *kkf_mode_str(uint8 mode)
{
    switch (mode)
    {
        case 1u: return "MODE 1 200";
        case 2u: return "MODE 2 SPCHD";
        case 3u: return "MODE 3 55";
        case 4u: return "MODE 4 45";
        case 5u: return "MODE 5 200+T";
        case 6u: return "MODE 6 8-TURN";
        case 7u: return "MODE 7 K3";
        case 8u: return "MODE 8";
        case 9u: return "MODE 9 INS-REC";
        case 10u: return "MODE10 INS-TRK";
        case 11u: return "MODE11";
        case 12u: return "MODE12";
        default: return "MODE 1";
    }
}

static const char *kkf_gps_nav_sw_str(void)
{
    return (s_kkf_gps_nav_enable != 0u) ? "ON" : "OFF";
}

static void kkf_draw_signature_bottom(void)
{
    // 在所有页面底部显示署名：MADE_BY -> hhttiuui
    // 使用 ips200_height_max 动态计算最后一行的 y，避免不同旋转/分辨率下越界。
    uint16 y;

    // 8x16 字体高度约 16 像素，这里把基线放在倒数第 16 像素处。
    if (ips200_height_max > 16u)
    {
        y = (uint16)(ips200_height_max - 16u);
    }
    else
    {
        y = 0u;
    }

    ips200_show_string(0, y, "MADE_BY -> hhttiuui");
}

static void kkf_main_draw(void)
{
    // 界面1：
    // 扣扣飞车
    //   参数调试
    //   START
    //   RESET
    //   STATE(显示是否开始跑)
    //   清零
    //   模式：（显示当前模式）
    //   yaw：
    //   pitch：
    // > 速度：
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "PID_CONFIG");
    ips200_show_string(16, 2 * 15, "START");
    ips200_show_string(16, 3 * 15, "RESET");
    ips200_show_string(16, 4 * 15, "CLEAR");
    ips200_show_string(16, 5 * 15, "GPS:");
    ips200_show_string(16 * 6, 5 * 15, (char *)kkf_gps_nav_sw_str());
    ips200_show_string(16, 6 * 15, "GPS_DBG");
    ips200_show_string(16, 7 * 15, "STATE:");
    ips200_show_string(16 * 6, 7 * 15, funtion_ready_flag ? "RUN" : "STOP");
    ips200_show_string(16, 8 * 15, "MODE:");
    ips200_show_string(16 * 6, 8 * 15, (char *)kkf_mode_str(s_kkf_mode));
    ips200_show_string(16, 9 * 15, "ball:");
    ips200_show_int(16 * 6, 9 * 15, (int32)g_liangan_ball_pos, 4);
    ips200_show_string(16, 10 * 15, "t_bal:");
    ips200_show_int(16 * 6, 10 * 15, (int32)g_liangan_target, 4);
    ips200_show_string(16, 11 * 15, "pitch:");
    ips200_show_float(16 * 6, 11 * 15, show_pitch_deg, 8, 2);
    ips200_show_string(16, 12 * 15, "roll:");
    ips200_show_float(16 * 6, 12 * 15, show_roll_deg, 8, 2);
    ips200_show_string(16, 13 * 15, "SPD:");
    ips200_show_int(16 * 6, 13 * 15, motor_speed_raw_counts, 6);

    ips200_show_string(16, 14 * 15, "xy:");
    ips200_show_float(16 * 3, 14 * 15, ins_dr_get_pos_x(), 2, 3);
    ips200_show_float(16 * 8, 14 * 15, ins_dr_get_pos_y(), 2, 3);

    ips200_show_string(16, 15 * 15, "g_ins_path");

    ips200_show_string(16, 16 * 15, "GPSStorage:");
    ips200_show_string(16 * 12, 16 * 15, gps_storage_is_recording() ? "on" : "off");

    ips200_show_string(0, s_kkf_main_item * 15, "> ");

    kkf_draw_signature_bottom();
}

static inline uint16 kkf_ui_rows_available(void)
{
    // 以 15px 行高为准，并为底部签名预留 16px。
    uint16 usable_h = (ips200_height_max > 16u) ? (uint16)(ips200_height_max - 16u) : ips200_height_max;
    uint16 rows = (usable_h / 15u);
    return (rows > 0u) ? rows : 1u;
}

static void kkf_ins_path_draw(void)
{
    // 航点查看页：按屏幕可显示行数分页
    // - KEY1/KEY3：翻页
    // - KEY4：返回
    // 显示：索引 + x + y

    uint16 rows = kkf_ui_rows_available();
    uint16 count = g_ins_path_count;
    if (count > (uint16)INS_PATH_MAX_POINTS) count = (uint16)INS_PATH_MAX_POINTS;

    // 预留 2 行标题/信息
    uint16 points_per_page = (rows > 2u) ? (uint16)(rows - 2u) : 1u;
    if (points_per_page == 0u) points_per_page = 1u;

    uint16 page_count = 1u;
    if (count > 0u)
    {
        page_count = (uint16)((count + points_per_page - 1u) / points_per_page);
        if (page_count == 0u) page_count = 1u;
    }

    uint16 page = s_kkf_ins_path_page;
    if (page >= page_count) page = (uint16)(page_count - 1u);
    s_kkf_ins_path_page = page;

    uint16 start = (uint16)(page * points_per_page);
    uint16 end = (uint16)(start + points_per_page);
    if (end > count) end = count;

    ips200_show_string(0, 0, "G_INS_PATH");

    // 第 1 行：N + page
    ips200_show_string(0, 1 * 15, "N:");
    ips200_show_int(16 * 2, 1 * 15, (int32)count, 5);
    ips200_show_string(16 * 6, 1 * 15, "P:");
    ips200_show_int(16 * 8, 1 * 15, (int32)(page + 1u), 3);
    ips200_show_string(16 * 10, 1 * 15, "/");
    ips200_show_int(16 * 11, 1 * 15, (int32)page_count, 3);

    // 航点列表：从第 2 行开始
    if (count == 0u)
    {
        ips200_show_string(0, 2 * 15, "EMPTY");
        kkf_draw_signature_bottom();
        return;
    }

    for (uint16 i = start; i < end; i++)
    {
        uint16 row = (uint16)(2u + (i - start));
        uint16 y = (uint16)(row * 15u);

        // idx
        ips200_show_int(0, y, (int32)i, 4);
        // x / y
        ips200_show_float(16 * 3, y, g_ins_path[i].x, 7, 3);
        ips200_show_float(16 * 8, y, g_ins_path[i].y, 7, 3);
    }

    kkf_draw_signature_bottom();
}

static void kkf_param_draw(void)
{
    // 界面1_1：
    // 扣扣飞车
    //   模式选择：（当前模式）
    //   eight_turn
    //   PID_CHANGE（进入编辑）
    //   PID_MODE（选择要改哪套 PID）
    //   CON_DBG（遥控器通道调试）
    //   DYN_PID（动态速度/插值 Kp）
    //   KEY_DBG（按键状态）
    //   SPCH_DBG（速度变化调试）
    //   KEMU1_DBG（kemu1 段表）
    //   KEMU3_DBG（kemu3 段表）
    //   MOTOR_PID（电机 PID）
    //   RC_YAW_CFG（MODE3: yaw/record/replay 开关）
    //   MOTOR_DUTY（左右电机占空比直接调试）
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "MODE SEL:");
    ips200_show_string(16 * 8, 1 * 15, (char *)kkf_mode_str(s_kkf_mode));
    ips200_show_string(16, 2 * 15, "EIGHT_TURN");
    ips200_show_string(16, 3 * 15, "E8_PROG:");
    ips200_show_string(16 * 8, 3 * 15, (char *)kkf_eight2_prog_str(s_kkf_eight2_prog_id));
    ips200_show_string(16, 4 * 15, "PID_CHANGE");
    ips200_show_string(16, 5 * 15, "PID_MODE:");
    ips200_show_string(16 * 8, 5 * 15, (char *)kkf_pid_mode_str((pid_mode_t)s_kkf_pid_mode_id));
    ips200_show_string(16, 6 * 15, "CON_DBG");
    ips200_show_string(16, 7 * 15, "DYN_PID");
    ips200_show_string(16, 8 * 15, "KEY_DBG");
    ips200_show_string(16, 9 * 15, "SPCH_DBG");
    ips200_show_string(16, 10 * 15, "KEMU1_DBG");
    ips200_show_string(16, 11 * 15, "KEMU3_DBG");
    ips200_show_string(16, 12 * 15, "MOTOR_PID");
    ips200_show_string(16, 13 * 15, "RC_YAW_CFG");
    ips200_show_string(16, 14 * 15, "MOTOR_DUTY");
    ips200_show_string(16, 15 * 15, "LG_DBG");

    ips200_show_string(0, s_kkf_param_item * 15, "> ");

    kkf_draw_signature_bottom();
}

static void kkf_motor_duty_draw(void)
{
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "MOTOR_DUTY");
    ips200_show_string(16, 3 * 15, "DUTY:");
    ips200_show_int(16 * 7, 3 * 15, s_kkf_motor_duty, 6);
    ips200_show_string(16, 4 * 15, "OUTPUT:");
    ips200_show_string(16 * 8, 4 * 15, s_kkf_motor_duty_enable ? "ON" : "OFF");
    ips200_show_string(16, 6 * 15, "K1/K3: +/-1");
    ips200_show_string(16, 7 * 15, "LONG: +/-100");
    ips200_show_string(16, 8 * 15, "K2: ON/OFF");
    kkf_draw_signature_bottom();
}

static void kkf_lg_dbg_draw(void)
{
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "LG_DBG");
    ips200_show_string(16, 2 * 15, "CH0:");
    ips200_show_int(16 * 5, 2 * 15, (int32)uart_receiver.channel[0], 5);

    ips200_show_string(16, 3 * 15, "TGT:");
    ips200_show_int(16 * 6, 3 * 15, (int32)g_liangan_target, 5);
    ips200_show_string(16, 4 * 15, "BALL:");
    ips200_show_int(16 * 6, 4 * 15, (int32)g_liangan_ball_pos, 5);
    ips200_show_string(16, 5 * 15, "F_POS:");
    ips200_show_float(16 * 7, 5 * 15, g_liangan_ball_filt, 8, 1);
    ips200_show_string(16, 6 * 15, "POS_ERR:");
    ips200_show_float(16 * 7, 6 * 15, g_lg_pos_err, 8, 1);

    ips200_show_string(16, 7 * 15, "SPD2:");
    ips200_show_float(16 * 7, 7 * 15, g_liangan_speed_calc, 8, 1);
    ips200_show_string(16, 8 * 15, "ANG_OUT:");
    ips200_show_float(16 * 7, 8 * 15, g_lg_angle_out, 8, 1);

    ips200_show_string(16, 9 * 15, "ALPHA:");
    ips200_show_float(16 * 7, 9 * 15, g_liangan_alpha_pos, 5, 2);

    ips200_show_string(16, 10 * 15, "CENTER:");
    ips200_show_int(16 * 7, 10 * 15, (int32)(400 + g_liangan_center_offset), 5);

    ips200_show_string(16, 11 * 15, "SWP_DN:");
    ips200_show_int(16 * 7, 11 * 15, (int32)g_liangan_sweep_ofs1, 5);

    ips200_show_string(16, 12 * 15, "SWP_UP:");
    ips200_show_int(16 * 7, 12 * 15, (int32)g_liangan_sweep_ofs2, 5);

    ips200_show_string(16, 13 * 15, "TIME:");
    ips200_show_int(16 * 7, 13 * 15, (int32)g_liangan_sweep_time_ms, 5);

    ips200_show_string(16, 14 * 15, "MODE:");
    {
        const char *ms;
        if      (g_liangan_sweep_mode == 0u) ms = "CLOSED";
        else if (g_liangan_sweep_mode == 1u) ms = " OPEN ";
        else if (g_liangan_sweep_mode == 2u) ms = "RC_DIR";
        else if (g_liangan_sweep_mode == 3u) ms = "RC_REC";
        else                                 ms = "RC_PLY";
        ips200_show_string(16 * 7, 14 * 15, ms);
    }

    ips200_show_string(16, 15 * 15, "SERVO:");
    ips200_show_float(16 * 7, 15 * 15, ips_show_pid_out, 8, 1);

    /* 编辑光标：0=TGT(3), 1=ALPHA(9), 2=CENTER(10), 3=SWP1(11), 4=SWP2(12), 5=TIME(13), 6=MODE(14) */
    {
        uint16 cur_row;
        if      (s_kkf_lg_edit == 0u) cur_row = 3u;
        else if (s_kkf_lg_edit == 1u) cur_row = 9u;
        else if (s_kkf_lg_edit == 2u) cur_row = 10u;
        else if (s_kkf_lg_edit == 3u) cur_row = 11u;
        else if (s_kkf_lg_edit == 4u) cur_row = 12u;
        else if (s_kkf_lg_edit == 5u) cur_row = 13u;
        else                          cur_row = 14u;
        ips200_show_string(0, cur_row * 15, ">");
    }

    ips200_show_string(16, 16 * 15, "K1/K3:+/- K2:field");

    kkf_draw_signature_bottom();
}

static void kkf_dyn_pid_draw(void)
{
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "DYN_PID");

    ips200_show_string(16, 3 * 15, "SPD_F:");
    ips200_show_float(16 * 7, 3 * 15, ttop_speed_change_get_speed_abs_filt(), 8, 2);

    ips200_show_string(16, 5 * 15, "ANG_KP:");
    ips200_show_float(16 * 7, 5 * 15, ttop_speed_change_get_angle_kp(), 8, 3);

    ips200_show_string(16, 7 * 15, "RATE_KP:");
    ips200_show_float(16 * 7, 7 * 15, ttop_speed_change_get_rate_kp(), 8, 3);

    ips200_show_string(16, 9 * 15, "YAW_KP:");
    ips200_show_float(16 * 7, 9 * 15, ttop_speed_change_get_yaw_kp(), 8, 3);

    kkf_draw_signature_bottom();
}

static void kkf_spch_dbg_draw(void)
{
    char cursor[3] = ">";
    if (s_kkf_spch_state == KKF_SPCH_STATE_EDIT) cursor[0] = '-';

    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "SPCH_DBG");

    ips200_show_string(16, 3 * 15, "FIT_MODE:");
    ips200_show_int(16 * 8, 3 * 15, (int32)g_ttop_speed_change_fit_mode, 2);

    ips200_show_string(16, 4 * 15, "LPF_ALPHA:");
    ips200_show_float(16 * 8, 4 * 15, g_ttop_speed_change_lpf_alpha, 4, 2);

    ips200_show_string(16, 5 * 15, "ANG_LPF:");
    ips200_show_float(16 * 8, 5 * 15, g_pitch_angle_lpf_alpha, 5, 3);

    ips200_show_string(16, 6 * 15, "PITCH_Z :");
    ips200_show_float(16 * 8, 6 * 15, g_pitch_zero_offset_deg, 6, 2);

    ips200_show_string(16, 7 * 15, "SERVO_Z:");
    ips200_show_float(16 * 8, 7 * 15, g_servo_zero_offset_deg, 6, 2);

    ips200_show_string(0, (uint16)((2u + (uint16)s_kkf_spch_item) * 15u), cursor);

    kkf_draw_signature_bottom();
}

static const char *kkf_on_off_str(uint8 on)
{
    return (on != 0u) ? "ON" : "OFF";
}

static void kkf_rc_yaw_cfg_draw(void)
{
    char cursor[3] = ">";
    if (s_kkf_rc_yaw_cfg_edit) cursor[0] = '-';

    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "RC_YAW_CFG");

    ips200_show_string(16, 3 * 15, "YAW_ONLY:");
    ips200_show_string(16 * 8, 3 * 15, (char *)kkf_on_off_str(s_mode3_cfg_yaw_only));

    ips200_show_string(16, 4 * 15, "RECORD :");
    ips200_show_string(16 * 8, 4 * 15, (char *)kkf_on_off_str(s_mode3_cfg_record));

    ips200_show_string(16, 5 * 15, "REPLAY :");
    ips200_show_string(16 * 8, 5 * 15, (char *)kkf_on_off_str(s_mode3_cfg_replay));
    ips200_show_string(16, 6 * 15, "FLASH_ID:");
    ips200_show_int(16 * 8, 6 * 15, s_mode3_cfg_flash_slot, 1);

    ips200_show_string(16, 7 * 15, "POINTS:");
    ips200_show_int(16 * 8, 7 * 15, (int32)ttop_rc_yaw_record_get_count(), 5);

    ips200_show_string(0, (uint16)((uint16)(2u + s_kkf_rc_yaw_cfg_item) * 15u), cursor);

    kkf_draw_signature_bottom();
}

static void kkf_motor_pid_draw(void)
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float duty_k = 0.0f;
    float pitch_k = 0.0f;
    float acc_k = 0.0f;
    float pitch_boost = 0.0f;
    uint8 boost_mode = 0u;
    const char *mode_str = "OFF";
    char cursor[3] = ">";

    motor_get_pid_gains(&kp, &ki, &kd);
    motor_get_boost_map_k(&duty_k, &pitch_k, &acc_k);
    boost_mode = motor_get_boost_mode();
    if (boost_mode == 1u) mode_str = "DUTY";
    else if (boost_mode == 2u) mode_str = "PITCH";
    else if (boost_mode == 3u) mode_str = "ACC";
    pitch_boost = motor_get_pitch_speed_boost_counts();
    if (s_kkf_motor_pid_state == KKF_MOTOR_PID_STATE_EDIT) cursor[0] = '-';

    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "MOTOR_PID");

    ips200_show_string(16, 3 * 15, "MOT_KP:");
    ips200_show_float(16 * 8, 3 * 15, kp, 5, 2);

    ips200_show_string(16, 4 * 15, "MOT_KI:");
    ips200_show_float(16 * 8, 4 * 15, ki, 5, 2);

    ips200_show_string(16, 5 * 15, "MOT_KD:");
    ips200_show_float(16 * 8, 5 * 15, kd, 5, 2);

    ips200_show_string(16, 6 * 15, "BT_MOD:");
    ips200_show_string(16 * 8, 6 * 15, (char *)mode_str);

    ips200_show_string(16, 7 * 15, "DUT_KD:");
    ips200_show_float(16 * 8, 7 * 15, duty_k, 5, 2);

    ips200_show_string(16, 8 * 15, "PIT_KD:");
    ips200_show_float(16 * 8, 8 * 15, pitch_k, 5, 2);

    ips200_show_string(16, 9 * 15, "ACC_KD:");
    ips200_show_float(16 * 8, 9 * 15, acc_k, 5, 2);

    ips200_show_string(16, 10 * 15, "BT_MAX:");
    ips200_show_float(16 * 8, 10 * 15, motor_get_pitch_speed_boost_max_counts(), 5, 2);

    ips200_show_string(16, 11 * 15, "BOOST:");
    ips200_show_float(16 * 8, 11 * 15, pitch_boost, 5, 2);

    ips200_show_string(16, 12 * 15, "DUTY:");
    ips200_show_int(16 * 8, 12 * 15, (int32)show_duty, 6);

    ips200_show_string(0, (uint16)((2u + (uint16)s_kkf_motor_pid_item) * 15u), cursor);

    kkf_draw_signature_bottom();
}

static void kkf_key_dbg_draw(void)
{
    // 仅显示 KEY1/KEY2/KEY3 状态，退出键 KEY4 不显示
    uint8 k1 = Key_Read(KEY_ID_1);
    uint8 k2 = Key_Read(KEY_ID_2);
    uint8 k3 = Key_Read(KEY_ID_3);
    uint8 l1_evt = Key_ReadLongPress(KEY_ID_1);
    uint8 l2_evt = Key_ReadLongPress(KEY_ID_2);
    uint8 l3_evt = Key_ReadLongPress(KEY_ID_3);

    // 长按事件是单次脉冲，这里做短暂保持，便于肉眼观察
    static uint8 l1_hold = 0u;
    static uint8 l2_hold = 0u;
    static uint8 l3_hold = 0u;
    const uint8 HOLD_FRAMES = 20u;

    if (l1_evt) l1_hold = HOLD_FRAMES;
    else if (l1_hold > 0u) l1_hold--;

    if (l2_evt) l2_hold = HOLD_FRAMES;
    else if (l2_hold > 0u) l2_hold--;

    if (l3_evt) l3_hold = HOLD_FRAMES;
    else if (l3_hold > 0u) l3_hold--;

    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "KEY_DBG");

    ips200_show_string(16, 3 * 15, "KEY1:");
    ips200_show_int(16 * 7, 3 * 15, (int32)k1, 1);

    ips200_show_string(16, 4 * 15, "KEY2:");
    ips200_show_int(16 * 7, 4 * 15, (int32)k2, 1);

    ips200_show_string(16, 5 * 15, "KEY3:");
    ips200_show_int(16 * 7, 5 * 15, (int32)k3, 1);

    ips200_show_string(16, 7 * 15, "LP1:");
    ips200_show_int(16 * 7, 7 * 15, (int32)((l1_hold > 0u) ? 1u : 0u), 1);

    ips200_show_string(16, 8 * 15, "LP2:");
    ips200_show_int(16 * 7, 8 * 15, (int32)((l2_hold > 0u) ? 1u : 0u), 1);

    ips200_show_string(16, 9 * 15, "LP3:");
    ips200_show_int(16 * 7, 9 * 15, (int32)((l3_hold > 0u) ? 1u : 0u), 1);

    kkf_draw_signature_bottom();
}

static void kkf_kemu1_dbg_draw(void)
{
    uint8 i;
    char cursor[3] = ">";

    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "KEMU1_DBG");

    if (s_kkf_kemu1_state == KKF_KEMU1_SEL)
    {
        cursor[0] = '>';
    }
    else if (s_kkf_kemu1_state == KKF_KEMU1_EDIT_METERS)
    {
        cursor[0] = '-';
    }
    else
    {
        cursor[0] = '*';
    }

    // 首行：1m 对应计数
    if (s_kkf_kemu1_item == 0u)
    {
        ips200_show_string(0, 2 * 15, cursor);
    }
    else
    {
        ips200_show_string(0, 2 * 15, " ");
    }
    ips200_show_string(16, 2 * 15, "CNT_1M:");
    ips200_show_int(16 * 8, 2 * 15, (int32)g_ttop_kemu1_counts_per_meter, 6);

    // 后七行：距离 + 速度
    for (i = 0u; i < 7u; i++)
    {
        uint16 y = (uint16)((3u + (uint16)i) * 15u);
        uint8 row_item = (uint8)(i + 1u);

        if (s_kkf_kemu1_item == row_item)
        {
            ips200_show_string(0, y, cursor);
        }
        else
        {
            ips200_show_string(0, y, " ");
        }

        ips200_show_float(16 * 1, y, g_ttop_kemu1_seg_table[i].meters, 6, 1);
        ips200_show_int(16 * 7, y, (int32)g_ttop_kemu1_seg_table[i].speed_counts_5ms, 5);
    }

    kkf_draw_signature_bottom();
}

static void kkf_kemu3_dbg_draw(void)
{
    char cursor[3] = "> ";
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "KEMU3_DBG");

    ips200_show_string(16, 2 * 15, "KM3_PATH");
    ips200_show_string(16, 3 * 15, "END:");
    ips200_show_float(16 * 4, 3 * 15, g_ttop_kemu3_end_seg.target_rel_deg, 6, 1);
    ips200_show_float(16 * 9, 3 * 15, g_ttop_kemu3_end_seg.rate_dps, 6, 1);
    ips200_show_string(16, 4 * 15, "EX2_LOOP:");
    ips200_show_int(16 * 9, 4 * 15, (int32)g_ttop_kemu3_ex2_loop_count, 3);
    ips200_show_string(16, 5 * 15, "YAW_OFS:");
    ips200_show_float(16 * 9, 5 * 15, g_yaw_zero_offset_deg, 6, 2);
    ips200_show_string(16, 6 * 15, "YAW_KI:");
    ips200_show_float(16 * 9, 6 * 15, g_kemu3_yaw_ki_path, 6, 3);
    ips200_show_string(16, 7 * 15, "RECORD:");
    ips200_show_string(16 * 8, 7 * 15, (char *)kkf_on_off_str(s_kemu3_cfg_record));
    ips200_show_string(16, 8 * 15, "REPLAY:");
    ips200_show_string(16 * 8, 8 * 15, (char *)kkf_on_off_str(s_kemu3_cfg_replay));
    ips200_show_string(16, 9 * 15, "FLASH_ID:");
    ips200_show_int(16 * 9, 9 * 15, (int32)s_kemu3_cfg_flash_slot, 1);
    ips200_show_string(16, 10 * 15, "USE_REC:");
    ips200_show_string(16 * 8, 10 * 15, (g_kemu3_use_recorded_turn == 1u) ? "AUTO" :
                                         (g_kemu3_use_recorded_turn == 2u) ? "REC " :
                                         (g_kemu3_use_recorded_turn == 3u) ? "REP " : "?");
    ips200_show_string(16, 11 * 15, "SLOPE_DBG");

    if (s_kkf_kemu3_dbg_edit == 1u) cursor[0] = '-';
    else if (s_kkf_kemu3_dbg_edit == 2u) cursor[0] = '*';
    else if (s_kkf_kemu3_dbg_edit == 3u) cursor[0] = '-';
    else if (s_kkf_kemu3_dbg_edit == 4u) cursor[0] = '-';
    else if (s_kkf_kemu3_dbg_edit == 5u) cursor[0] = '-';
    else if (s_kkf_kemu3_dbg_edit == 6u) cursor[0] = '-';
    else if (s_kkf_kemu3_dbg_edit == 7u) cursor[0] = '-';
    else if (s_kkf_kemu3_dbg_edit == 8u) cursor[0] = '-';
    else if (s_kkf_kemu3_dbg_edit == 9u) cursor[0] = '-';
    ips200_show_string(0, (uint16)((1u + (uint16)s_kkf_kemu3_menu_item) * 15u), cursor);

    kkf_draw_signature_bottom();
}

static void kkf_kemu3_path_draw(void)
{
    uint8 i;
    char cursor[3] = ">";

    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "KM3_PATH");

    if (s_kkf_kemu3_state == KKF_KEMU3_SEL)
    {
        cursor[0] = '>';
    }
    else if (s_kkf_kemu3_state == KKF_KEMU3_EDIT_METERS)
    {
        cursor[0] = '-';
    }
    else
    {
        cursor[0] = '*';
    }

    // 首行：1m 对应计数
    if (s_kkf_kemu3_item == 0u)
    {
        ips200_show_string(0, 2 * 15, cursor);
    }
    else
    {
        ips200_show_string(0, 2 * 15, " ");
    }
    ips200_show_string(16, 2 * 15, "CNT_1M:");
    ips200_show_int(16 * 8, 2 * 15, (int32)g_ttop_kemu3_counts_per_meter, 6);

    // 后八行：段号 + 类型 + 距离 + 速度
    for (i = 0u; i < (uint8)TTOP_KEMU3_SEG_COUNT; i++)
    {
        uint16 y = (uint16)((3u + (uint16)i) * 15u);
        uint8 row_item = (uint8)(i + 1u);

        if (s_kkf_kemu3_item == row_item)
        {
            ips200_show_string(0, y, cursor);
        }
        else
        {
            ips200_show_string(0, y, " ");
        }

        ips200_show_int(16 * 1, y, (int32)i, 1);
        if (g_ttop_kemu3_seg_table[i].seg_type == TTOP_KEMU3_SEG_TURN)
        {
            ips200_show_string(16 * 2, y, "T");
            ips200_show_int(16 * 3, y, (int32)g_ttop_kemu3_seg_table[i].turn_prog_id, 1);
        }
        else
        {
            ips200_show_string(16 * 2, y, "S");
        }

        ips200_show_float(16 * 4, y, g_ttop_kemu3_seg_table[i].meters, 5, 2);
        ips200_show_int(16 * 9, y, (int32)g_ttop_kemu3_seg_table[i].speed_counts_5ms, 5);
    }

    kkf_draw_signature_bottom();
}

static void kkf_slope_dbg_draw(void)
{
    const roll_slope_speed_cfg_t *cfg = roll_slope_speed_get_cfg();
    char cursor[3] = ">";
    if (s_kkf_slope_state == KKF_SLOPE_STATE_EDIT) cursor[0] = '-';

    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "SLOPE_DBG");

    ips200_show_string(16, 2 * 15, "EN:");
    ips200_show_string(16 * 7, 2 * 15, roll_slope_speed_get_enabled() ? "ON" : "OFF");

    ips200_show_string(16, 3 * 15, "DIST:");
    ips200_show_float(16 * 7, 3 * 15, cfg->pre_up_decel_m, 5, 1);

    ips200_show_string(16, 4 * 15, "UP_ST:");
    ips200_show_float(16 * 7, 4 * 15, cfg->up_start_deg, 5, 2);

    ips200_show_string(16, 5 * 15, "UP_ED:");
    ips200_show_float(16 * 7, 5 * 15, cfg->up_end_deg, 5, 2);

    ips200_show_string(16, 6 * 15, "DN_ST:");
    ips200_show_float(16 * 7, 6 * 15, cfg->down_start_deg, 5, 2);

    ips200_show_string(16, 7 * 15, "DN_ED:");
    ips200_show_float(16 * 7, 7 * 15, cfg->down_end_deg, 5, 2);

    ips200_show_string(16, 8 * 15, "UP_D:");
    ips200_show_int(16 * 7, 8 * 15, (int32)cfg->up_delta_counts, 6);

    ips200_show_string(16, 9 * 15, "PUP_D:");
    ips200_show_int(16 * 7, 9 * 15, (int32)cfg->pre_up_decel_counts, 6);

    ips200_show_string(16, 10 * 15, "CR_D:");
    ips200_show_int(16 * 7, 10 * 15, (int32)cfg->crest_delta_counts, 6);

    ips200_show_string(16, 11 * 15, "FL_D:");
    ips200_show_int(16 * 7, 11 * 15, (int32)cfg->flat_delta_counts, 6);

    ips200_show_string(0, (uint16)((1u + (uint16)s_kkf_slope_item) * 15u), cursor);

    kkf_draw_signature_bottom();
}

static void kkf_con_dbg_draw(void)
{
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "CON_DBG");

    ips200_show_string(16, 2 * 15, "CH1:");
    ips200_show_int(16 * 6, 2 * 15, (int32)uart_receiver.channel[0], 5);

    ips200_show_string(16, 3 * 15, "CH2:");
    ips200_show_int(16 * 6, 3 * 15, (int32)uart_receiver.channel[1], 5);

    ips200_show_string(16, 4 * 15, "CH3:");
    ips200_show_int(16 * 6, 4 * 15, (int32)uart_receiver.channel[2], 5);

    ips200_show_string(16, 5 * 15, "CH4:");
    ips200_show_int(16 * 6, 5 * 15, (int32)uart_receiver.channel[3], 5);

    ips200_show_string(16, 6 * 15, "CH5:");
    ips200_show_int(16 * 6, 6 * 15, (int32)uart_receiver.channel[4], 5);

    ips200_show_string(16, 7 * 15, "CH6:");
    ips200_show_int(16 * 6, 7 * 15, (int32)uart_receiver.channel[5], 5);

    ips200_show_string(16, 8 * 15, "STATE:");
    ips200_show_string(16 * 8, 8 * 15, (uart_receiver.state == 1u) ? "OK" : "FAIL");

    ips200_show_string(16, 9 * 15, "RX_ISR:");
    ips200_show_int(16 * 8, 9 * 15, (int32)g_uart1_rx_isr_count, 8);

    ips200_show_string(16, 10 * 15, "F25:");
    ips200_show_int(16 * 6, 10 * 15, (int32)uart_receiver_frame25_count, 8);

    ips200_show_string(16, 11 * 15, "FOK:");
    ips200_show_int(16 * 6, 11 * 15, (int32)uart_receiver_frame_ok_count, 8);

    ips200_show_string(16, 12 * 15, "ABN:");
    ips200_show_int(16 * 6, 12 * 15, (int32)uart_receiver_abnormal_count, 8);

    kkf_draw_signature_bottom();
}

static void kkf_pid_mode_select_draw(void)
{
    // PID_MODE 选择页：KEY1/KEY3 移动；KEY2 确认；KEY4 返回
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "PID_MODE");

    ips200_show_string(16, 2 * 15, "CASCADE");
    ips200_show_string(16, 3 * 15, "CASCADE2");
    ips200_show_string(16, 4 * 15, "CASCADE3");
    ips200_show_string(16, 5 * 15, "CASCADE4");
    ips200_show_string(16, 6 * 15, "CASCADE5");
    ips200_show_string(16, 7 * 15, "CASCADE6");
    ips200_show_string(16, 8 * 15, "CASCADE7");
    ips200_show_string(16, 9 * 15, "CASCADE8");
    ips200_show_string(16, 10 * 15, "CASCADE9");
    ips200_show_string(16, 11 * 15, "CASCADE10");
    ips200_show_string(16, 12 * 15, "CASCADE11");
    ips200_show_string(16, 13 * 15, "CASCADE12");
    ips200_show_string(16, 14 * 15, "ANGLE");
    ips200_show_string(16, 15 * 15, "DAMPED2");
    ips200_show_string(16, 16 * 15, "DAMPED3");
    ips200_show_string(16, 17 * 15, "DAMPED4");

    // 光标：从第 2 行开始
    ips200_show_string(0, (uint16)((1u + (uint16)s_kkf_pid_mode_item) * 15u), "> ");

    kkf_draw_signature_bottom();
}

static void kkf_eight_turn_draw(void)
{
    // 界面1_1_3：EIGHT_TURN（只显示，不做修改）
    // 期望显示方式：直接显示 0~11 行
    // 55.0    20.0
    //  0.0    20.0
    //  ...
    //
    // 说明：这里每行显示两列数值：
    // - 左列：target_rel_deg（偏移量，deg）
    // - 右列：rate_dps（限速，deg/s）
    uint16 i;
    char cursor[3] = ">";
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "EIGHT_TURN");

    // 光标符号：选择/编辑偏移/编辑限速
    if (s_kkf_eight_turn_state == KKF_EIGHT_TURN_SEL)
    {
        cursor[0] = '>';
    }
    else if (s_kkf_eight_turn_state == KKF_EIGHT_TURN_EDIT_DEG)
    {
        cursor[0] = '-';
    }
    else
    {
        cursor[0] = '*';
    }

    volatile ttop_eight2_seg_t *prog = kkf_eight2_prog_ptr((ttop_eight2_prog_id_t)s_kkf_eight2_prog_id);
    uint8 dec_places = (s_kkf_eight2_prog_id == TTOP_EIGHT2_PROG_EX1) ? 2u : 1u;
    uint8 width      = (s_kkf_eight2_prog_id == TTOP_EIGHT2_PROG_EX1) ? 7u : 6u;

    for (i = 0u; i < (uint16)TTOP_EIGHT2_PROG_LEN; i++)
    {
        // 从第 2 行开始显示 12 行
        uint16 y = (uint16)((2u + i) * 15u);

        // 选中行显示光标
        if (i == (uint16)s_kkf_eight_turn_item)
        {
            ips200_show_string(0, y, cursor);
        }
        else
        {
            ips200_show_string(0, y, " ");
        }

        ips200_show_float(16 * 1, y, prog[i].target_rel_deg, width, dec_places);
        ips200_show_float(16 * 6, y, prog[i].rate_dps, width, dec_places);
    }

    kkf_draw_signature_bottom();
}

static void kkf_eight2_prog_select_draw(void)
{
    // 段表选择页：
    // > MAIN
    //   EX1
    //   EX2
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "E8_PROG SEL");

    ips200_show_string(16, 2 * 15, "MAIN");
    ips200_show_string(16, 3 * 15, "EX1");
    ips200_show_string(16, 4 * 15, "EX2");
    ips200_show_string(16, 5 * 15, "EX3");
    ips200_show_string(16, 6 * 15, "EX4");
    ips200_show_string(16, 7 * 15, "EX5");
    ips200_show_string(16, 8 * 15, "EX6");
    ips200_show_string(16, 9 * 15, "EX7");

    ips200_show_string(0, (uint16)((1u + (uint16)s_kkf_eight2_prog_item) * 15u), "> ");

    kkf_draw_signature_bottom();
}

// GPS 调试空菜单（5 项）
static void kkf_gps_debug_draw(void)
{
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");

    ips200_show_string(16, 1 * 15, "SAVE_PT");
    ips200_show_string(16, 2 * 15, "SHOW_ROUTE");
    ips200_show_string(16, 3 * 15, "SHOW_HEADS");
    ips200_show_string(16, 4 * 15, "CIRCLE");
    ips200_show_string(16, 5 * 15, "CLEAR");

    ips200_show_string(0, (uint16)((uint16)s_kkf_gps_item * 15u), "> ");

    kkf_draw_signature_bottom();
}

// GPS 单点绕圈调试页
static void kkf_gps_circle_draw(void)
{
    const gps_nav_circle_status_t* st = gps_nav_circle_get_status();

    ips200_show_string(0, 0 * 15, "GPS_CIRCLE");

    ips200_show_string(16, 1 * 15, "SAVE_CENTER");
    ips200_show_string(16, 2 * 15, "START");
    ips200_show_string(16, 3 * 15, "STOP");
    if (st->direction == 1)
        ips200_show_string(16, 4 * 15, "DIR: CW ");
    else
        ips200_show_string(16, 4 * 15, "DIR: CCW");
    ips200_show_string(16, 5 * 15, "BACK");

    ips200_show_string(0, (uint16)((uint16)s_kkf_gps_circle_item * 15u), "> ");

    ips200_show_string(0, 7 * 15, "CEN:");
    ips200_show_uint(40, 7 * 15, st->center_valid, 1);
    ips200_show_string(56, 7 * 15, "RUN:");
    ips200_show_uint(96, 7 * 15, st->started, 1);
    ips200_show_string(112, 7 * 15, "CAL:");
    ips200_show_uint(152, 7 * 15, st->calib_done, 1);

    ips200_show_string(0, 8 * 15, "DST:");
    ips200_show_float(40, 8 * 15, st->dist_to_center_m, 5, 2);

    ips200_show_string(0, 9 * 15, "TAN:");
    ips200_show_float(40, 9 * 15, st->tangent_heading_deg, 5, 1);

    ips200_show_string(0, 10 * 15, "IMU:");
    ips200_show_float(40, 10 * 15, st->imu_target_deg, 5, 1);

    kkf_draw_signature_bottom();
}

static void kkf_mode_select_draw(void)
{
    // 界面1_1_2：
    // 扣扣飞车
    // > MODE 1
    //   MODE 2
    //   MODE 3
    //   MODE 4
    //   MODE 5
    //   MODE 6
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "MODE 1 200");
    ips200_show_string(16, 2 * 15, "MODE 2 SPCHD");
    ips200_show_string(16, 3 * 15, "MODE 3 55");
    ips200_show_string(16, 4 * 15, "MODE 4 45");
    ips200_show_string(16, 5 * 15, "MODE 5 200+T");
    ips200_show_string(16, 6 * 15, "MODE 6 8-TURN");
    ips200_show_string(16, 7 * 15, "MODE 7 K3");
    ips200_show_string(16, 8 * 15, "MODE 8");
    ips200_show_string(16, 9 * 15, "MODE 9 INS-REC");
    ips200_show_string(16, 10 * 15, "MODE10 INS-TRK");
    ips200_show_string(16, 11 * 15, "MODE11");
    ips200_show_string(16, 12 * 15, "MODE12");
    ips200_show_string(0, s_kkf_mode_item * 15, "> ");

    kkf_draw_signature_bottom();
}

// =========================
// PID 参数编辑页（1_1_4 ~ 1_1_8）
// =========================

typedef enum
{
    KKF_PID_FIELD_AHRS_KP = 1u,
    KKF_PID_FIELD_YAW_KP,
    KKF_PID_FIELD_YAW_KI,
    KKF_PID_FIELD_YAW_KD,
    KKF_PID_FIELD_ANGLE_KP,
    KKF_PID_FIELD_ANGLE_KI,
    KKF_PID_FIELD_ANGLE_KD,
    KKF_PID_FIELD_ANGLE_MX_ABS,
    KKF_PID_FIELD_ANGLE_IMX_ABS,
    KKF_PID_FIELD_RATE_KP,
    KKF_PID_FIELD_RATE_KI,
    KKF_PID_FIELD_RATE_KD,
    KKF_PID_FIELD_DAMP_K,
    KKF_PID_FIELD_DAMP_MAX,
    KKF_PID_FIELD_MOTOR_TGT,
    KKF_PID_FIELD_IMU_FILTER_ALPHA,
    KKF_PID_FIELD_COUNT = KKF_PID_FIELD_IMU_FILTER_ALPHA,
} kkf_pid_field_t;

typedef enum
{
    KKF_PID_STATE_SEL = 0u,   // '>' 选择字段
    KKF_PID_STATE_EDIT = 1u,  // '-' 编辑当前字段
} kkf_pid_state_t;

static volatile kkf_pid_state_t s_kkf_pid_state = KKF_PID_STATE_SEL;

static inline pid_mode_t kkf_page_to_pid_mode(ui_page_t page)
{
    switch (page)
    {
        case UI_PAGE_KKF_PID_CASCADE: return PID_MODE_CASCADE;
        case UI_PAGE_KKF_PID_CASCADE2: return PID_MODE_CASCADE_EX1;
        case UI_PAGE_KKF_PID_CASCADE3: return PID_MODE_CASCADE_EX2;
        case UI_PAGE_KKF_PID_CASCADE4: return PID_MODE_CASCADE_EX3;
        case UI_PAGE_KKF_PID_CASCADE5: return PID_MODE_CASCADE_EX4;
        case UI_PAGE_KKF_PID_CASCADE6: return PID_MODE_CASCADE_EX5;
        case UI_PAGE_KKF_PID_CASCADE7: return PID_MODE_CASCADE_EX6;
        case UI_PAGE_KKF_PID_CASCADE8: return PID_MODE_CASCADE_EX7;
        case UI_PAGE_KKF_PID_CASCADE9: return PID_MODE_CASCADE_EX8;
        case UI_PAGE_KKF_PID_CASCADE10: return PID_MODE_CASCADE_EX9;
        case UI_PAGE_KKF_PID_CASCADE11: return PID_MODE_CASCADE_EX10;
        case UI_PAGE_KKF_PID_CASCADE12: return PID_MODE_CASCADE_EX11;
        case UI_PAGE_KKF_PID_ANGLE: return PID_MODE_ANGLE;
        case UI_PAGE_KKF_PID_DAMPED_TWO: return PID_MODE_DAMPED_TWO;
        case UI_PAGE_KKF_PID_DAMPED_THREE: return PID_MODE_DAMPED_THREE;
        case UI_PAGE_KKF_PID_DAMPED_FOUR: return PID_MODE_DAMPED_FOUR;
        default: return PID_MODE_CASCADE;
    }
}

static void kkf_pid_draw_for_mode(pid_mode_t mode)
{
    // 显示字段：
    // 1 ahrs_kp
    // 2 yaw.kp
    // 3 yaw.ki
    // 4 angle.kp
    // 5 angle.ki
    // 6 angle.kd
    // 7 angle |out|max（同时作用于 out_min/out_max）
    // 8 angle |i|max（同时作用于 i_min/i_max）
    // 9 rate.kp
    // 10 rate.ki
    // 11 rate.kd
    // 12 damp_k
    // 13 damp_max
    // 14 motor_target_counts_5ms
    // 15 imu_filter_alpha
    const pid_mode_cfg_t *cfg = &s_pid_mode_cfg[(uint16)mode];

    char cursor[3] = ">";
    if (s_kkf_pid_state == KKF_PID_STATE_EDIT) cursor[0] = '-';

    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");

    // 顶部显示当前 mode 名
    if (mode == PID_MODE_CASCADE) ips200_show_string(16, 1 * 15, "PID_CASCADE");
    else if (mode == PID_MODE_CASCADE_EX1) ips200_show_string(16, 1 * 15, "PID_CASCADE2");
    else if (mode == PID_MODE_CASCADE_EX2) ips200_show_string(16, 1 * 15, "PID_CASCADE3");
    else if (mode == PID_MODE_CASCADE_EX3) ips200_show_string(16, 1 * 15, "PID_CASCADE4");
    else if (mode == PID_MODE_CASCADE_EX4) ips200_show_string(16, 1 * 15, "PID_CASCADE5");
    else if (mode == PID_MODE_CASCADE_EX5) ips200_show_string(16, 1 * 15, "PID_CASCADE6");
    else if (mode == PID_MODE_CASCADE_EX6) ips200_show_string(16, 1 * 15, "PID_CASCADE7");
    else if (mode == PID_MODE_CASCADE_EX7) ips200_show_string(16, 1 * 15, "PID_CASCADE8");
    else if (mode == PID_MODE_CASCADE_EX8) ips200_show_string(16, 1 * 15, "PID_CASCADE9");
    else if (mode == PID_MODE_CASCADE_EX9) ips200_show_string(16, 1 * 15, "PID_CASCADE10");
    else if (mode == PID_MODE_CASCADE_EX10) ips200_show_string(16, 1 * 15, "PID_CASCADE11");
    else if (mode == PID_MODE_CASCADE_EX11) ips200_show_string(16, 1 * 15, "PID_CASCADE12");
    else if (mode == PID_MODE_ANGLE) ips200_show_string(16, 1 * 15, "PID_ANGLE");
    else if (mode == PID_MODE_DAMPED_TWO) ips200_show_string(16, 1 * 15, "PID_DAMPED2");
    else if (mode == PID_MODE_DAMPED_THREE) ips200_show_string(16, 1 * 15, "PID_DAMPED3");
    else ips200_show_string(16, 1 * 15, "PID_DAMPED4");

    // 每行: label + value
    ips200_show_string(16, 2 * 15, "AHRS_KP:");
    ips200_show_float(16 * 9, 2 * 15, cfg->ahrs_kp, 5, 2);

    ips200_show_string(16, 3 * 15, "YAW_KP :");
    ips200_show_float(16 * 9, 3 * 15, cfg->yaw.kp, 5, 2);

    ips200_show_string(16, 4 * 15, "YAW_KI :");
    ips200_show_float(16 * 9, 4 * 15, cfg->yaw.ki, 5, 2);

    ips200_show_string(16, 5 * 15, "YAW_KD :");
    ips200_show_float(16 * 9, 5 * 15, cfg->yaw.kd * 10000.0f, 5, 2);

    ips200_show_string(16, 6 * 15, "ANG_KP :");
    ips200_show_float(16 * 9, 6 * 15, cfg->angle.kp, 5, 2);
    
    ips200_show_string(16, 7 * 15, "ANG_KI :");
    ips200_show_int(16 * 9, 7 * 15, (int32)cfg->angle.ki, 6);

    ips200_show_string(16, 8 * 15, "ANG_KD :");
    ips200_show_float(16 * 9, 8 * 15, cfg->angle.kd, 5, 2);

    ips200_show_string(16, 9 * 15, "A_MX   :");
    ips200_show_float(16 * 9, 9 * 15, fabsf(cfg->angle.out_max), 5, 2);

    ips200_show_string(16, 10 * 15, "A_IMX  :");
    ips200_show_float(16 * 9, 10 * 15, fabsf(cfg->angle.i_max), 5, 2);

    ips200_show_string(16, 11 * 15, "RATE_KP:");
    ips200_show_float(16 * 9, 11 * 15, cfg->rate.kp, 5, 2);

    ips200_show_string(16, 12 * 15, "RATE_KI:");
    ips200_show_float(16 * 9, 12 * 15, cfg->rate.ki, 5, 2);

    ips200_show_string(16, 13 * 15, "RATE_KD:");
    ips200_show_float(16 * 9, 13 * 15, cfg->rate.kd, 5, 2);

    ips200_show_string(16, 14 * 15, "DAMP_K :");
    ips200_show_float(16 * 9, 14 * 15, cfg->damp_k, 5, 2);

    ips200_show_string(16, 15 * 15, "DAMP_MX:");
    ips200_show_float(16 * 9, 15 * 15, cfg->damp_max, 5, 2);

    ips200_show_string(16, 16 * 15, "MOT_TGT:");
    ips200_show_int(16 * 9, 16 * 15, cfg->motor_target_counts_5ms, 6);

    ips200_show_string(16, 17 * 15, "IMU_ALP:");
    ips200_show_float(16 * 9, 17 * 15, cfg->imu_filter_alpha, 5, 3);

    // 光标：行号 = 字段序号 + 1（因为从第2行开始显示字段）
    ips200_show_string(0, (uint16)((1u + (uint16)s_kkf_pid_item) * 15u), cursor);

    kkf_draw_signature_bottom();
}

static void kkf_pid_adjust_value(pid_mode_t mode, kkf_pid_field_t field, int8 dir)
{
    // dir: +1 / -1
    pid_mode_cfg_t *cfg = &s_pid_mode_cfg[(uint16)mode];

    // 浮点项统一步进 0.1；长按由外层按 50 次重复调用实现（等效 5.0）。
    if (field == KKF_PID_FIELD_AHRS_KP)
    {
        cfg->ahrs_kp += (dir > 0) ? 0.1f : -0.1f;
        if (cfg->ahrs_kp < 0.0f) cfg->ahrs_kp = 0.0f;
    }
    else if (field == KKF_PID_FIELD_YAW_KP)
    {
        cfg->yaw.kp += (dir > 0) ? 0.01f : -0.01f;
        if (cfg->yaw.kp < 0.0f) cfg->yaw.kp = 0.0f;
    }
    else if (field == KKF_PID_FIELD_YAW_KI)
    {
        cfg->yaw.ki += (dir > 0) ? 0.01f : -0.01f;
        if (cfg->yaw.ki < 0.0f) cfg->yaw.ki = 0.0f;
    }
    else if (field == KKF_PID_FIELD_YAW_KD)
    {
        cfg->yaw.kd += (dir > 0) ? 0.0001f : -0.0001f;
        if (cfg->yaw.kd < 0.0f) cfg->yaw.kd = 0.0f;
    }
    else if (field == KKF_PID_FIELD_ANGLE_KP)
    {
        cfg->angle.kp += (dir > 0) ? 0.01f : -0.01f;
        if (cfg->angle.kp < 0.0f) cfg->angle.kp = 0.0f;
    }
    else if (field == KKF_PID_FIELD_ANGLE_KI)
    {
        cfg->angle.ki += (dir > 0) ? 1.0f : -1.0f;
        if (cfg->angle.ki < 0.0f) cfg->angle.ki = 0.0f;
    }
    else if (field == KKF_PID_FIELD_ANGLE_KD)
    {
        cfg->angle.kd += (dir > 0) ? 0.1f : -0.1f;
        if (cfg->angle.kd < 0.0f) cfg->angle.kd = 0.0f;
    }
    else if (field == KKF_PID_FIELD_ANGLE_MX_ABS)
    {
        float v = fabsf(cfg->angle.out_max) + ((dir > 0) ? 1.0f : -1.0f);
        if (v < 0.0f) v = 0.0f;
        if (v > 2000.0f) v = 2000.0f;
        cfg->angle.out_max = v;
        cfg->angle.out_min = -v;
    }
    else if (field == KKF_PID_FIELD_ANGLE_IMX_ABS)
    {
        float v = fabsf(cfg->angle.i_max) + ((dir > 0) ? 1.0f : -1.0f);
        if (v < 0.0f) v = 0.0f;
        if (v > 2000.0f) v = 2000.0f;
        cfg->angle.i_max = v;
        cfg->angle.i_min = -v;
    }
    else if (field == KKF_PID_FIELD_RATE_KP)
    {
        cfg->rate.kp += (dir > 0) ? 0.01f : -0.01f;
        if (cfg->rate.kp < 0.0f) cfg->rate.kp = 0.0f;
    }
    else if (field == KKF_PID_FIELD_RATE_KI)
    {
        cfg->rate.ki += (dir > 0) ? 0.1f : -0.1f;
        if (cfg->rate.ki < 0.0f) cfg->rate.ki = 0.0f;
    }
    else if (field == KKF_PID_FIELD_RATE_KD)
    {
        cfg->rate.kd += (dir > 0) ? 0.1f : -0.1f;
        if (cfg->rate.kd < 0.0f) cfg->rate.kd = 0.0f;
    }
    else if (field == KKF_PID_FIELD_DAMP_K)
    {
        cfg->damp_k += (dir > 0) ? 0.1f : -0.1f;
        if (cfg->damp_k < 0.0f) cfg->damp_k = 0.0f;
    }
    else if (field == KKF_PID_FIELD_DAMP_MAX)
    {
        cfg->damp_max += (dir > 0) ? 0.1f : -0.1f;
        if (cfg->damp_max < 0.0f) cfg->damp_max = 0.0f;
    }
    else if (field == KKF_PID_FIELD_MOTOR_TGT)
    {
        int32 v = (int32)cfg->motor_target_counts_5ms + ((dir > 0) ? 1 : -1);
        if (v < 0) v = 0;
        if (v > 2000) v = 2000;
        cfg->motor_target_counts_5ms = (int16)v;
    }
    else // IMU_FILTER_ALPHA
    {
        cfg->imu_filter_alpha += (dir > 0) ? 0.001f : -0.001f;
        if (cfg->imu_filter_alpha < 0.0f) cfg->imu_filter_alpha = 0.0f;
        if (cfg->imu_filter_alpha > 1.0f) cfg->imu_filter_alpha = 1.0f;
    }
}

static void kkf_kemu1_adjust_meters(uint8 idx, int8 dir)
{
    float v;
    if (idx >= 7u) return;

    v = g_ttop_kemu1_seg_table[idx].meters;
    v += (dir > 0) ? 0.1f : -0.1f;
    if (v < 0.0f) v = 0.0f;
    if (v > 999.0f) v = 999.0f;
    g_ttop_kemu1_seg_table[idx].meters = v;

    ttop_kemu1_rebuild_targets();
}

static void kkf_kemu1_adjust_speed(uint8 idx, int8 dir)
{
    int32 v;
    if (idx >= 7u) return;

    v = (int32)g_ttop_kemu1_seg_table[idx].speed_counts_5ms + ((dir > 0) ? 1 : -1);
    if (v < 0) v = 0;
    if (v > 2000) v = 2000;
    g_ttop_kemu1_seg_table[idx].speed_counts_5ms = (short)v;
}

static void kkf_kemu1_adjust_counts_per_meter(int8 dir)
{
    int32 v = (int32)g_ttop_kemu1_counts_per_meter + ((dir > 0) ? 1 : -1);
    if (v < 1) v = 1;
    if (v > 99999) v = 99999;
    g_ttop_kemu1_counts_per_meter = (int)v;

    ttop_kemu1_rebuild_targets();
}

static void kkf_kemu3_adjust_meters(uint8 idx, int8 dir)
{
    float v;
    if (idx >= (uint8)TTOP_KEMU3_SEG_COUNT) return;

    v = g_ttop_kemu3_seg_table[idx].meters;
    v += (dir > 0) ? 0.1f : -0.1f;
    if (v < 0.0f) v = 0.0f;
    if (v > 999.0f) v = 999.0f;
    g_ttop_kemu3_seg_table[idx].meters = v;

    ttop_kemu3_rebuild_targets();
}

static void kkf_kemu3_adjust_speed(uint8 idx, int8 dir)
{
    int32 v;
    if (idx >= (uint8)TTOP_KEMU3_SEG_COUNT) return;

    v = (int32)g_ttop_kemu3_seg_table[idx].speed_counts_5ms + ((dir > 0) ? 1 : -1);
    if (v < 0) v = 0;
    if (v > 2000) v = 2000;
    g_ttop_kemu3_seg_table[idx].speed_counts_5ms = (short)v;
}

static void kkf_kemu3_adjust_counts_per_meter(int8 dir)
{
    int32 v = (int32)g_ttop_kemu3_counts_per_meter + ((dir > 0) ? 1 : -1);
    if (v < 1) v = 1;
    if (v > 99999) v = 99999;
    g_ttop_kemu3_counts_per_meter = (int)v;

    ttop_kemu3_rebuild_targets();
}

static void kkf_slope_adjust_value(kkf_slope_field_t field, int8 dir)
{
    roll_slope_speed_cfg_t cfg = *roll_slope_speed_get_cfg();

    if (field == KKF_SLOPE_FIELD_ENABLE)
    {
        roll_slope_speed_set_enabled((dir > 0) ? 1u : 0u);
        return;
    }
    else if (field == KKF_SLOPE_FIELD_DIST)
    {
        float v = cfg.pre_up_decel_m + ((dir > 0) ? 0.1f : -0.1f);
        if (v < 0.0f) v = 0.0f;
        if (v > 999.0f) v = 999.0f;
        cfg.pre_up_decel_m = v;
    }
    else if (field == KKF_SLOPE_FIELD_UP_START)
    {
        cfg.up_start_deg += (dir > 0) ? 0.1f : -0.1f;
        if (cfg.up_start_deg < 0.0f) cfg.up_start_deg = 0.0f;
        if (cfg.up_start_deg > 90.0f) cfg.up_start_deg = 90.0f;
    }
    else if (field == KKF_SLOPE_FIELD_UP_END)
    {
        cfg.up_end_deg += (dir > 0) ? 0.1f : -0.1f;
        if (cfg.up_end_deg < 0.0f) cfg.up_end_deg = 0.0f;
        if (cfg.up_end_deg > 90.0f) cfg.up_end_deg = 90.0f;
    }
    else if (field == KKF_SLOPE_FIELD_DOWN_START)
    {
        cfg.down_start_deg += (dir > 0) ? 0.1f : -0.1f;
        if (cfg.down_start_deg < 0.0f) cfg.down_start_deg = 0.0f;
        if (cfg.down_start_deg > 90.0f) cfg.down_start_deg = 90.0f;
    }
    else if (field == KKF_SLOPE_FIELD_DOWN_END)
    {
        cfg.down_end_deg += (dir > 0) ? 0.1f : -0.1f;
        if (cfg.down_end_deg < 0.0f) cfg.down_end_deg = 0.0f;
        if (cfg.down_end_deg > 90.0f) cfg.down_end_deg = 90.0f;
    }
    else if (field == KKF_SLOPE_FIELD_UP_DELTA)
    {
        int32 v = (int32)cfg.up_delta_counts + ((dir > 0) ? 1 : -1);
        if (v < -2000) v = -2000;
        if (v > 2000) v = 2000;
        cfg.up_delta_counts = (int16)v;
    }
    else if (field == KKF_SLOPE_FIELD_PRE_UP_DECEL)
    {
        int32 v = (int32)cfg.pre_up_decel_counts + ((dir > 0) ? 1 : -1);
        if (v > 2000) v = 2000;
        if (v < -2000) v = -2000;
        cfg.pre_up_decel_counts = (int16)v;
    }
    else if (field == KKF_SLOPE_FIELD_CREST_DELTA)
    {
        int32 v = (int32)cfg.crest_delta_counts + ((dir > 0) ? 1 : -1);
        if (v < -2000) v = -2000;
        if (v > 2000) v = 2000;
        cfg.crest_delta_counts = (int16)v;
    }
    else
    {
        int32 v = (int32)cfg.flat_delta_counts + ((dir > 0) ? 1 : -1);
        if (v < -2000) v = -2000;
        if (v > 2000) v = 2000;
        cfg.flat_delta_counts = (int16)v;
    }

    roll_slope_speed_set_cfg(&cfg);
}

static void kkf_motor_pid_adjust_value(kkf_motor_pid_field_t field, int8 dir)
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float duty_k = 0.0f;
    float pitch_k = 0.0f;
    float acc_k = 0.0f;
    uint8 boost_mode = 0u;

    motor_get_pid_gains(&kp, &ki, &kd);
    motor_get_boost_map_k(&duty_k, &pitch_k, &acc_k);
    boost_mode = motor_get_boost_mode();

    if (field == KKF_MOTOR_PID_FIELD_KP)
    {
        kp += (dir > 0) ? 0.01f : -0.01f;
        if (kp < 0.0f) kp = 0.0f;
    }
    else if (field == KKF_MOTOR_PID_FIELD_KI)
    {
        ki += (dir > 0) ? 0.01f : -0.01f;
        if (ki < 0.0f) ki = 0.0f;
    }
    else if (field == KKF_MOTOR_PID_FIELD_KD)
    {
        kd += (dir > 0) ? 0.01f : -0.01f;
        if (kd < 0.0f) kd = 0.0f;
    }

    else if (field == KKF_MOTOR_PID_FIELD_BT_MODE)
    {
        int32 v = (int32)boost_mode + ((dir > 0) ? 1 : -1);
        if (v < 0) v = 3;
        if (v > 3) v = 0;
        boost_mode = (uint8)v;
    }
    else if (field == KKF_MOTOR_PID_FIELD_DUTY_KD)
    {
        duty_k += (dir > 0) ? 0.01f : -0.01f;
        if (duty_k < 0.0f) duty_k = 0.0f;
    }
    else if (field == KKF_MOTOR_PID_FIELD_PITCH_KD)
    {
        pitch_k += (dir > 0) ? 0.01f : -0.01f;
        if (pitch_k < 0.0f) pitch_k = 0.0f;
    }
    else if (field == KKF_MOTOR_PID_FIELD_ACC_KD)
    {
        acc_k += (dir > 0) ? 0.01f : -0.01f;
        if (acc_k < 0.0f) acc_k = 0.0f;
    }
    else
    {
        float bmax = motor_get_pitch_speed_boost_max_counts();
        bmax += (dir > 0) ? 1.0f : -1.0f;
        if (bmax < 0.0f) bmax = 0.0f;
        motor_set_pitch_speed_boost_max_counts(bmax);
    }

    motor_set_pid_gains(kp, ki, kd);
    motor_set_boost_map_k(duty_k, pitch_k, acc_k);
    motor_set_boost_mode(boost_mode);
}

static void kkf_spch_adjust_value(kkf_spch_field_t field, int8 dir)
{
    if (field == KKF_SPCH_FIELD_FIT_MODE)
    {
        int32 mode = (int32)g_ttop_speed_change_fit_mode + ((dir > 0) ? 1 : -1);
        if (mode < 0) mode = 0;
        if (mode > 2) mode = 2;
        g_ttop_speed_change_fit_mode = (uint8)mode;
    }
    else if (field == KKF_SPCH_FIELD_LPF_ALPHA)
    {
        g_ttop_speed_change_lpf_alpha += (dir > 0) ? 0.01f : -0.01f;
        if (g_ttop_speed_change_lpf_alpha < 0.0f) g_ttop_speed_change_lpf_alpha = 0.0f;
        if (g_ttop_speed_change_lpf_alpha > 1.0f) g_ttop_speed_change_lpf_alpha = 1.0f;
    }
    else if (field == KKF_SPCH_FIELD_ANG_LPF)
    {
        g_pitch_angle_lpf_alpha += (dir > 0) ? 0.001f : -0.001f;
        if (g_pitch_angle_lpf_alpha < 0.0f) g_pitch_angle_lpf_alpha = 0.0f;
        if (g_pitch_angle_lpf_alpha > 0.999f) g_pitch_angle_lpf_alpha = 0.999f;
    }
    else if (field == KKF_SPCH_FIELD_SERVO_ZERO)
    {
        g_servo_zero_offset_deg += (dir > 0) ? 0.1f : -0.1f;
        if (g_servo_zero_offset_deg < -200.0f) g_servo_zero_offset_deg = -200.0f;
        if (g_servo_zero_offset_deg > 200.0f) g_servo_zero_offset_deg = 200.0f;
    }
    else
    {
        g_pitch_zero_offset_deg += (dir > 0) ? 0.01f : -0.01f;
        if (g_pitch_zero_offset_deg < -45.0f) g_pitch_zero_offset_deg = -45.0f;
        if (g_pitch_zero_offset_deg > 45.0f) g_pitch_zero_offset_deg = 45.0f;
    }
}

// PID 编辑页长按加速：每次长按事件按短按步进执行 100 次
#define KKF_PID_LONG_PRESS_EXTRA_STEPS   (100u)
// KEMU1 编辑页长按加速：每次长按事件按短按步进执行 50 次
#define KKF_KEMU1_LONG_PRESS_EXTRA_STEPS (50u)
// 八字段表编辑页长按加速：每次长按事件按短按步进执行 50 次
#define KKF_EIGHT_TURN_LONG_PRESS_EXTRA_STEPS (50u)
// SPCH 编辑页长按加速：每次长按事件按短按步进执行 50 次
#define KKF_SPCH_LONG_PRESS_EXTRA_STEPS (50u)

// // 旧版菜单显示控制/状态信息
// volatile uint8 menu_deep=1;     
// volatile uint8 sec_menu_item=1; 
// volatile uint8 main_menu_item=1;
 volatile uint8 menu_show_flag=1;
// volatile uint8 data_change_flag=0;

float ips_show_pid_out;
uint16 count_pid_main;
int16 show_duty;

float show_roll_deg;
float show_pitch_deg;
float show_yaw_deg;
float show_pitch_rate_dps_f;

extern volatile int16 motor_speed_raw_counts;

// IPS200 的 16x16 字体在驱动里标注为“暂不支持”，因此这里通过：
// 1) 强制使用 8x16 字体；2) 使用 16 的行距
// 来实现“比原来(15行距)更大一号/更清晰”的显示。
#define KKF_LINE_H      (16u)

static void kkf_ui_prepare_font(void)
{
    ips200_set_font(IPS200_8X16_FONT);
}

void Menu_StartAll(void)
{
    // Start 时执行一次遥控零点重置
    ttop_rc_control_zero_reset_yaw_center_by_ch0();

    funtion_ready_flag = 1u;
    ahrs_yaw_zero_request_set();

    /* [liangan] MODE3: START 时启动自动扫目标 */
    if (funtion_kemu == 3u) liangan_auto_sweep_start();

    ttop_eight2_reset_now();
    eight_flag = 0u;
    eight_count = 0u;
    roll_slope_speed_start();
    ttop_kemu3_start();
    if (s_kkf_gps_nav_enable != 0u)
    {
        gps_nav_start();
    }
    else
    {
        gps_nav_stop();
    }

    // MODE3: START 时锁存配置开关，并执行对应 start 动作
    if (funtion_kemu == 3u)
    {
        s_mode3_run_yaw_only = s_mode3_cfg_yaw_only;
        s_mode3_run_record = s_mode3_cfg_record;
        s_mode3_run_replay = s_mode3_cfg_replay;

        if (s_mode3_run_replay)
        {
            (void)ttop_rc_yaw_record_load_from_flash(s_mode3_cfg_flash_slot);
        }

        if (s_mode3_run_record)
        {
            ttop_rc_yaw_record_start();
        }
        if (s_mode3_run_replay)
        {
            ttop_rc_yaw_replay_start();
        }
    }
    else if (funtion_kemu == 7u)
    {
        // KM3 USE_REC=2: 录制模式 — 全程录到 slot2
        if (g_kemu3_use_recorded_turn == 2u)
        {
            ttop_kemu3_record_start(2u);
        }
        // KM3 USE_REC=3: 回放模式 — 先加载 slot2，再回放
        else if (g_kemu3_use_recorded_turn == 3u)
        {
            (void)ttop_kemu3_record_load_from_flash(2u);
            ttop_kemu3_replay_start(2u);
        }
    }
    else if (funtion_kemu == 11u)
    {
        if (s_kemu3_cfg_record)
        {
            ttop_kemu3_record_start(s_kemu3_cfg_flash_slot);
        }
        if (s_kemu3_cfg_replay)
        {
            ttop_kemu3_replay_start(s_kemu3_cfg_flash_slot);
        }
    }
}

void Menu_ResetAll(void)
{
    
    encoder_clear_demo5_count_total();
    ahrs_yaw_zero_request_set();
    PID_CLEAR_INTEGRATORS();
    ttop_eight2_reset_now();
    small_driver_set_duty(0, 0);
    servo_zero();
    ins_dr_init();
    ins_path_follow_init();
    ttop_kemu1_init();
    gps_nav_init();
    eight_flag = 0u;
    eight_count = 0u;
    funtion_ready_flag = 0u;

    /* [liangan] RESET 时停止自动扫目标 */
    liangan_auto_sweep_reset();

    roll_slope_speed_reset();
    ttop_kemu3_reset();

    // MODE3: STOP 时执行对应 stop 动作，并清运行开关
    if (s_mode3_run_record)
    {
        ttop_rc_yaw_record_stop();
    }
    if (s_mode3_run_replay)
    {
        ttop_rc_yaw_replay_stop();
    }
    ttop_kemu3_record_stop();
    ttop_kemu3_replay_stop();
    s_mode3_run_yaw_only = 0u;
    s_mode3_run_record = 0u;
    s_mode3_run_replay = 0u;
    
}

uint8 menu_mode3_run_yaw_only_enabled(void)
{
    return s_mode3_run_yaw_only;
}

uint8 menu_mode3_run_record_enabled(void)
{
    return s_mode3_run_record;
}

uint8 menu_mode3_run_replay_enabled(void)
{
    return s_mode3_run_replay;
}

uint8 menu_kemu3_record_enabled(void)
{
    return s_kemu3_cfg_record;
}

uint8 menu_kemu3_replay_enabled(void)
{
    return s_kemu3_cfg_replay;
}

uint8 menu_kemu3_flash_slot(void)
{
    return s_kemu3_cfg_flash_slot;
}

uint8 menu_mode3_flash_slot(void)
{
    return s_mode3_cfg_flash_slot;
}

void User_Interface(void)//UI显示
{

    if(menu_show_flag==1)
    {
        kkf_ui_prepare_font();
        if (s_ui_page == UI_PAGE_KKF_MAIN)
        {
            kkf_main_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_PARAM)
        {
            kkf_param_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_MODE_SELECT)
        {
            kkf_mode_select_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_EIGHT2_PROG_SELECT)
        {
            kkf_eight2_prog_select_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_PID_MODE_SELECT)
        {
            kkf_pid_mode_select_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_GPS_DEBUG)
        {
            kkf_gps_debug_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_CON_DBG)
        {
            kkf_con_dbg_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_DYN_PID)
        {
            kkf_dyn_pid_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_SPCH_DBG)
        {
            kkf_spch_dbg_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_KEY_DBG)
        {
            kkf_key_dbg_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_KEMU1_DBG)
        {
            kkf_kemu1_dbg_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_KEMU3_DBG)
        {
            kkf_kemu3_dbg_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_KEMU3_PATH)
        {
            kkf_kemu3_path_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_SLOPE_DBG)
        {
            kkf_slope_dbg_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_MOTOR_PID)
        {
            kkf_motor_pid_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_RC_YAW_CFG)
        {
            kkf_rc_yaw_cfg_draw();
        }

        else if (s_ui_page == UI_PAGE_KKF_GPS_ROUTE)
        {
            // 专门界面：绘制已保存路线（点/线）
            // 使用 gps_display 提供的绘制函数
//            ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
            ttop_gps_display_route();//            kkf_draw_signature_bottom();
        }
        else if (s_ui_page == UI_PAGE_KKF_GPS_HEADINGS)
        {
            // 专门界面：计算并显示多段航向角
            ttop_gps_calc_and_show_headings();
        }
        else if (s_ui_page == UI_PAGE_KKF_GPS_CIRCLE)
        {
            kkf_gps_circle_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_INS_PATH_VIEW)
        {
            kkf_ins_path_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_GPS_STORAGE)
        {
            gps_storage_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_MOTOR_DUTY)
        {
            kkf_motor_duty_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_LG_DBG)
        {
            kkf_lg_dbg_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_PID_CASCADE ||
             s_ui_page == UI_PAGE_KKF_PID_CASCADE2 ||
             s_ui_page == UI_PAGE_KKF_PID_CASCADE3 ||
             s_ui_page == UI_PAGE_KKF_PID_CASCADE4 ||
             s_ui_page == UI_PAGE_KKF_PID_CASCADE5 ||
             s_ui_page == UI_PAGE_KKF_PID_CASCADE6 ||
             s_ui_page == UI_PAGE_KKF_PID_CASCADE7 ||
             s_ui_page == UI_PAGE_KKF_PID_CASCADE8 ||
             s_ui_page == UI_PAGE_KKF_PID_CASCADE9 ||
             s_ui_page == UI_PAGE_KKF_PID_CASCADE10 ||
             s_ui_page == UI_PAGE_KKF_PID_CASCADE11 ||
             s_ui_page == UI_PAGE_KKF_PID_CASCADE12 ||
             s_ui_page == UI_PAGE_KKF_PID_ANGLE ||
             s_ui_page == UI_PAGE_KKF_PID_DAMPED_TWO ||
             s_ui_page == UI_PAGE_KKF_PID_DAMPED_THREE ||
             s_ui_page == UI_PAGE_KKF_PID_DAMPED_FOUR)
        {
            kkf_pid_draw_for_mode(kkf_page_to_pid_mode(s_ui_page));
        }
        else
        {
            kkf_eight_turn_draw();
        }
    }
    else
    {
        return;
    }
}

void Menu_Select(void)
{
    uint8 key = 0;
    static uint8 last_key = 0; // 上一次的“事件键值”

    // 组合键：KEY2(确认) + KEY4(返回) 同时按下 -> 切换屏幕显示开关
    // 用锁存避免长按导致反复翻转：必须先松开组合键才能再次触发。
    static uint8 k24_latched = 0u;

    // 组合键：KEY1(上) + KEY3(下) 同时按下 -> 保存参数到 Flash
    // 用锁存避免长按导致反复保存：必须先松开组合键才能再次触发。
    static uint8 k13_latched = 0u;

    // Key 模块现在只提供稳定按下状态 Key_Read(i)。
    // 这里用“边沿检测”把稳定状态转换为单次触发事件，保持原菜单逻辑不变。
    static uint8 k1_last = 0u;
    static uint8 k2_last = 0u;
    static uint8 k3_last = 0u;
    static uint8 k4_last = 0u;

    uint8 k1_now = Key_Read(KEY_ID_1);
    uint8 k2_now = Key_Read(KEY_ID_2);
    uint8 k3_now = Key_Read(KEY_ID_3);
    uint8 k4_now = Key_Read(KEY_ID_4);

    // 检测组合键（稳定态）：先于单键事件处理，避免冲突。
    if (k2_now && k4_now)
    {
        if (!k24_latched)
        {
            k24_latched = 1u;
            menu_show_flag = (uint8)(!menu_show_flag);
            ips200_clear();
        }
        // 组合键按下时，不再生成单键事件
        k1_last = k1_now;
        k2_last = k2_now;
        k3_last = k3_now;
        k4_last = k4_now;
        last_key = 0u;
        return;
    }
    else
    {
        // 组合键已松开，允许下次触发
        k24_latched = 0u;
    }

    if (k1_now && k3_now)
    {
        if (!k13_latched)
        {
            uint8 ok1;
            uint8 ok3;
            uint8 ok4;
            uint8 ok5;
            k13_latched = 1u;

            uint8 ok2;
            ok1 = flash_storage_save_pid_mode_cfg();
            ok3 = flash_storage_save_kemu_all();
            flash_storage_save_ins_path();
            flash_storage_save_kemu_all();
            flash_storage_save_slope_cfg();
            ok4 = flash_storage_save_misc_cfg();
            ok5 = (s_mode3_cfg_record != 0u) ?
                ttop_rc_yaw_record_save_to_flash(s_mode3_cfg_flash_slot) : 1u;
            // KM3 录制轨迹（slot2）：长按存储
            ok2 = (ttop_kemu3_get_record_count(2u) > 0u) ?
                ttop_kemu3_record_save_to_flash(2u) : 1u;
            // 球-梁 RC 录制数据 → Flash slot 2
            uint8 ok_lg = (liangan_rc_record_get_count() > 0u) ?
                liangan_rc_record_save_to_flash(2u) : 1u;
            
            // 轻量提示：显示 SAVE OK/FAIL
            if (ok1 && ok2 && ok3 && ok4 && ok5 && ok_lg)
            {
                ips200_show_string(16 * 8, 0, "SAVE OK");
            }
            else
            {
                ips200_show_string(16 * 8, 0, "SAVE FAIL");
                // 显示单项结果，便于定位是哪一类保存失败
                // ips200_show_int(16 * 8, 16, (int32)ok1, 1);
                // ips200_show_int(16 * 9, 16, (int32)ok2, 1);
                // ips200_show_int(16 * 10, 16, (int32)ok3, 1);
                // ips200_show_int(16 * 11, 16, (int32)ok4, 1);
            }
            ttop_funtion_mode_once_apply();
        }

        // 组合键按下时，不再生成单键事件
        k1_last = k1_now;
        k2_last = k2_now;
        k3_last = k3_now;
        k4_last = k4_now;
        last_key = 0u;
        return;
    }
    else
    {
        // 组合键已松开，允许下次触发
        k13_latched = 0u;
    }

    if (k1_now && !k1_last) key = KEY1_PRES;
    else if (k2_now && !k2_last) key = KEY2_PRES;
    else if (k3_now && !k3_last) key = KEY3_PRES;
    else if (k4_now && !k4_last) key = KEY4_PRES;

    k1_last = k1_now;
    k2_last = k2_now;
    k3_last = k3_now;
    k4_last = k4_now;

    // 长按事件需要脱离短按边沿门控，否则会出现“下次按键才生效”的滞后。
    uint8 key_rise = (uint8)((key != 0u) && (last_key == 0u));
    uint8 motor_duty_page = (uint8)(s_ui_page == UI_PAGE_KKF_MOTOR_DUTY);
    uint8 pid_edit_page =
        (uint8)(s_ui_page == UI_PAGE_KKF_PID_CASCADE ||
                s_ui_page == UI_PAGE_KKF_PID_CASCADE2 ||
                s_ui_page == UI_PAGE_KKF_PID_CASCADE3 ||
                s_ui_page == UI_PAGE_KKF_PID_CASCADE4 ||
                s_ui_page == UI_PAGE_KKF_PID_CASCADE5 ||
                s_ui_page == UI_PAGE_KKF_PID_CASCADE6 ||
                s_ui_page == UI_PAGE_KKF_PID_CASCADE7 ||
                s_ui_page == UI_PAGE_KKF_PID_CASCADE8 ||
                s_ui_page == UI_PAGE_KKF_PID_CASCADE9 ||
                s_ui_page == UI_PAGE_KKF_PID_CASCADE10 ||
                s_ui_page == UI_PAGE_KKF_PID_CASCADE11 ||
                s_ui_page == UI_PAGE_KKF_PID_CASCADE12 ||
                s_ui_page == UI_PAGE_KKF_PID_ANGLE ||
                s_ui_page == UI_PAGE_KKF_PID_DAMPED_TWO ||
                s_ui_page == UI_PAGE_KKF_PID_DAMPED_THREE ||
                s_ui_page == UI_PAGE_KKF_PID_DAMPED_FOUR);
    uint8 eight_turn_edit_page =
        (uint8)(s_ui_page == UI_PAGE_KKF_EIGHT_TURN &&
                (s_kkf_eight_turn_state == KKF_EIGHT_TURN_EDIT_DEG ||
                 s_kkf_eight_turn_state == KKF_EIGHT_TURN_EDIT_DPS));
    uint8 kemu1_edit_page =
        (uint8)(s_ui_page == UI_PAGE_KKF_KEMU1_DBG &&
                (s_kkf_kemu1_state == KKF_KEMU1_EDIT_METERS ||
                 s_kkf_kemu1_state == KKF_KEMU1_EDIT_SPEED));
    uint8 kemu3_edit_page =
        (uint8)((s_ui_page == UI_PAGE_KKF_KEMU3_PATH &&
                (s_kkf_kemu3_state == KKF_KEMU3_EDIT_METERS ||
                 s_kkf_kemu3_state == KKF_KEMU3_EDIT_SPEED)) ||
                (s_ui_page == UI_PAGE_KKF_KEMU3_DBG &&
                 (s_kkf_kemu3_dbg_edit == 4u || s_kkf_kemu3_dbg_edit == 5u)));
    uint8 slope_edit_page =
        (uint8)(s_ui_page == UI_PAGE_KKF_SLOPE_DBG &&
                (s_kkf_slope_state == KKF_SLOPE_STATE_EDIT));
    uint8 motor_pid_edit_page =
        (uint8)(s_ui_page == UI_PAGE_KKF_MOTOR_PID &&
                (s_kkf_motor_pid_state == KKF_MOTOR_PID_STATE_EDIT));
    uint8 spch_edit_page =
        (uint8)(s_ui_page == UI_PAGE_KKF_SPCH_DBG &&
                (s_kkf_spch_state == KKF_SPCH_STATE_EDIT));
    uint8 lg_dbg_page =
        (uint8)(s_ui_page == UI_PAGE_KKF_LG_DBG);
    uint8 k1_long_evt = 0u;
    uint8 k3_long_evt = 0u;

    if (pid_edit_page && (s_kkf_pid_state == KKF_PID_STATE_EDIT))
    {
        k1_long_evt = Key_ReadLongPress(KEY_ID_1);
        k3_long_evt = Key_ReadLongPress(KEY_ID_3);
    }
    else if (eight_turn_edit_page)
    {
        k1_long_evt = Key_ReadLongPress(KEY_ID_1);
        k3_long_evt = Key_ReadLongPress(KEY_ID_3);
    }
    else if (kemu1_edit_page)
    {
        k1_long_evt = Key_ReadLongPress(KEY_ID_1);
        k3_long_evt = Key_ReadLongPress(KEY_ID_3);
    }
    else if (kemu3_edit_page)
    {
        k1_long_evt = Key_ReadLongPress(KEY_ID_1);
        k3_long_evt = Key_ReadLongPress(KEY_ID_3);
    }
    else if (slope_edit_page)
    {
        k1_long_evt = Key_ReadLongPress(KEY_ID_1);
        k3_long_evt = Key_ReadLongPress(KEY_ID_3);
    }
    else if (motor_pid_edit_page)
    {
        k1_long_evt = Key_ReadLongPress(KEY_ID_1);
        k3_long_evt = Key_ReadLongPress(KEY_ID_3);
    }
    else if (spch_edit_page)
    {
        k1_long_evt = Key_ReadLongPress(KEY_ID_1);
        k3_long_evt = Key_ReadLongPress(KEY_ID_3);
    }
    else if (motor_duty_page)
    {
        k1_long_evt = Key_ReadLongPress(KEY_ID_1);
        k3_long_evt = Key_ReadLongPress(KEY_ID_3);
    }
    else if (lg_dbg_page)
    {
        k1_long_evt = Key_ReadLongPress(KEY_ID_1);
        k3_long_evt = Key_ReadLongPress(KEY_ID_3);
    }

    if (key_rise || k1_long_evt || k3_long_evt ||
        (motor_duty_page && (k1_now || k3_now)))
    {
        // =========================
        // 按键语义：KEY1上 / KEY2确认 / KEY3下 / KEY4返回
        // =========================
        if (s_ui_page == UI_PAGE_KKF_MAIN)
        {
            if (KEY3_PRES == key)
            {
                if (s_kkf_main_item > 1u) s_kkf_main_item--;
                else s_kkf_main_item = 15u;
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                if (s_kkf_main_item < 15u) s_kkf_main_item++;
                else s_kkf_main_item = 1u;
                ips200_clear();
            }
            else if (KEY2_PRES == key)
            {
                // 1:PID_CONFIG 2:START 3:RESET 4:CLEAR 5:GPS_SW 6:GPS_DBG 7:STATE 8:MODE 9:yaw 10:t_yaw 11:pitch 12:SPD 13:xy 14:g_ins_path
                if (s_kkf_main_item == 1u)
                {
                    s_ui_page = UI_PAGE_KKF_PARAM;
                    s_kkf_param_item = 1u;
                    ips200_clear();
                }
                else if (s_kkf_main_item == 2u)
                {
                    // START：开始跑（使能 PID/ENC 等周期任务）
                    // START 同时执行一次 CLEAR：重置航向/八字状态，确保从干净状态起跑。
                    Menu_StartAll();
                   //motor_request_start(s_menu_moto_mode);
                    ips200_clear();
                }
                else if (s_kkf_main_item == 3u)
                {
                    // RESET：停止跑 + 同时进行与 CLEAR 类似的“状态复位”
                    Menu_ResetAll();
                    ips200_clear();
                }
                else if (s_kkf_main_item == 4u)
                {
                    // 清零：对齐原 ttop_key_task_5ms 的行为
                    ahrs_yaw_zero_request_set();
                    ttop_eight2_reset_now();
                    eight_flag = 0u;
                    eight_count = 0u;
                    ips200_clear();
                }
                else if (s_kkf_main_item == 5u)
                {
                    // GPS 导航开关：ON 时 START 启用 gps_nav；OFF 时 START 不启用
                    s_kkf_gps_nav_enable = (uint8)(!s_kkf_gps_nav_enable);
                    if (s_kkf_gps_nav_enable == 0u)
                    {
                        gps_nav_stop();
                    }
                    ips200_clear();
                }
                else if (s_kkf_main_item == 6u)
                {
                    // 进入 GPS 调试入口（原 MT_ST 插槽）
                    s_ui_page = UI_PAGE_KKF_GPS_DEBUG;
                    s_kkf_gps_item = 1u;
                    ips200_clear();
                }
                else if (s_kkf_main_item == 14u)
                {
                    // 航点查看
                    s_ui_page = UI_PAGE_KKF_INS_PATH_VIEW;
                    s_kkf_ins_path_page = 0u;
                    ips200_clear();
                }
                else if (s_kkf_main_item == 15u)
                {
                    gps_storage_start();
                    s_ui_page = UI_PAGE_KKF_GPS_STORAGE;
                    ips200_clear();
                }
                else
                {
                    // 其它项：暂不进入编辑
                    ips200_clear();

                }
            }
            else if (KEY4_PRES == key)
            {
                // 主界面按返回：保持在主界面
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_GPS_ROUTE)
        {
            // 路线显示页：按 KEY4 返回到 GPS 菜单
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_GPS_DEBUG;
                ips200_clear();
            }
            else
            {
                // 其它键暂不处理（可在此扩展缩放/翻页功能）
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_GPS_HEADINGS)
        {
            // 航向显示页：按 KEY4 返回到 GPS 菜单
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_GPS_DEBUG;
                ips200_clear();
            }
            else
            {
                // 其它键暂不处理
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_INS_PATH_VIEW)
        {
            // 航点查看页：KEY1/KEY3 翻页；KEY4 返回
            uint16 count = g_ins_path_count;
            if (count > (uint16)INS_PATH_MAX_POINTS) count = (uint16)INS_PATH_MAX_POINTS;

            uint16 rows = kkf_ui_rows_available();
            uint16 points_per_page = (rows > 2u) ? (uint16)(rows - 2u) : 1u;
            if (points_per_page == 0u) points_per_page = 1u;

            uint16 page_count = 1u;
            if (count > 0u)
            {
                page_count = (uint16)((count + points_per_page - 1u) / points_per_page);
                if (page_count == 0u) page_count = 1u;
            }

            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_MAIN;
                ips200_clear();
            }
            else if (KEY3_PRES == key)
            {
                // 上一页
                if (s_kkf_ins_path_page > 0u) s_kkf_ins_path_page--;
                else s_kkf_ins_path_page = (uint16)(page_count - 1u);
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                // 下一页
                if (s_kkf_ins_path_page + 1u < page_count) s_kkf_ins_path_page++;
                else s_kkf_ins_path_page = 0u;
                ips200_clear();
            }
            else
            {
                // KEY2：不处理
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_GPS_DEBUG)
        {
            // GPS 调试菜单：KEY1 下 / KEY3 上 / KEY2 确认 / KEY4 返回
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_MAIN;
                ips200_clear();
            }
            else if (KEY3_PRES == key)
            {
                if (s_kkf_gps_item > 1u) s_kkf_gps_item--;
                else s_kkf_gps_item = 5u;
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                if (s_kkf_gps_item < 5u) s_kkf_gps_item++;
                else s_kkf_gps_item = 1u;
                ips200_clear();
            }
            else if (KEY2_PRES == key)
            {
                if (s_kkf_gps_item == 1u)
                {
                    ttop_gps_save_point();
                    ips200_show_uint(16 * 8, (uint16)((uint16)s_kkf_gps_item * 15u), gps_saved_point_count, 3);
                }
                else if (s_kkf_gps_item == 2u)
                {
                    s_ui_page = UI_PAGE_KKF_GPS_ROUTE;
                    ips200_clear();
                }
                else if (s_kkf_gps_item == 3u)
                {
                    s_ui_page = UI_PAGE_KKF_GPS_HEADINGS;
                    ips200_clear();
                }
                else if (s_kkf_gps_item == 4u)
                {
                    // 进入单点绕圈调试页
                    s_ui_page = UI_PAGE_KKF_GPS_CIRCLE;
                    s_kkf_gps_circle_item = 1u;
                    ips200_clear();
                }
                else
                {
                    // 第5项：清屏并显示帮助
                    ttop_gps_clear_and_show_help();
                }
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_GPS_CIRCLE)
        {
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_GPS_DEBUG;
                ips200_clear();
            }
            else if (KEY3_PRES == key)
            {
                if (s_kkf_gps_circle_item > 1u) s_kkf_gps_circle_item--;
                else s_kkf_gps_circle_item = 5u;
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                if (s_kkf_gps_circle_item < 5u) s_kkf_gps_circle_item++;
                else s_kkf_gps_circle_item = 1u;
                ips200_clear();
            }
            else if (KEY2_PRES == key)
            {
                if (s_kkf_gps_circle_item == 1u)
                    gps_nav_circle_save_center();
                else if (s_kkf_gps_circle_item == 2u)
                    gps_nav_circle_start();
                else if (s_kkf_gps_circle_item == 3u)
                    gps_nav_circle_stop();
                else if (s_kkf_gps_circle_item == 4u)
                    gps_nav_circle_toggle_dir();
                else
                {
                    s_ui_page = UI_PAGE_KKF_GPS_DEBUG;
                }
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_GPS_STORAGE)
        {
            if (KEY4_PRES == key || KEY2_PRES == key)
            {
                gps_storage_stop();
                s_ui_page = UI_PAGE_KKF_MAIN;
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_PARAM)
        {
            // 1:MODE_SEL 2:EIGHT_TURN ... 14:MOTOR_DUTY 15:LG_DBG
            if (KEY3_PRES == key)
            {
                if (s_kkf_param_item > 1u) s_kkf_param_item--;
                else s_kkf_param_item = 15u;
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                if (s_kkf_param_item < 15u) s_kkf_param_item++;
                else s_kkf_param_item = 1u;
                ips200_clear();
            }
            else if (KEY2_PRES == key)
            {
                if (s_kkf_param_item == 1u)
                {
                    s_ui_page = UI_PAGE_KKF_MODE_SELECT;
                    s_kkf_mode_item = s_kkf_mode; // 默认指到当前模式
                    ips200_clear();
                }
                else if (s_kkf_param_item == 2u)
                {
                    s_ui_page = UI_PAGE_KKF_EIGHT_TURN;
                    s_kkf_eight_turn_item = 1u;
                    ips200_clear();
                }
                else if (s_kkf_param_item == 3u)
                {
                    s_ui_page = UI_PAGE_KKF_EIGHT2_PROG_SELECT;
                        // 进入时光标对齐当前选择（1=MAIN,2=EX1,3=EX2,4=EX3 ... 9=EX7）
                        if (s_kkf_eight2_prog_id == TTOP_EIGHT2_PROG_EX1) s_kkf_eight2_prog_item = 2u;
                        else if (s_kkf_eight2_prog_id == TTOP_EIGHT2_PROG_EX2) s_kkf_eight2_prog_item = 3u;
                        else if (s_kkf_eight2_prog_id == TTOP_EIGHT2_PROG_EX3) s_kkf_eight2_prog_item = 4u;
                        else if (s_kkf_eight2_prog_id == TTOP_EIGHT2_PROG_EX4) s_kkf_eight2_prog_item = 5u;
                        else if (s_kkf_eight2_prog_id == TTOP_EIGHT2_PROG_EX5) s_kkf_eight2_prog_item = 6u;
                        else if (s_kkf_eight2_prog_id == TTOP_EIGHT2_PROG_EX6) s_kkf_eight2_prog_item = 7u;
                        else if (s_kkf_eight2_prog_id == TTOP_EIGHT2_PROG_EX7) s_kkf_eight2_prog_item = 8u;
                        else s_kkf_eight2_prog_item = 1u;
                    ips200_clear();
                }
                else if (s_kkf_param_item == 4u)
                {
                    // PID_CHANGE：进入 PID 编辑页（按当前 PID_MODE）
                    s_ui_page = kkf_pid_mode_to_page((pid_mode_t)s_kkf_pid_mode_id);
                    s_kkf_pid_state = KKF_PID_STATE_SEL;
                    s_kkf_pid_item = 1u;
                    ips200_clear();
                }
                else if (s_kkf_param_item == 5u)
                {
                    // PID_MODE：选择要编辑哪套 PID 参数
                    s_ui_page = UI_PAGE_KKF_PID_MODE_SELECT;
                    s_kkf_pid_mode_item = kkf_pid_mode_to_item((pid_mode_t)s_kkf_pid_mode_id);
                    ips200_clear();
                }
                else if (s_kkf_param_item == 6u)
                {
                    s_ui_page = UI_PAGE_KKF_CON_DBG;
                    ips200_clear();
                }
                else if (s_kkf_param_item == 7u)
                {
                    s_ui_page = UI_PAGE_KKF_DYN_PID;
                    ips200_clear();
                }
                else if (s_kkf_param_item == 8u)
                {
                    s_ui_page = UI_PAGE_KKF_KEY_DBG;
                    ips200_clear();
                }
                else if (s_kkf_param_item == 9u)
                {
                    s_ui_page = UI_PAGE_KKF_SPCH_DBG;
                    s_kkf_spch_item = 1u;
                    s_kkf_spch_state = KKF_SPCH_STATE_SEL;
                    ips200_clear();
                }
                else if (s_kkf_param_item == 10u)
                {
                    s_ui_page = UI_PAGE_KKF_KEMU1_DBG;
                    s_kkf_kemu1_item = 0u;
                    s_kkf_kemu1_state = KKF_KEMU1_SEL;
                    ips200_clear();
                }
                else if (s_kkf_param_item == 11u)
                {
                    s_ui_page = UI_PAGE_KKF_KEMU3_DBG;
                    s_kkf_kemu3_menu_item = 1u;
                    ips200_clear();
                }
                else if (s_kkf_param_item == 12u)
                {
                    s_ui_page = UI_PAGE_KKF_MOTOR_PID;
                    s_kkf_motor_pid_item = 1u;
                    s_kkf_motor_pid_state = KKF_MOTOR_PID_STATE_SEL;
                    ips200_clear();
                }
                else if (s_kkf_param_item == 13u)
                {
                    s_ui_page = UI_PAGE_KKF_RC_YAW_CFG;
                    s_kkf_rc_yaw_cfg_item = 1u;
                    s_kkf_rc_yaw_cfg_edit = 0u;
                    ips200_clear();
                }
                else if (s_kkf_param_item == 14u)
                {
                    s_ui_page = UI_PAGE_KKF_MOTOR_DUTY;
                    s_kkf_motor_duty_enable = 0u;
                    small_driver_set_duty(0, 0);
                    ips200_clear();
                }
                else if (s_kkf_param_item == 15u)
                {
                    s_ui_page = UI_PAGE_KKF_LG_DBG;
                    ips200_clear();
                }
                else
                {
                    ips200_clear();
                }
            }
            else if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_MAIN;
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_MOTOR_DUTY)
        {
            // 占空比调试页：一个共享值同时作用于左右电机。
            int16 step = 1;
            int32 next_duty;
            if (KEY4_PRES == key)
            {
                s_kkf_motor_duty_enable = 0u;
                small_driver_set_duty(0, 0);
                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
            else if (KEY2_PRES == key)
            {
                s_kkf_motor_duty_enable = (uint8)(!s_kkf_motor_duty_enable);
                if (s_kkf_motor_duty_enable)
                {
                    small_driver_set_duty(s_kkf_motor_duty, s_kkf_motor_duty);
                }
                else
                {
                    small_driver_set_duty(0, 0);
                }
                ips200_clear();
            }
            else if ((k1_long_evt && k1_now) || KEY1_PRES == key)
            {
                if (k1_long_evt) step = KKF_MOTOR_DUTY_LONG_PRESS_STEP;
                next_duty = (int32)s_kkf_motor_duty - step;
                if (next_duty < -10000) next_duty = -10000;
                s_kkf_motor_duty = (int16)next_duty;
                if (s_kkf_motor_duty_enable)
                    small_driver_set_duty(s_kkf_motor_duty, s_kkf_motor_duty);
                ips200_clear();
            }
            else if ((k3_long_evt && k3_now) || KEY3_PRES == key)
            {
                if (k3_long_evt) step = KKF_MOTOR_DUTY_LONG_PRESS_STEP;
                next_duty = (int32)s_kkf_motor_duty + step;
                if (next_duty > 10000) next_duty = 10000;
                s_kkf_motor_duty = (int16)next_duty;
                if (s_kkf_motor_duty_enable)
                    small_driver_set_duty(s_kkf_motor_duty, s_kkf_motor_duty);
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_RC_YAW_CFG)
        {
            // MODE3 RC yaw 配置页：
            // 1:YAW_ONLY 2:RECORD 3:REPLAY
            // 选择态：KEY1/KEY3 移动，KEY2 进编辑，KEY4 返回
            // 编辑态：KEY1/KEY3 切 ON/OFF，KEY2 回选择态
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_PARAM;
                s_kkf_rc_yaw_cfg_edit = 0u;
                ips200_clear();
            }
            else if (!s_kkf_rc_yaw_cfg_edit)
            {
                if (KEY3_PRES == key)
                {
                    if (s_kkf_rc_yaw_cfg_item > 1u) s_kkf_rc_yaw_cfg_item--;
                    else s_kkf_rc_yaw_cfg_item = 4u;
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    if (s_kkf_rc_yaw_cfg_item < 4u) s_kkf_rc_yaw_cfg_item++;
                    else s_kkf_rc_yaw_cfg_item = 1u;
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_rc_yaw_cfg_edit = 1u;
                    ips200_clear();
                }
            }
            else
            {
                if (KEY1_PRES == key || KEY3_PRES == key)
                {
                    if (s_kkf_rc_yaw_cfg_item == 1u)
                    {
                        s_mode3_cfg_yaw_only = (uint8)(!s_mode3_cfg_yaw_only);
                    }
                    else if (s_kkf_rc_yaw_cfg_item == 2u)
                    {
                        s_mode3_cfg_record = (uint8)(!s_mode3_cfg_record);
                    }
                    else if (s_kkf_rc_yaw_cfg_item == 3u)
                    {
                        s_mode3_cfg_replay = (uint8)(!s_mode3_cfg_replay);
                    }
                    else
                    {
                        s_mode3_cfg_flash_slot = (s_mode3_cfg_flash_slot == 1u) ? 2u : 1u;
                    }
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_rc_yaw_cfg_edit = 0u;
                    ips200_clear();
                }
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_EIGHT_TURN)
        {
            // eight_turn：三段式状态机
            // 1) 选择行：'>'
            //    - KEY1/KEY3：移动行
            //    - KEY2：进入 编辑deg（'-'）
            // 2) 编辑deg：'-'
            //    - KEY1：减一；KEY3：加一
            //    - KEY2：保存并进入 编辑degs（'*'）
            // 3) 编辑degs：'*'
            //    - KEY1：减一；KEY3：加一
            //    - KEY2：保存并回到 选择行（'>'）
            // KEY4：返回 pid_config（从任何状态返回都会回到选择态）

            uint16 idx = (uint16)s_kkf_eight_turn_item;
            volatile ttop_eight2_seg_t *prog = kkf_eight2_prog_ptr((ttop_eight2_prog_id_t)s_kkf_eight2_prog_id);
            /* EX1 槽位用于小球开环角度控制，步进 0.01；其余槽位保持原有步进 */
            float deg_step = (s_kkf_eight2_prog_id == TTOP_EIGHT2_PROG_EX1) ? 0.01f : 1.0f;
            float dps_step = (s_kkf_eight2_prog_id == TTOP_EIGHT2_PROG_EX1) ? 0.01f : 0.1f;

            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_PARAM;
                s_kkf_eight_turn_state = KKF_EIGHT_TURN_SEL;
                ips200_clear();
            }
            else if (s_kkf_eight_turn_state == KKF_EIGHT_TURN_SEL)
            {
                if (KEY3_PRES == key)
                {
                    if (s_kkf_eight_turn_item > 0u) s_kkf_eight_turn_item--;
                    else s_kkf_eight_turn_item = (uint8)(TTOP_EIGHT2_PROG_LEN - 1u);
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    if (s_kkf_eight_turn_item < (uint8)(TTOP_EIGHT2_PROG_LEN - 1u)) s_kkf_eight_turn_item++;
                    else s_kkf_eight_turn_item = 0u;
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_eight_turn_state = KKF_EIGHT_TURN_EDIT_DEG;
                    ips200_clear();
                }
            }
            else if (s_kkf_eight_turn_state == KKF_EIGHT_TURN_EDIT_DEG)
            {
                if (k1_long_evt && k1_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_EIGHT_TURN_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        prog[idx].target_rel_deg -= deg_step;
                    }
                    ips200_clear();
                }
                else if (k3_long_evt && k3_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_EIGHT_TURN_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        prog[idx].target_rel_deg += deg_step;
                    }
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    prog[idx].target_rel_deg -= deg_step;
                    ips200_clear();
                }
                else if (KEY3_PRES == key)
                {
                    prog[idx].target_rel_deg += deg_step;
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_eight_turn_state = KKF_EIGHT_TURN_EDIT_DPS;
                    ips200_clear();
                }
            }
            else // KKF_EIGHT_TURN_EDIT_DPS
            {
                if (k1_long_evt && k1_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_EIGHT_TURN_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        prog[idx].rate_dps -= dps_step;
                    }
                    ips200_clear();
                }
                else if (k3_long_evt && k3_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_EIGHT_TURN_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        prog[idx].rate_dps += dps_step;
                    }
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    prog[idx].rate_dps -= dps_step;
                    ips200_clear();
                }
                else if (KEY3_PRES == key)
                {
                    prog[idx].rate_dps += dps_step;
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_eight_turn_state = KKF_EIGHT_TURN_SEL;
                    ips200_clear();
                }
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_EIGHT2_PROG_SELECT)
        {
            // 段表选择页：KEY1/KEY3 移动；KEY2 确认并切换；KEY4 返回
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
            else if (KEY3_PRES == key)
            {
                if (s_kkf_eight2_prog_item > 1u) s_kkf_eight2_prog_item--;
                else s_kkf_eight2_prog_item = 8u;
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                if (s_kkf_eight2_prog_item < 8u) s_kkf_eight2_prog_item++;
                else s_kkf_eight2_prog_item = 1u;
                ips200_clear();
            }
            else if (KEY2_PRES == key)
            {
                if (s_kkf_eight2_prog_item == 2u) s_kkf_eight2_prog_id = TTOP_EIGHT2_PROG_EX1;
                else if (s_kkf_eight2_prog_item == 3u) s_kkf_eight2_prog_id = TTOP_EIGHT2_PROG_EX2;
                else if (s_kkf_eight2_prog_item == 4u) s_kkf_eight2_prog_id = TTOP_EIGHT2_PROG_EX3;
                else if (s_kkf_eight2_prog_item == 5u) s_kkf_eight2_prog_id = TTOP_EIGHT2_PROG_EX4;
                else if (s_kkf_eight2_prog_item == 6u) s_kkf_eight2_prog_id = TTOP_EIGHT2_PROG_EX5;
                else if (s_kkf_eight2_prog_item == 7u) s_kkf_eight2_prog_id = TTOP_EIGHT2_PROG_EX6;
                else if (s_kkf_eight2_prog_item == 8u) s_kkf_eight2_prog_id = TTOP_EIGHT2_PROG_EX7;
                else s_kkf_eight2_prog_id = TTOP_EIGHT2_PROG_MAIN;

                // 同步到八字模块：运行时从选中段表输出 g_yaw_target_deg
                ttop_eight2_select_program((ttop_eight2_prog_id_t)s_kkf_eight2_prog_id);

                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_PID_MODE_SELECT)
        {
            // PID_MODE 选择页：KEY1/KEY3 移动；KEY2 确认；KEY4 返回
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
            else if (KEY3_PRES == key)
            {
                if (s_kkf_pid_mode_item > 1u) s_kkf_pid_mode_item--;
                else s_kkf_pid_mode_item = 16u;
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                if (s_kkf_pid_mode_item < 16u) s_kkf_pid_mode_item++;
                else s_kkf_pid_mode_item = 1u;
                ips200_clear();
            }
            else if (KEY2_PRES == key)
            {
                s_kkf_pid_mode_id = kkf_item_to_pid_mode(s_kkf_pid_mode_item);
                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_LG_DBG)
        {
            /* KEY2 切换编辑字段：TGT → ALPHA → CENTER → SWP1 → SWP2 → TIME → MODE */
            if (KEY2_PRES == key)
            {
                s_kkf_lg_edit++;
                if (s_kkf_lg_edit > 6u) s_kkf_lg_edit = 0u;
                ips200_clear();
            }
            /* KEY1/KEY3 调整当前字段 */
            else if (KEY1_PRES == key)
            {
                if (s_kkf_lg_edit == 0u && g_liangan_target < 800u) g_liangan_target++;
                else if (s_kkf_lg_edit == 1u && g_liangan_alpha_pos < 0.99f) g_liangan_alpha_pos += 0.01f;
                else if (s_kkf_lg_edit == 2u && g_liangan_center_offset < 200) { g_liangan_center_offset++; g_liangan_target = (uint16_t)(400 + g_liangan_center_offset); }
                else if (s_kkf_lg_edit == 3u && g_liangan_sweep_ofs1 < 300) g_liangan_sweep_ofs1++;
                else if (s_kkf_lg_edit == 4u && g_liangan_sweep_ofs2 < 300) g_liangan_sweep_ofs2++;
                else if (s_kkf_lg_edit == 5u && g_liangan_sweep_time_ms < 60000u) g_liangan_sweep_time_ms += 100u;
                else if (s_kkf_lg_edit == 6u) { g_liangan_sweep_mode++; if (g_liangan_sweep_mode > 4u) g_liangan_sweep_mode = 0u; }
                ips200_clear();
            }
            else if (KEY3_PRES == key)
            {
                if (s_kkf_lg_edit == 0u && g_liangan_target > 0u) g_liangan_target--;
                else if (s_kkf_lg_edit == 1u && g_liangan_alpha_pos > 0.00f) g_liangan_alpha_pos -= 0.01f;
                else if (s_kkf_lg_edit == 2u && g_liangan_center_offset > -200) { g_liangan_center_offset--; g_liangan_target = (uint16_t)(400 + g_liangan_center_offset); }
                else if (s_kkf_lg_edit == 3u && g_liangan_sweep_ofs1 > -300) g_liangan_sweep_ofs1--;
                else if (s_kkf_lg_edit == 4u && g_liangan_sweep_ofs2 > -300) g_liangan_sweep_ofs2--;
                else if (s_kkf_lg_edit == 5u && g_liangan_sweep_time_ms > 100u) g_liangan_sweep_time_ms -= 100u;
                else if (s_kkf_lg_edit == 6u) { if (g_liangan_sweep_mode > 0u) g_liangan_sweep_mode--; else g_liangan_sweep_mode = 4u; }
                ips200_clear();
            }
            /* 长按：MODE 字段不处理长按 */
            else if (k1_long_evt && k1_now)
            {
                if (s_kkf_lg_edit == 0u) { if (g_liangan_target < 760u) g_liangan_target += 50u; else g_liangan_target = 800u; }
                else if (s_kkf_lg_edit == 1u) { g_liangan_alpha_pos += 0.10f; if (g_liangan_alpha_pos > 0.99f) g_liangan_alpha_pos = 0.99f; }
                else if (s_kkf_lg_edit == 2u) { g_liangan_center_offset += 10; if (g_liangan_center_offset > 200) g_liangan_center_offset = 200; g_liangan_target = (uint16_t)(400 + g_liangan_center_offset); }
                else if (s_kkf_lg_edit == 3u) { g_liangan_sweep_ofs1 += 10; if (g_liangan_sweep_ofs1 > 300) g_liangan_sweep_ofs1 = 300; }
                else if (s_kkf_lg_edit == 4u) { g_liangan_sweep_ofs2 += 10; if (g_liangan_sweep_ofs2 > 300) g_liangan_sweep_ofs2 = 300; }
                else if (s_kkf_lg_edit == 5u) { g_liangan_sweep_time_ms += 1000u; if (g_liangan_sweep_time_ms > 60000u) g_liangan_sweep_time_ms = 60000u; }
                ips200_clear();
            }
            else if (k3_long_evt && k3_now)
            {
                if (s_kkf_lg_edit == 0u) { if (g_liangan_target > 50u) g_liangan_target -= 50u; else g_liangan_target = 0u; }
                else if (s_kkf_lg_edit == 1u) { g_liangan_alpha_pos -= 0.10f; if (g_liangan_alpha_pos < 0.00f) g_liangan_alpha_pos = 0.00f; }
                else if (s_kkf_lg_edit == 2u) { g_liangan_center_offset -= 10; if (g_liangan_center_offset < -200) g_liangan_center_offset = -200; g_liangan_target = (uint16_t)(400 + g_liangan_center_offset); }
                else if (s_kkf_lg_edit == 3u) { g_liangan_sweep_ofs1 -= 10; if (g_liangan_sweep_ofs1 < -300) g_liangan_sweep_ofs1 = -300; }
                else if (s_kkf_lg_edit == 4u) { g_liangan_sweep_ofs2 -= 10; if (g_liangan_sweep_ofs2 < -300) g_liangan_sweep_ofs2 = -300; }
                else if (s_kkf_lg_edit == 5u) { g_liangan_sweep_time_ms -= 1000u; if (g_liangan_sweep_time_ms < 100u) g_liangan_sweep_time_ms = 100u; }
                ips200_clear();
            }
            /* KEY4 返回参数页 */
            else if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_CON_DBG)
        {
            // 遥控器调试页：KEY4 返回参数页
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_DYN_PID)
        {
            // 动态 PID 显示页：KEY4 返回参数页
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_SPCH_DBG)
        {
            // 速度变化调试页：KEY4 返回参数页
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_PARAM;
                s_kkf_spch_state = KKF_SPCH_STATE_SEL;
                ips200_clear();
            }
            else if (s_kkf_spch_state == KKF_SPCH_STATE_SEL)
            {
                if (KEY3_PRES == key)
                {
                    if (s_kkf_spch_item > 1u) s_kkf_spch_item--;
                    else s_kkf_spch_item = (uint8)KKF_SPCH_FIELD_COUNT;
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    if (s_kkf_spch_item < (uint8)KKF_SPCH_FIELD_COUNT) s_kkf_spch_item++;
                    else s_kkf_spch_item = 1u;
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_spch_state = KKF_SPCH_STATE_EDIT;
                    ips200_clear();
                }
            }
            else
            {
                if (k1_long_evt && k1_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_SPCH_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        kkf_spch_adjust_value((kkf_spch_field_t)s_kkf_spch_item, -1);
                    }
                    ips200_clear();
                }
                else if (k3_long_evt && k3_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_SPCH_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        kkf_spch_adjust_value((kkf_spch_field_t)s_kkf_spch_item, 1);
                    }
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    kkf_spch_adjust_value((kkf_spch_field_t)s_kkf_spch_item, -1);
                    ips200_clear();
                }
                else if (KEY3_PRES == key)
                {
                    kkf_spch_adjust_value((kkf_spch_field_t)s_kkf_spch_item, 1);
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_spch_state = KKF_SPCH_STATE_SEL;
                    ips200_clear();
                }
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_KEY_DBG)
        {
            // 按键调试页：KEY4 返回参数页
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_KEMU1_DBG)
        {
            // kemu1_dbg：
            // - 选择态('>')：KEY1/KEY3 选择 0..5 行（0=1m计数，1..5=段）
            // - 编辑左列('-')：KEY1 减，KEY3 加；KEY2：
            //     item=0 -> 回选择态；item=1..5 -> 进编辑右列
            // - 编辑右列('*')：KEY1 减，KEY3 加；KEY2 回选择态
            // - KEY4：返回参数页

            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_PARAM;
                s_kkf_kemu1_state = KKF_KEMU1_SEL;
                ips200_clear();
            }
            else if (s_kkf_kemu1_state == KKF_KEMU1_SEL)
            {
                if (KEY3_PRES == key)
                {
                    if (s_kkf_kemu1_item > 0u) s_kkf_kemu1_item--;
                    else s_kkf_kemu1_item = 7u;
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    if (s_kkf_kemu1_item < 7u) s_kkf_kemu1_item++;
                    else s_kkf_kemu1_item = 0u;
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_kemu1_state = KKF_KEMU1_EDIT_METERS;
                    ips200_clear();
                }
            }
            else if (s_kkf_kemu1_state == KKF_KEMU1_EDIT_METERS)
            {
                if (k1_long_evt && k1_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_KEMU1_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        if (s_kkf_kemu1_item == 0u)
                        {
                            kkf_kemu1_adjust_counts_per_meter(-1);
                        }
                        else
                        {
                            kkf_kemu1_adjust_meters((uint8)(s_kkf_kemu1_item - 1u), -1);
                        }
                    }
                    ips200_clear();
                }
                else if (k3_long_evt && k3_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_KEMU1_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        if (s_kkf_kemu1_item == 0u)
                        {
                            kkf_kemu1_adjust_counts_per_meter(+1);
                        }
                        else
                        {
                            kkf_kemu1_adjust_meters((uint8)(s_kkf_kemu1_item - 1u), +1);
                        }
                    }
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    if (s_kkf_kemu1_item == 0u)
                    {
                        kkf_kemu1_adjust_counts_per_meter(-1);
                    }
                    else
                    {
                        kkf_kemu1_adjust_meters((uint8)(s_kkf_kemu1_item - 1u), -1);
                    }
                    ips200_clear();
                }
                else if (KEY3_PRES == key)
                {
                    if (s_kkf_kemu1_item == 0u)
                    {
                        kkf_kemu1_adjust_counts_per_meter(+1);
                    }
                    else
                    {
                        kkf_kemu1_adjust_meters((uint8)(s_kkf_kemu1_item - 1u), +1);
                    }
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    if (s_kkf_kemu1_item == 0u)
                    {
                        s_kkf_kemu1_state = KKF_KEMU1_SEL;
                    }
                    else
                    {
                        s_kkf_kemu1_state = KKF_KEMU1_EDIT_SPEED;
                    }
                    ips200_clear();
                }
            }
            else // KKF_KEMU1_EDIT_SPEED
            {
                if (k1_long_evt && k1_now)
                {
                    if (s_kkf_kemu1_item > 0u)
                    {
                        uint8 i;
                        for (i = 0u; i < KKF_KEMU1_LONG_PRESS_EXTRA_STEPS; i++)
                        {
                            kkf_kemu1_adjust_speed((uint8)(s_kkf_kemu1_item - 1u), -1);
                        }
                    }
                    ips200_clear();
                }
                else if (k3_long_evt && k3_now)
                {
                    if (s_kkf_kemu1_item > 0u)
                    {
                        uint8 i;
                        for (i = 0u; i < KKF_KEMU1_LONG_PRESS_EXTRA_STEPS; i++)
                        {
                            kkf_kemu1_adjust_speed((uint8)(s_kkf_kemu1_item - 1u), +1);
                        }
                    }
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    if (s_kkf_kemu1_item > 0u)
                    {
                        kkf_kemu1_adjust_speed((uint8)(s_kkf_kemu1_item - 1u), -1);
                    }
                    ips200_clear();
                }
                else if (KEY3_PRES == key)
                {
                    if (s_kkf_kemu1_item > 0u)
                    {
                        kkf_kemu1_adjust_speed((uint8)(s_kkf_kemu1_item - 1u), +1);
                    }
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_kemu1_state = KKF_KEMU1_SEL;
                    ips200_clear();
                }
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_KEMU3_DBG)
        {
            // kemu3_dbg：子菜单
            // - KEY1/KEY3 选择 KM3_PATH / END / EX2_LOOP / SLOPE_DBG
            // - KEY2 进入/切换编辑
            // - KEY4 返回参数页

            if (KEY4_PRES == key)
            {
                if (s_kkf_kemu3_dbg_edit != 0u)
                {
                    s_kkf_kemu3_dbg_edit = 0u;
                }
                else
                {
                    s_ui_page = UI_PAGE_KKF_PARAM;
                }
                ips200_clear();
            }
            else if (s_kkf_kemu3_dbg_edit != 0u)
            {
                if (s_kkf_kemu3_dbg_edit == 1u)
                {
                    if (KEY1_PRES == key)
                    {
                        g_ttop_kemu3_end_seg.target_rel_deg -= 1.0f;
                        ips200_clear();
                    }
                    else if (KEY3_PRES == key)
                    {
                        g_ttop_kemu3_end_seg.target_rel_deg += 1.0f;
                        ips200_clear();
                    }
                    else if (KEY2_PRES == key)
                    {
                        s_kkf_kemu3_dbg_edit = 2u;
                        ips200_clear();
                    }
                }
                else if (s_kkf_kemu3_dbg_edit == 2u)
                {
                    if (KEY1_PRES == key)
                    {
                        g_ttop_kemu3_end_seg.rate_dps -= 0.1f;
                        if (g_ttop_kemu3_end_seg.rate_dps < 0.0f) g_ttop_kemu3_end_seg.rate_dps = 0.0f;
                        ips200_clear();
                    }
                    else if (KEY3_PRES == key)
                    {
                        g_ttop_kemu3_end_seg.rate_dps += 0.1f;
                        ips200_clear();
                    }
                    else if (KEY2_PRES == key)
                    {
                        s_kkf_kemu3_dbg_edit = 0u;
                        ips200_clear();
                    }
                }
                else if (s_kkf_kemu3_dbg_edit == 3u)
                {
                    if (KEY1_PRES == key)
                    {
                        if (g_ttop_kemu3_ex2_loop_count > 0u) g_ttop_kemu3_ex2_loop_count--;
                        ips200_clear();
                    }
                    else if (KEY3_PRES == key)
                    {
                        if (g_ttop_kemu3_ex2_loop_count < 5u) g_ttop_kemu3_ex2_loop_count++;
                        ips200_clear();
                    }
                    else if (KEY2_PRES == key)
                    {
                        s_kkf_kemu3_dbg_edit = 0u;
                        ips200_clear();
                    }
                }
                else if (s_kkf_kemu3_dbg_edit == 4u)
                {
                    // edit==4: YAW_OFS 步进 0.01，支持长按
                    if (k1_long_evt && k1_now)
                    {
                        uint8 i;
                        for (i = 0u; i < KKF_PID_LONG_PRESS_EXTRA_STEPS; i++)
                        {
                            g_yaw_zero_offset_deg -= 0.01f;
                        }
                        ips200_clear();
                    }
                    else if (k3_long_evt && k3_now)
                    {
                        uint8 i;
                        for (i = 0u; i < KKF_PID_LONG_PRESS_EXTRA_STEPS; i++)
                        {
                            g_yaw_zero_offset_deg += 0.01f;
                        }
                        ips200_clear();
                    }
                    else if (KEY1_PRES == key)
                    {
                        g_yaw_zero_offset_deg -= 0.01f;
                        ips200_clear();
                    }
                    else if (KEY3_PRES == key)
                    {
                        g_yaw_zero_offset_deg += 0.01f;
                        ips200_clear();
                    }
                    else if (KEY2_PRES == key)
                    {
                        s_kkf_kemu3_dbg_edit = 0u;
                        ips200_clear();
                    }
                }
                else if (s_kkf_kemu3_dbg_edit == 5u)
                {
                    // edit==5: YAW_KI 步进 0.001，支持长按
                    if (k1_long_evt && k1_now)
                    {
                        uint8 i;
                        for (i = 0u; i < KKF_PID_LONG_PRESS_EXTRA_STEPS; i++)
                        {
                            g_kemu3_yaw_ki_path -= 0.001f;
                        }
                        if (g_kemu3_yaw_ki_path < 0.0f) g_kemu3_yaw_ki_path = 0.0f;
                        ips200_clear();
                    }
                    else if (k3_long_evt && k3_now)
                    {
                        uint8 i;
                        for (i = 0u; i < KKF_PID_LONG_PRESS_EXTRA_STEPS; i++)
                        {
                            g_kemu3_yaw_ki_path += 0.001f;
                        }
                        ips200_clear();
                    }
                    else if (KEY1_PRES == key)
                    {
                        g_kemu3_yaw_ki_path -= 0.001f;
                        if (g_kemu3_yaw_ki_path < 0.0f) g_kemu3_yaw_ki_path = 0.0f;
                        ips200_clear();
                    }
                    else if (KEY3_PRES == key)
                    {
                        g_kemu3_yaw_ki_path += 0.001f;
                        ips200_clear();
                    }
                    else if (KEY2_PRES == key)
                    {
                        s_kkf_kemu3_dbg_edit = 0u;
                        ips200_clear();
                    }
                }
                else if (s_kkf_kemu3_dbg_edit == 6u)
                {
                    if (KEY1_PRES == key || KEY3_PRES == key)
                    {
                        s_kemu3_cfg_record = (uint8)!s_kemu3_cfg_record;
                        if (s_kemu3_cfg_record) ttop_kemu3_record_start(s_kemu3_cfg_flash_slot);
                        else ttop_kemu3_record_stop();
                        ips200_clear();
                    }
                    else if (KEY2_PRES == key)
                    {
                        s_kkf_kemu3_dbg_edit = 0u;
                        ips200_clear();
                    }
                }
                else if (s_kkf_kemu3_dbg_edit == 7u)
                {
                    if (KEY1_PRES == key || KEY3_PRES == key)
                    {
                        s_kemu3_cfg_replay = (uint8)!s_kemu3_cfg_replay;
                        if (s_kemu3_cfg_replay) ttop_kemu3_replay_start(s_kemu3_cfg_flash_slot);
                        else ttop_kemu3_replay_stop();
                        ips200_clear();
                    }
                    else if (KEY2_PRES == key)
                    {
                        s_kkf_kemu3_dbg_edit = 0u;
                        ips200_clear();
                    }
                }
                else if (s_kkf_kemu3_dbg_edit == 8u)
                {
                    if (KEY1_PRES == key || KEY3_PRES == key)
                    {
                        s_kemu3_cfg_flash_slot = (s_kemu3_cfg_flash_slot == 1u) ? 2u : 1u;
                        ips200_clear();
                    }
                    else if (KEY2_PRES == key)
                    {
                        s_kkf_kemu3_dbg_edit = 0u;
                        ips200_clear();
                    }
                }
                else if (s_kkf_kemu3_dbg_edit == 9u)
                {
                    if (KEY1_PRES == key || KEY3_PRES == key)
                    {
                        g_kemu3_use_recorded_turn++;
                        if (g_kemu3_use_recorded_turn > 3u) g_kemu3_use_recorded_turn = 1u;
                        ips200_clear();
                    }
                    else if (KEY2_PRES == key)
                    {
                        s_kkf_kemu3_dbg_edit = 0u;
                        ips200_clear();
                    }
                }
            }
            else if (KEY3_PRES == key)
            {
                if (s_kkf_kemu3_menu_item > 1u) s_kkf_kemu3_menu_item--;
                else s_kkf_kemu3_menu_item = 10u;
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                if (s_kkf_kemu3_menu_item < 10u) s_kkf_kemu3_menu_item++;
                else s_kkf_kemu3_menu_item = 1u;
                ips200_clear();
            }
            else if (KEY2_PRES == key)
            {
                if (s_kkf_kemu3_menu_item == 1u)
                {
                    s_ui_page = UI_PAGE_KKF_KEMU3_PATH;
                    s_kkf_kemu3_item = 0u;
                    s_kkf_kemu3_state = KKF_KEMU3_SEL;
                }
                else if (s_kkf_kemu3_menu_item == 2u)
                {
                    s_kkf_kemu3_dbg_edit = 1u;
                }
                else if (s_kkf_kemu3_menu_item == 3u)
                {
                    s_kkf_kemu3_dbg_edit = 3u;
                }
                else if (s_kkf_kemu3_menu_item == 4u)
                {
                    s_kkf_kemu3_dbg_edit = 4u;
                }
                else if (s_kkf_kemu3_menu_item == 5u)
                {
                    s_kkf_kemu3_dbg_edit = 5u;
                }
                else if (s_kkf_kemu3_menu_item == 6u)
                {
                    s_kkf_kemu3_dbg_edit = 6u;
                }
                else if (s_kkf_kemu3_menu_item == 7u)
                {
                    s_kkf_kemu3_dbg_edit = 7u;
                }
                else if (s_kkf_kemu3_menu_item == 8u)
                {
                    s_kkf_kemu3_dbg_edit = 8u;
                }
                else if (s_kkf_kemu3_menu_item == 9u)
                {
                    s_kkf_kemu3_dbg_edit = 9u;
                }
                else
                {
                    s_ui_page = UI_PAGE_KKF_SLOPE_DBG;
                    s_kkf_slope_item = 1u;
                    s_kkf_slope_state = KKF_SLOPE_STATE_SEL;
                }
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_KEMU3_PATH)
        {
            // kemu3_dbg：
            // - 选择态('>')：KEY1/KEY3 选择 0..8 行（0=1m计数，1..8=段）
            // - 编辑左列('-')：KEY1 减，KEY3 加；KEY2：
            //     item=0 -> 回选择态；item=1..8 -> 进编辑右列
            // - 编辑右列('*')：KEY1 减，KEY3 加；KEY2 回选择态
            // - KEY4：返回 KEMU3 子菜单

            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_KEMU3_DBG;
                s_kkf_kemu3_state = KKF_KEMU3_SEL;
                ips200_clear();
            }
            else if (s_kkf_kemu3_state == KKF_KEMU3_SEL)
            {
                if (KEY3_PRES == key)
                {
                    if (s_kkf_kemu3_item > 0u) s_kkf_kemu3_item--;
                    else s_kkf_kemu3_item = (uint8)TTOP_KEMU3_SEG_COUNT;
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    if (s_kkf_kemu3_item < (uint8)TTOP_KEMU3_SEG_COUNT) s_kkf_kemu3_item++;
                    else s_kkf_kemu3_item = 0u;
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_kemu3_state = KKF_KEMU3_EDIT_METERS;
                    ips200_clear();
                }
            }
            else if (s_kkf_kemu3_state == KKF_KEMU3_EDIT_METERS)
            {
                if (k1_long_evt && k1_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_KEMU1_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        if (s_kkf_kemu3_item == 0u)
                        {
                            kkf_kemu3_adjust_counts_per_meter(-1);
                        }
                        else
                        {
                            kkf_kemu3_adjust_meters((uint8)(s_kkf_kemu3_item - 1u), -1);
                        }
                    }
                    ips200_clear();
                }
                else if (k3_long_evt && k3_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_KEMU1_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        if (s_kkf_kemu3_item == 0u)
                        {
                            kkf_kemu3_adjust_counts_per_meter(+1);
                        }
                        else
                        {
                            kkf_kemu3_adjust_meters((uint8)(s_kkf_kemu3_item - 1u), +1);
                        }
                    }
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    if (s_kkf_kemu3_item == 0u)
                    {
                        kkf_kemu3_adjust_counts_per_meter(-1);
                    }
                    else
                    {
                        kkf_kemu3_adjust_meters((uint8)(s_kkf_kemu3_item - 1u), -1);
                    }
                    ips200_clear();
                }
                else if (KEY3_PRES == key)
                {
                    if (s_kkf_kemu3_item == 0u)
                    {
                        kkf_kemu3_adjust_counts_per_meter(+1);
                    }
                    else
                    {
                        kkf_kemu3_adjust_meters((uint8)(s_kkf_kemu3_item - 1u), +1);
                    }
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    if (s_kkf_kemu3_item == 0u)
                    {
                        s_kkf_kemu3_state = KKF_KEMU3_SEL;
                    }
                    else
                    {
                        s_kkf_kemu3_state = KKF_KEMU3_EDIT_SPEED;
                    }
                    ips200_clear();
                }
            }
            else // KKF_KEMU3_EDIT_SPEED
            {
                if (k1_long_evt && k1_now)
                {
                    if (s_kkf_kemu3_item > 0u)
                    {
                        uint8 i;
                        for (i = 0u; i < KKF_KEMU1_LONG_PRESS_EXTRA_STEPS; i++)
                        {
                            kkf_kemu3_adjust_speed((uint8)(s_kkf_kemu3_item - 1u), -1);
                        }
                    }
                    ips200_clear();
                }
                else if (k3_long_evt && k3_now)
                {
                    if (s_kkf_kemu3_item > 0u)
                    {
                        uint8 i;
                        for (i = 0u; i < KKF_KEMU1_LONG_PRESS_EXTRA_STEPS; i++)
                        {
                            kkf_kemu3_adjust_speed((uint8)(s_kkf_kemu3_item - 1u), +1);
                        }
                    }
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    if (s_kkf_kemu3_item > 0u)
                    {
                        kkf_kemu3_adjust_speed((uint8)(s_kkf_kemu3_item - 1u), -1);
                    }
                    ips200_clear();
                }
                else if (KEY3_PRES == key)
                {
                    if (s_kkf_kemu3_item > 0u)
                    {
                        kkf_kemu3_adjust_speed((uint8)(s_kkf_kemu3_item - 1u), +1);
                    }
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_kemu3_state = KKF_KEMU3_SEL;
                    ips200_clear();
                }
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_SLOPE_DBG)
        {
            // slope_dbg：
            // - 选择态('>')：KEY1/KEY3 选择 1..7 行
            // - 编辑态('-')：KEY1 减，KEY3 加；KEY2 回选择态
            // - KEY4：返回 KEMU3 子菜单

            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_KEMU3_DBG;
                s_kkf_slope_state = KKF_SLOPE_STATE_SEL;
                ips200_clear();
            }
            else if (s_kkf_slope_state == KKF_SLOPE_STATE_SEL)
            {
                if (KEY3_PRES == key)
                {
                    if (s_kkf_slope_item > 1u) s_kkf_slope_item--;
                    else s_kkf_slope_item = (uint8)KKF_SLOPE_FIELD_COUNT;
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    if (s_kkf_slope_item < (uint8)KKF_SLOPE_FIELD_COUNT) s_kkf_slope_item++;
                    else s_kkf_slope_item = 1u;
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_slope_state = KKF_SLOPE_STATE_EDIT;
                    ips200_clear();
                }
            }
            else
            {
                if (k1_long_evt && k1_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_PID_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        kkf_slope_adjust_value((kkf_slope_field_t)s_kkf_slope_item, -1);
                    }
                    ips200_clear();
                }
                else if (k3_long_evt && k3_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_PID_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        kkf_slope_adjust_value((kkf_slope_field_t)s_kkf_slope_item, +1);
                    }
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    kkf_slope_adjust_value((kkf_slope_field_t)s_kkf_slope_item, -1);
                    ips200_clear();
                }
                else if (KEY3_PRES == key)
                {
                    kkf_slope_adjust_value((kkf_slope_field_t)s_kkf_slope_item, +1);
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_slope_state = KKF_SLOPE_STATE_SEL;
                    ips200_clear();
                }
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_MOTOR_PID)
        {
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_PARAM;
                s_kkf_motor_pid_state = KKF_MOTOR_PID_STATE_SEL;
                ips200_clear();
            }
            else if (s_kkf_motor_pid_state == KKF_MOTOR_PID_STATE_SEL)
            {
                if (KEY3_PRES == key)
                {
                    if (s_kkf_motor_pid_item > 1u) s_kkf_motor_pid_item--;
                    else s_kkf_motor_pid_item = (uint8)KKF_MOTOR_PID_FIELD_COUNT;
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    if (s_kkf_motor_pid_item < (uint8)KKF_MOTOR_PID_FIELD_COUNT) s_kkf_motor_pid_item++;
                    else s_kkf_motor_pid_item = 1u;
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_motor_pid_state = KKF_MOTOR_PID_STATE_EDIT;
                    ips200_clear();
                }
            }
            else
            {
                if (k1_long_evt && k1_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_PID_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        kkf_motor_pid_adjust_value((kkf_motor_pid_field_t)s_kkf_motor_pid_item, -1);
                    }
                    ips200_clear();
                }
                else if (k3_long_evt && k3_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_PID_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        kkf_motor_pid_adjust_value((kkf_motor_pid_field_t)s_kkf_motor_pid_item, +1);
                    }
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    kkf_motor_pid_adjust_value((kkf_motor_pid_field_t)s_kkf_motor_pid_item, -1);
                    ips200_clear();
                }
                else if (KEY3_PRES == key)
                {
                    kkf_motor_pid_adjust_value((kkf_motor_pid_field_t)s_kkf_motor_pid_item, +1);
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_motor_pid_state = KKF_MOTOR_PID_STATE_SEL;
                    ips200_clear();
                }
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_MODE_SELECT)
        {
            // 1~12: MODE 1..12（与 funtion_kemu 对齐）
            if (KEY3_PRES == key)
            {
                if (s_kkf_mode_item > 1u) s_kkf_mode_item--;
                else s_kkf_mode_item = 12u;
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                if (s_kkf_mode_item < 12u) s_kkf_mode_item++;
                else s_kkf_mode_item = 1u;
                ips200_clear();
            }
            else if (KEY2_PRES == key)
            {
                // 确认选择
                s_kkf_mode = s_kkf_mode_item;

                // 关键：在“确认键”这一刻立刻把模式应用到顶层，否则会出现
                // “光标选中了但需要再进一次模式选择界面才生效”的体验。
                // 顶层 funtion_kemu：1/2/3 对应菜单 MODE 1/2/3
                funtion_kemu = (uint8)(s_kkf_mode);
                ttop_funtion_mode_once_apply();

                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
            else if (KEY4_PRES == key)
            {
                // 返回不更改模式
                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
        }

        else if (s_ui_page == UI_PAGE_KKF_PID_CASCADE ||
         s_ui_page == UI_PAGE_KKF_PID_CASCADE2 ||
         s_ui_page == UI_PAGE_KKF_PID_CASCADE3 ||
         s_ui_page == UI_PAGE_KKF_PID_CASCADE4 ||
         s_ui_page == UI_PAGE_KKF_PID_CASCADE5 ||
         s_ui_page == UI_PAGE_KKF_PID_CASCADE6 ||
         s_ui_page == UI_PAGE_KKF_PID_CASCADE7 ||
         s_ui_page == UI_PAGE_KKF_PID_CASCADE8 ||
         s_ui_page == UI_PAGE_KKF_PID_CASCADE9 ||
         s_ui_page == UI_PAGE_KKF_PID_CASCADE10 ||
         s_ui_page == UI_PAGE_KKF_PID_CASCADE11 ||
         s_ui_page == UI_PAGE_KKF_PID_CASCADE12 ||
             s_ui_page == UI_PAGE_KKF_PID_ANGLE ||
             s_ui_page == UI_PAGE_KKF_PID_DAMPED_TWO ||
             s_ui_page == UI_PAGE_KKF_PID_DAMPED_THREE ||
             s_ui_page == UI_PAGE_KKF_PID_DAMPED_FOUR)
        {
            // PID 参数编辑页：
            // - 选择态('>')：KEY1/KEY3 移动字段；KEY2 进入编辑态('-')
            // - 编辑态('-')：KEY1/KEY3 调值；KEY2 保存并调用一次 apply；回到选择态
            // - KEY4：返回参数页（并复位到选择态）

            pid_mode_t mode = kkf_page_to_pid_mode(s_ui_page);

            if (KEY4_PRES == key)
            {
                ttop_funtion_mode_once_apply();  // 退出时应用参数
                s_ui_page = UI_PAGE_KKF_PARAM;
                s_kkf_pid_state = KKF_PID_STATE_SEL;
                s_kkf_pid_item = 1u;
                ips200_clear();
            }
            else if (s_kkf_pid_state == KKF_PID_STATE_SEL)
            {
                if (KEY3_PRES == key)
                {
                    if (s_kkf_pid_item > 1u) s_kkf_pid_item--;
                    else s_kkf_pid_item = (uint8)KKF_PID_FIELD_COUNT;
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    if (s_kkf_pid_item < (uint8)KKF_PID_FIELD_COUNT) s_kkf_pid_item++;
                    else s_kkf_pid_item = 1u;
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_pid_state = KKF_PID_STATE_EDIT;
                    ips200_clear();
                }
            }
            else // EDIT
            {
                // 仅当对应按键当前仍处于稳定按下时，才执行长按加速，避免旧事件串扰。
                if (k1_long_evt && k1_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_PID_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        kkf_pid_adjust_value(mode, (kkf_pid_field_t)s_kkf_pid_item, -1);
                    }
                    ips200_clear();
                }
                else if (k3_long_evt && k3_now)
                {
                    uint8 i;
                    for (i = 0u; i < KKF_PID_LONG_PRESS_EXTRA_STEPS; i++)
                    {
                        kkf_pid_adjust_value(mode, (kkf_pid_field_t)s_kkf_pid_item, +1);
                    }
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    kkf_pid_adjust_value(mode, (kkf_pid_field_t)s_kkf_pid_item, -1);
                    ips200_clear();
                }
                else if (KEY3_PRES == key)
                {
                    kkf_pid_adjust_value(mode, (kkf_pid_field_t)s_kkf_pid_item, +1);
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    // 保存/应用：按你的约定，调用一次模式 apply 触发 PID_INIT_BY_MODE
                    ttop_funtion_mode_once_apply();
                    s_kkf_pid_state = KKF_PID_STATE_SEL;
                    ips200_clear();
                }
            }
        }

    }
    last_key = key;
}
