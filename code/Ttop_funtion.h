/*
 * Ttop_funtion.h
 *
 *  Created on: 2026-01-22
 *      Author: hhttiuui
 *
 *  赛事功能模块（占位）
 */

#ifndef CODE_TTOP_FUNTION_H_
#define CODE_TTOP_FUNTION_H_

#ifdef __cplusplus
extern "C" {
#endif

// ============================ 统一标定：编码器每米计数 ============================
// 标定：车行驶 1m 时，encoder_get_demo5_count_total() 约为 18813。
// 约定：后续所有赛事功能（turn / 8字 / stop 及你拆出的新模块）都应使用这个定义，避免到处散落 18813。
#ifndef ENCODER_COUNTS_PER_M
#define ENCODER_COUNTS_PER_M          (18813LL)
#endif

// ============================ 统一周期：赛事任务 dt（ms） ============================
// 当前赛事相关任务约定按 5ms 调用一次（转弯/八字/其它规划）。
// 约定：后续拆分的新模块若需要 dt，请直接使用该宏，避免各文件重复定义。
#ifndef TTOP_TURN_DT_MS
#define TTOP_TURN_DT_MS               (5u)
#endif

// ============================ 封装主循环任务（把 if 块体一起搬过来） ============================
// 5ms 编码器/电机/按键等任务：内部会判断 yaw_delay_done / stop / encoder_flag
// 满足条件时自动清 encoder_flag 并执行原来的 if{} 内容
void ttop_encoder_task_5ms(void);

// 注意：ttop_pid_task_5ms() 全工程没有实现，PID 已迁到
// common_flie.c 的 pid_timer_handler_multirate_2ms()（2/4/8ms 多速率）。
// void ttop_pid_task_5ms(void);   // 历史残留声明，无定义，勿调用

void ttop_funtion_new(void);

static void load_saved_route_points(void);

// ============================ 模式切换：电机目标值一次性设置 ============================
// 建议在 while(TRUE) 主循环里周期调用（例如每圈调用一次即可）。
// - funtion_kemu==1：只在“首次切换到1”时执行 motor_set_target_counts_per_5ms(200)
// - funtion_kemu==2：只在“首次切换到2”时执行 motor_set_target_counts_per_5ms(100)
// - 其它值：不处理
void ttop_funtion_mode_once_apply(void);

void ttop_funtion_init(void);

extern volatile uint8 funtion_kemu;          // 功能科目选择：0=无/未选；1~12 对应菜单 MODE 1~12
extern volatile uint8 funtion_ready_flag;    // 运行使能/ready 标志（菜单确认后置 1）

// ============================ GPS 工具事件（供菜单调用） ============================
// KEY1: 保存当前 GPS 点
void ttop_gps_save_point(void);
// KEY2: 加载已保存路线并绘制
void ttop_gps_display_route(void);
// KEY3: 清屏并显示帮助提示
void ttop_gps_clear_and_show_help(void);
// KEY4: 璁＄畻骞舵樉绀哄娈佃埅鍚戣
void ttop_gps_calc_and_show_headings(void);
// 停止 GPS 导航任务输出（不清空保存点/导航状态）
void ttop_gps_nav_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_TTOP_FUNTION_H_ */
