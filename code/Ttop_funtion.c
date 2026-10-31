/*
 * Ttop_funtion.c
 *
 *  Created on: 2026-01-22
 *      Author: hhttiuui
 *
 *  总顶层调度文件，用于调度各个赛事功能模块
 */

#include "zf_common_headfile.h"
#include "gps_display.h"
#include "gps_key_led.h"
#include "gps_Azimuth.h"
#include "ttop_kemu3.h"
#include "Menu.h"
#include <stdio.h>


// cpu0_main 里原本用到的外部变量/计数
extern volatile uint8 pid_flag;
extern volatile uint8 encoder_flag;
extern float show_yaw_deg;
uint8 count_20ms;

// ============================ 主周期任务调用接口 ============================

volatile uint8 funtion_kemu = 3,funtion_ready_flag;

//=======按键切换初始化，funtion_kemu 发生变化，就允许对应模式的“初始化动作”再执行一次。=======//
void ttop_funtion_mode_once_apply(void)
{
	
	static uint8 s_last_kemu = 0xFFu;

	if (s_last_kemu == funtion_kemu)
	{
		return;
	}
	s_last_kemu = funtion_kemu;


	if (funtion_kemu == 1u)// MODE1：小球+速度+舵机角度
	{
		PID_INIT_BY_MODE(PID_MODE_ANGLE);
	}
	else if (funtion_kemu == 2u)// MODE2：变速
	{		
		PID_INIT_BY_MODE(PID_MODE_CASCADE_EX3);
		ttop_speed_change_init(PID_MODE_CASCADE_EX2, PID_MODE_CASCADE_EX1);
	}
	else if (funtion_kemu == 3u)// MODE3：直线速度55
	{
		ttop_rc_control_init();
		PID_INIT_BY_MODE(PID_MODE_DAMPED_TWO);
	}
	else if (funtion_kemu == 4u)// MODE4：直线速度45
	{		
		PID_INIT_BY_MODE(PID_MODE_DAMPED_THREE);
	}
	else if (funtion_kemu == 5u)// MODE5：直线速度200+转弯
	{		
		ttop_kemu1_init();
		ttop_speed_change_init(PID_MODE_CASCADE_EX2, PID_MODE_CASCADE_EX1);
		PID_INIT_BY_MODE(PID_MODE_CASCADE);
	}
	else if (funtion_kemu == 6u)// MODE6：8字
	{
		ttop_eight2_select_program(TTOP_EIGHT2_PROG_MAIN);
		PID_INIT_BY_MODE(PID_MODE_DAMPED_TWO);
		ttop_eight2_init();
	}
	else if (funtion_kemu == 7u)// MODE7：kemu3
	{
		ttop_kemu3_init();
		ttop_speed_change_init(PID_MODE_CASCADE_EX2, PID_MODE_CASCADE_EX1);
		//PID_INIT_BY_MODE(PID_MODE_CASCADE);
	}
	else if (funtion_kemu == 8u)// MODE8：
	{
		ttop_rc_control_init();
		ttop_speed_change_init(PID_MODE_CASCADE_EX2, PID_MODE_CASCADE_EX1);
	}
	else if (funtion_kemu == 9u)// MODE9：惯导 - 示教录制
	{
		ttop_rc_control_init();
		ttop_speed_change_init(PID_MODE_CASCADE_EX2, PID_MODE_CASCADE_EX1);	
		ins_dr_init();// 初始化航迹推算 + 路径录制
		ins_path_record_init();
	}
	else if (funtion_kemu == 10u)// MODE10：惯导 - 自动跟踪
	{
		ttop_rc_control_init();
		ttop_speed_change_init(PID_MODE_CASCADE_EX2, PID_MODE_CASCADE_EX1);		
		ins_dr_init();// 初始化航迹推算
		ins_path_follow_init();// 初始化跟踪控制器
	}
	else if(funtion_kemu == 11u)// MODE11：遥控器专属 KM3 轨迹录制/回放
	{
		// MODE11 也需要预加载 KM3 两个轨迹区，不能只在 MODE7 初始化。
		ttop_kemu3_init();
		ttop_rc_control_init();
		ttop_speed_change_init(PID_MODE_CASCADE_EX2, PID_MODE_CASCADE_EX1);
	}
	else if (funtion_kemu == 12u)// MODE12：预留
	{
		PID_INIT_BY_MODE(PID_MODE_DAMPED_FOUR);
	}
	else
	{
		// 其它模式：不处理
	}
}

//=======编码器+电机控制任务，5ms调用一次=======//
void ttop_encoder_task_5ms(void)
{
	if (!funtion_ready_flag) return;
	gpio_toggle_level(P21_2);
	encoder_flag = 0;
	encoder_tim5_demo_update_5ms();// 编码器速度 5ms 更新函数
	roll_slope_speed_task_5ms(); // 根据当前速度自动调整坡度补偿
	motor_control_task_5ms();     // 电机速度环控制任务 5ms 调用函数
	
	gpio_toggle_level(P21_2);
}

volatile uint8 eight_flag = 0;
volatile uint16 eight_count = 0;


//=======PID任务，5ms调用一次=======//
void ttop_funtion_new(void)
{
	gpio_toggle_level(P22_2);
	// CH4 的急停/启动判断需要在任何状态下都可用，必须放在 ready 早退之前。
	ttop_rc_control_emergency_stop_now();
	if (!funtion_ready_flag) return;

	// MODE1：直线速度200
	if(funtion_kemu == 1){}

	//变速
	else if(funtion_kemu == 2)
	{
		ttop_speed_change_task_5ms();

	}
	// MODE3 (KM2)：自动扫目标 + 遥控器控制
	else if(funtion_kemu == 3)
	{
		liangan_auto_sweep_task_5ms();

		uint8 replay_was = ttop_rc_yaw_replay_is_active();

		ttop_rc_control_yaw_only_task_5ms();
		ttop_rc_yaw_record_task_5ms();
		ttop_rc_yaw_replay_task_5ms();

		// 回放刚完成 → 自动停止
		if (replay_was && !ttop_rc_yaw_replay_is_active())
		{
			funtion_ready_flag = 0u;
			motor_set_target_counts_per_5ms(0);
		}
	}
	// MODE4：直线速度45
	else if(funtion_kemu == 4){}
	// MODE5 (KM1)：7 段路径，跑完自动停
	else if(funtion_kemu == 5)
	{
		if (ttop_kemu1_task_5ms())
		{
			funtion_ready_flag = 0u;
			motor_set_target_counts_per_5ms(0);
		}
		ttop_speed_change_task_5ms();


	}
	// MODE6：8字
	else if(funtion_kemu == 6)
	{
		
		if(eight_count++>300)eight_flag = 1; //延时一段时间
		if(eight_flag)ttop_eight2_task_5ms();

	}
	// MODE7：kemu3
	else if(funtion_kemu == 7)
	{
		// USE_REC=2: 遥控录制 — RC 裸 yaw 写方向，录制任务采样到 slot2
		if (g_kemu3_use_recorded_turn == 2u)
		{
			ttop_rc_control_yaw_direct_task_5ms();
			ttop_kemu3_record_task_5ms();
		}
		// USE_REC=3: 回放 — slot2 全程回放写方向
		else if (g_kemu3_use_recorded_turn == 3u)
		{
			ttop_kemu3_replay_task_5ms();
		}
		// KM3 只管速度+距离，方向由上面分支管理
		if (ttop_kemu3_task_5ms())
		{
			funtion_ready_flag = 0u;
			motor_set_target_counts_per_5ms(0);
			ttop_kemu3_record_stop();
			ttop_kemu3_replay_stop();
		}
		ttop_speed_change_task_5ms();
	}
	// MODE8：
	else if(funtion_kemu == 8)
	{
		ttop_rc_control_speed_only_task_5ms();
		ttop_speed_change_task_5ms();

	}
	// MODE9：惯导 - 示教录制
	else if(funtion_kemu == 9)
	{
		// 录制任务：更新航迹推算 + 按距离记录航点
		if(eight_count++>300)eight_flag = 1;
		if(eight_flag)ins_path_record_task_5ms();
		ttop_rc_control_task_5ms();
		ttop_speed_change_task_5ms();

	}
	// MODE10：惯导 - 自动跟踪
	else if(funtion_kemu == 10)
	{
		// 跟踪任务：更新航迹推算 + Pure Pursuit 计算目标航向 → 自动写 g_yaw_target_deg
		if(eight_count++>300)eight_flag = 1; 
		if(eight_flag)ins_path_follow_task_5ms();
		ttop_rc_control_speed_only_task_5ms();
		ttop_speed_change_task_5ms();
		
	}
	// MODE11：遥控器专属 KM3 轨迹录制/回放
	else if(funtion_kemu == 11)
	{
		if (menu_kemu3_record_enabled())
		{
			ttop_kemu3_record_task_5ms();
		}
		if (menu_kemu3_replay_enabled())
		{
			// 回放时保留遥控器 CH1 速度控制，屏蔽完整 RC 任务中的 CH0 yaw 控制。
			ttop_rc_control_speed_only_task_5ms();
			ttop_kemu3_replay_task_5ms();
		}
		else
		{
			ttop_rc_control_task_5ms();
		}
		ttop_speed_change_task_5ms();
	}
	// MODE12：预留
	else if(funtion_kemu == 12){}

	else{		}
	gpio_toggle_level(P22_2);
}

static void load_saved_route_points(void)
{
    uint8 count = gps_saved_point_count;
    if(count > GPS_POINT_MAX_NUM)
    {
        count = GPS_POINT_MAX_NUM;
    }

    for(uint8 i = 0; i < count; i++)
    {
        gps_points[i] = (gps_point_t){gps_save_points[i].lon, gps_save_points[i].lat};
    }
    gps_draw_point_count = count;
}


#define IPS200_TYPE     (IPS200_TYPE_SPI)   // 选择 IPS200 显示接口类型：IPS200_TYPE_SPI 或 IPS200_TYPE_PARALLEL8
void ttop_funtion_init(void)
{
	pit_init(CCU61_CH0,1000);  //舵机pid+编码器的定时器
    pit_init(CCU60_CH1,2000);   // 2 ms 定时器，用于驱动姿态解算
    pwm_init(PWM_CH1, 300, 0);   //pwm
    //uart_init(UART_0, 460800, UART0_TX_P14_0, UART0_RX_P14_1);// UART0->现用于测试，后面会用于gps接收

    ips200_set_dir(IPS200_PORTAIT);//屏幕初始化
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_init(IPS200_TYPE);

    Key_init();                 // 按键初始化
	motor_pid_init();  // 电机速度环 PID 初始化
    //PID_INIT_NO_RATE();         // 舵机PID 初始化，NO_RATE->无角速度环版本
	// 上电加载：从 Flash 恢复 PID 配置表与 KM3 轨迹
    // 说明：
    // - 若 Flash 未初始化/校验失败，load 会返回 0，不会覆盖 RAM 默认值。
    // - 载入完成后，直接用“当前模式”做一次初始化（PID_INIT_BY_MODE），确保参数生效。
    (void)flash_storage_load_pid_mode_cfg();
	(void)flash_storage_load_kemu_all();
	(void)flash_storage_load_ins_path();
	(void)flash_storage_load_slope_cfg();
	(void)flash_storage_load_misc_cfg();

	// 舵机 PID 初始化：按当前模式（funtion_kemu）初始化一次
	// funtion_kemu 与菜单模式对齐：1..12；PID_INIT_BY_MODE 使用 pid_mode_t
    // 这里沿用上电默认 MODE1（cascade）；如果你上电希望跟随菜单选择，可以在 load 后先同步 funtion_kemu。
    PID_INIT_BY_MODE(PID_MODE_ANGLE);   /* MODE1：小球位置+速度+舵机角 */
             
    encoder_tim5_demo_init();   // 编码器初始化
    imu660rx_init();            // 陀螺仪
    //imu660ra_init();            // 陀螺仪
    //wifi_spi_demo_init();       // WiFi-SPI TCP初始化
    
	uart_receiver_init();          // 无线接收模块初始化
	small_driver_uart_init();       // 初始化电机
	ttop_rc_control_init();         // 遥控器控制模块初始化
    // 惯性导航：航迹推算模块初始化
    ins_dr_init();
	roll_slope_speed_init(); // 坡度补偿模块初始化

	gps_nav_init();
}

// ============================ GPS 事件实现 ============================
// 这些函数把按键相关逻辑拆成独立函数，菜单中直接调用即可。
void ttop_gps_save_point(void)
{
	// 直接调用现有保存函数
	save_current_gps_point();

}

void ttop_gps_display_route(void)
{
	// 加载保存的路线并按照现有显示流程绘制（不使用蜂鸣器）
	//(void)flash_storage_load_ins_path(); // 如需从 flash 恢复路线，先尝试加载（无副作用时可保留）
	// 如果有专门的加载函数可用，请替换上面的调用
	// 调整显示滚动/缩放并绘制点线
    load_saved_route_points();
	gps_calc_scroll_offset();
	gps_draw_points_lines(RGB565_BLACK, RGB565_BLACK, 3);

	// 如果实时轨迹记录已启动，绘制红色实时轨迹线
	if(gps_realtime_recording && gps_realtime_point_count > 0)
	{
		gps_draw_realtime_line(RGB565_RED);  // 改为红色
	}

	// 添加延时，防止闪屏（200ms）
	system_delay_ms(200);
}

void ttop_gps_clear_and_show_help(void)
{
	ips200_clear();
	// 清除绘图点计数并显示按键帮助
	extern uint8 gps_draw_point_count; // 在 gps_display.h 中声明
	gps_draw_point_count = 0;
	ips200_show_string(1, 0, "KEY1:Save GPS");
	ips200_show_string(1, 16, "KEY2:Display line");
	ips200_show_string(1, 32, "KEY3:Clear Screen");
}

void ttop_gps_calc_and_show_headings(void)
{
	ips200_clear();
	ips200_show_string(0, 0, "Heading Angles:");

	FLOAT32 headings[MAX_GPS_POINT_NUM];
	uint8 heading_count = gps_calc_multi_heading(headings, (uint8)(MAX_GPS_POINT_NUM - 1u));

	if (heading_count == 0u)
	{
		ips200_show_string(0, 16, "Need >= 2 points!");
		return;
	}

	uint8 i;
	uint16 cursor_x = 0;
	uint16 cursor_y = 16; /* 从第1行开始显示 */
	const uint16 screen_w = 240;

	for (i = 0; i < heading_count; i++)
	{
		char label[16];
		sprintf(label, "%d>%d", i, i + 1);
		uint16 label_w = (uint16)(strlen(label) * 8);
		uint16 space_w = 8;
		uint16 float_w = (uint16)((3 + 1 + 2) * 8); // 3 整数 + 小数点 + 2 小数
		uint16 item_w = (uint16)(label_w + space_w + float_w);

		if (cursor_x + item_w > screen_w && cursor_x > 0)
		{
			cursor_x = 0;
			cursor_y += 16;
		}

		ips200_show_string(cursor_x, cursor_y, label);
		ips200_show_float((uint16)(cursor_x + label_w + space_w), cursor_y, (double)headings[i], 3, 2);

		cursor_x += (uint16)(item_w + 16);
	}
}

void ttop_gps_nav_stop(void)
{
	gps_nav_stop();
}
