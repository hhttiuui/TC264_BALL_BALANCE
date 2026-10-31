#include "zf_common_headfile.h"
#include "quaternion_ahrs.h"
#include "Ttop_funtion.h"
#include "flash_storage.h"
#include "liangan.h"

#pragma section all "cpu0_dsram"

uint32 count_imu;

#include "zf_device_wifi_spi.h"
#define IPS200_TYPE     (IPS200_TYPE_SPI)

int core0_main(void)
{
    clock_init();                   // 获取并设置系统时钟
    debug_init();                   // 璋冭瘯妯″潡鍒濆鍖栵紙涓插彛/鏃ュ織绛夛級
    system_start();
    /* [liangan] 用摄像头 UART 初始化替代原 GPS 初始化 */
    //gnss_init(TAU1201);
    liangan_init();
    system_delay_ms(500);

    ttop_funtion_init();
    gpio_init(P21_2, GPO, GPIO_HIGH, GPO_PUSH_PULL);//娴嬭瘯
    gpio_init(P21_3, GPO, GPIO_HIGH, GPO_PUSH_PULL);//娴嬭瘯

    gpio_init(P22_0, GPO, GPIO_HIGH, GPO_PUSH_PULL);//娴嬭瘯
    gpio_init(P22_1, GPO, GPIO_HIGH, GPO_PUSH_PULL);//暂定pid调控指示引脚
    gpio_init(P22_2, GPO, GPIO_HIGH, GPO_PUSH_PULL);//暂定主周期指示引脚
    gpio_init(P22_3, GPO, GPIO_HIGH, GPO_PUSH_PULL);//娴嬭瘯

    cpu_wait_event_ready();

    while (TRUE)
    {
        //test
        //test
        Menu_Select();
        User_Interface();
        /* [liangan] 原 GPS 解析已注释 */
        //gnss_data_parse();
        //ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
        //led_bzw_show_task_20ms_auto();// LED 状态显示 + 更新标志位

        
    }

}

#pragma section all restore
// ********************************************************
// 运行调度说明（以 user/isr.c 为准，旧注释已作废）
//
// core0_main() 的主循环只做三件事，全是非实时任务：
//     Menu_Select()      菜单状态机
//     User_Interface()   IPS200 刷新
//     gnss_data_parse()  GNSS 解析（中断里也会调一次）
//
// 所有实时任务都在 user/isr.c 的 cc61_pit_ch0_isr 里，
// 由 pit_init(CCU61_CH0, 1000) 以 1ms 触发，内部分频：
//     1ms  -> imu_task_handler_2ms()            读 IMU + Mahony 解算（dt=0.001）
//     2ms  -> pid_timer_handler_multirate_2ms() 内部再分 2/4/8ms 三层 PID
//     5ms  -> Key_Sample()                      按键采样
//     5ms  -> ttop_funtion_new()                按 funtion_kemu 分派模式任务
//     5ms  -> ttop_encoder_task_5ms()           编码器 + 坡度补偿 + 电机速度环
//     20ms -> gps_nav_task_100ms()              GNSS 导航（名字写 100ms，实际 20ms）
//     事件 -> gnss_data_parse()                 gnss_flag 置位时
//
// 上电后约 2 秒（enable_count > 2000）才开始跑任务，给 IMU/GNSS 留稳定时间。
// pit_init(CCU60_CH1, 2000) 那个 2ms 定时器虽然初始化了，但中断体是空的。
//
// 已使用引脚（以宏定义为准，不要以旧注释为准）：
//   舵机        P33_9              (ATOM0_CH1, 300Hz)
//   编码器      P33_7 / P33_6      (TIM2 正交)
//   小驱动板    UART2, 460800      (TX 脚 P10_5 / RX 脚 P10_6，宏名与方向相反)
//   摄像头      UART3, 460800      (原 GPS 接口，见 code/liangan.c)
//   按键 KEY1~4 P13_0 / P13_3 / P13_1 / P13_2
//   LED1~4      P20_9 / P20_8 / P21_5 / P21_4
//   蜂鸣器      P33_10
//
// 测试脚（任务入口/出口各翻转一次，HIGH 宽度即任务耗时）：
//   P21_2 <- ttop_encoder_task_5ms()            独占
//   P22_1 <- imu_task_handler_2ms()             独占
//   P22_2 <- pid_timer_handler_multirate_2ms() 与 ttop_funtion_new() 共用
//   P21_3 / P22_0 / P22_3  已初始化但没有任何地方翻转
// ********************************************************
