/*
 * gps_key_led.c
 *
 *  Created on: 2026年1月28日
 *      Author: elle2
 */

#include "gps_key_led.h"
#include "zf_common_headfile.h"
#include "zf_device_gnss.h"
#define IPS200_TYPE     (IPS200_TYPE_SPI)

gps_point_t_save gps_save_points[MAX_GPS_POINT_NUM] = {0};
uint8       gps_saved_point_count = 0;
uint8       led1_state = 0, led2_state = 0;


extern gnss_info_struct gnss;

void all_periph_init(void)
{
    // 1. 系统初始化
    clock_init();
    debug_init();
    gnss_init(TAU1201);
    

        gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);          // 初始化 LED1 引脚：默认高电平，推挽输出
        gpio_init(LED2, GPO, GPIO_HIGH, GPO_PUSH_PULL);         // 初始化 LED2 引脚：默认高电平，推挽输出
        gpio_init(LED3, GPO, GPIO_HIGH, GPO_PUSH_PULL);          // 初始化 LED3 引脚：默认高电平，推挽输出
        gpio_init(LED4, GPO, GPIO_HIGH, GPO_PUSH_PULL);         // 初始化 LED4 引脚：默认高电平，推挽输出

//        gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY1 输入 默认高电平 上拉输入
//        gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY2 输入 默认高电平 上拉输入
//        gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY3 输入 默认高电平 上拉输入
//        gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY4 输入 默认高电平 上拉输入

    // 4. IPS200 屏幕初始化
        ips200_set_dir(IPS200_PORTAIT);
        ips200_set_color(RGB565_BLACK, RGB565_WHITE);
        ips200_init(IPS200_TYPE);

//    ips200_init();
//    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_clear();

//    ips200_show_string(0, 50, "KEY1:Save GPS", RGB565_BLACK, RGB565_WHITE);
//    ips200_show_string(50, 50, "KEY3:Clear Screen", RGB565_BLACK, RGB565_WHITE);

    // 5. GPS 初始化（TAU1201）
    // 初始化 GPS 模块
     cpu_wait_event_ready();
}

// uint8 key_scan(gpio_pin_enum key_pin)
// {
//     #define KEY_NUM                    (4U)
//     #define KEY_DEBOUNCE_SCAN_COUNT    (2U)

//     static uint8 key_pressed_latch[KEY_NUM] = {0};
//     static uint8 key_low_counter[KEY_NUM] = {0};
//     int8 key_index = -1;

//     if(KEY1 == key_pin) key_index = 0;
//     else if(KEY2 == key_pin) key_index = 1;
//     else if(KEY3 == key_pin) key_index = 2;
//     else if(KEY4 == key_pin) key_index = 3;
//     else return 0;

//     if(gpio_get_level(key_pin) == GPIO_LOW)
//     {
//         if(key_low_counter[key_index] < KEY_DEBOUNCE_SCAN_COUNT)
//         {
//             key_low_counter[key_index]++;
//         }

//         if((key_low_counter[key_index] >= KEY_DEBOUNCE_SCAN_COUNT) && (0 == key_pressed_latch[key_index]))
//         {
//             key_pressed_latch[key_index] = 1;
//             return 1;   // 返回 1：检测到一次按下
//         }
//     }
//     else
//     {
//         key_low_counter[key_index] = 0;
//         key_pressed_latch[key_index] = 0;
//     }

//     return 0;
// }
void save_current_gps_point(void)
{
    // 检查 GPS 是否有效
    if(gnss.state != 1)
    {
//        ips200_full(RGB565_RED);
        return;
    }

    // 检查是否超出保存点数量上限
    if(gps_saved_point_count >= MAX_GPS_POINT_NUM)
    {
//        ips200_full(RGB565_RED);
        return;
    }
        gps_save_points[gps_saved_point_count].lon = gnss.longitude;
        gps_save_points[gps_saved_point_count].lat = gnss.latitude;
        gps_save_points[gps_saved_point_count].valid = 1;
        gps_saved_point_count++;

    // 刷新显示
//        display_gps_points();
}

void display_gps_points(void)
{
    ips200_clear(); // 清屏，避免多次显示叠加造成重影/倾斜
    ips200_show_string(0, 0, "Saved GPS Count:");
    ips200_show_uint(0, 16, gps_saved_point_count, 3);
}
void led_toggle_control(void)
{
    // KEY2 控制 LED1 翻转
    if(Key_Read(KEY_ID_2))
    {
        led1_state = !led1_state;
        gpio_set_level(LED1, led1_state ? GPIO_LOW : GPIO_HIGH);
    }

    // KEY4 控制 LED2 翻转
    if(Key_Read(KEY_ID_4))
    {
        led2_state = !led2_state;
        gpio_set_level(LED2, led2_state ? GPIO_LOW : GPIO_HIGH);
    }
}



