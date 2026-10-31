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
    clock_init();                   // 鑾峰彇骞惰缃郴缁熸椂閽�
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
        //led_bzw_show_task_20ms_auto();// LED 鐘舵€佹樉绀� + 鏇存柊鏍囧織浣�

        
    }

}

#pragma section all restore
// ********************************************************
//鐜板湪鐨勪换鍔¤Е鍙戯細濮挎€佽В绠�->2ms涓柇瑙﹀彂
//               缂栫爜鍣�+鎸夐敭+pid+鎸夐敭鍒囨崲鍚庣殑鍒濆鍖� -> while涓�5ms绛夊緟瑙﹀彂

/* 
鐜板湪鐨勮皟鎺ф柟寮忥細




鐢�1ms瀹氭椂鍣紝浠�5ms涓哄懆鏈燂紝鍒嗗埆瑙﹀彂锛�
0ms ~ 1ms -> 缂栫爜鍣ㄦ洿鏂�
1ms ~ 2ms -> 鎸夐敭
2ms ~ 3ms -> 濮挎€佽В绠�
3ms ~ 4ms -> pid璁＄畻
4ms ~ 5ms -> 鐣欑┖
鑿滃崟鎿嶄綔鏀惧湪涓诲惊鐜腑



宸蹭娇鐢ㄥ紩鑴氾細
鑸垫満 -> 33_9

UART_2_TX -> P10_5锛圲ART2_TX_P10_5锛�
UART_2_RX -> P10_6锛圲ART2_RX_P10_6锛�
UART_3_TX -> P15_6锛堟敞鎰忥細瀹忓悕鍐欑殑鏄� SMALL_DRIVER_RX (UART3_TX_P15_6)锛�
UART_3_RX -> P15_7锛堟敞鎰忥細瀹忓悕鍐欑殑鏄� SMALL_DRIVER_TX (UART3_RX_P15_7)锛�

KEY1 -> P11_3
KEY2 -> P11_2
KEY3 -> P20_7

LED1 -> P20_9
LED2 -> P20_8
LED3 -> P21_5
LED4 -> P21_4

缂栫爜鍣� A 鐩� / CH1 -> P33_7锛圱IM2_ENCODER_CH1_P33_7锛�
缂栫爜鍣� B 鐩� / CH2 -> P33_6锛圱IM2_ENCODER_CH2_P33_6锛�


鍑芥暟鑰楁椂锛�
ttop_encoder_task_5ms();//缂栫爜鍣�-> 135.26us
ttop_key_task_5ms();    //鎸夐敭-> 2.06us
imu_task_handler_2ms(); //濮挎€佽В绠�-> 25.84us
ttop_pid_task_5ms();    //pid璁＄畻-> 19.62us


*/
