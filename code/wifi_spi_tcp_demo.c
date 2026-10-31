/*
 * wifi_spi_tcp_demo.c
 *
 *  Created on: 2026-01-26
 *      Author: hhttiuui
 *
 *  WiFi-SPI TCP 示例：连接热点并建立 TCP 连接，供上层任务演示/调试使用。
 *  说明：
 *  - 本文件为示例用途（demo），不要误当成生产逻辑。
 *  - wifi_spi_init("SSID", NULL) 表示无密码热点。
 */

#include "zf_common_headfile.h"


// ============================ WiFi-SPI 连接参数（示例） ============================
char wifi_ssid[] = "YOUR_SSID";     //热点名称
char wifi_pwd[]  = "YOUR_PASSWORD"; //密码
char tcp_ip[]    = "192.168.137.1"; //远端 IP，和服务器 IP 对应
char tcp_port[]  = "8086";          //远端端口，和服务器端口对应
char local_port[] = "6666";         //本地端口，随便给，不要和远端端口冲突就行

uint8 wifi_ok = 0;
uint8 tcp_ok  = 0; 

uint8 wifi_spi_demo_flag = 0;


// ============================ 初始化：连接热点 + 建立 TCP ============================
void wifi_spi_demo_init(void)
{
    // 1) 初始化 WiFi-SPI 并连接热点（来自 zf_device_wifi_spi.c 的示例）
    wifi_ok = (0 == wifi_spi_init(wifi_ssid, wifi_pwd));
    // 2) 连接 TCP（若你把 zf_device_wifi_spi.h 的 WIFI_SPI_AUTO_CONNECT 设为 1，这步也可以省略）
    if (wifi_ok)
    {
        tcp_ok = (0 == wifi_spi_socket_connect("TCP", tcp_ip, tcp_port, local_port));
    }   

}

