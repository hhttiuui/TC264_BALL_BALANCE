/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include "isr_config.h"
#include "isr.h"
//#include "gps_nav.h"
//#include "gps_storage.h"
#include "liangan.h"

extern int count_pid;
extern volatile uint8 pid_flag;
extern volatile uint8 encoder_flag;
extern uint8 wifi_spi_demo_flag;
extern volatile uint8 imu_flag_2ms;
extern volatile uint8 key_flag;

uint16 imu_imucount = 0;

// 由于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要使用 interrupt_global_enable(0); 来开启中断嵌套
// 简单地说实际就是将中断函数TC系列的硬件原则断掉再 interrupt_global_disable(); 就可以响应任何的中断了，但是需要在结束自己的中断后 interrupt_global_enable(0); 恢复中断的响应



uint8_t ceshi_count = 0;
volatile uint32 g_uart1_rx_isr_count = 0;


IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);
    
//   if(!imu_flag_2ms)
//    {
//         imu_flag_2ms = 1;
//    }

   if(imu_imucount++>1000)imu_flag_2ms = 1;
   if(imu_flag_2ms)
   {
        //imu_task_handler_2ms();
        // if(ceshi_count++>8)
        // {
        //     uart_write_byte(UART_0,ceshi_count);
        //     ceshi_count = 0;
           
        // }
   }


}

uint16 enable_count = 0 , period_count = 0;
uint8_t enable_flag = 0;
static uint8_t s_gps_nav_div20 = 0u;
static uint16 s_gps_storage_div100 = 0u;

uint16 period_1ms_count = 0,period_2ms_count = 0,period_5ms_count = 0,period_10ms_count = 0;

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH0);

    if(enable_count++>2000)enable_flag = 1;

    /* [liangan] 原 GPS 数据解析已注释 */
    //if(gnss_flag)
    //{
    //    gnss_flag = 0;
    //    gnss_data_parse();           //开始解析数据
    //}    


    //2ms陀螺仪+2ms角速度环+4ms角度环+8msyaw环
    if(enable_flag)
    {
        if(period_1ms_count == 0)imu_task_handler_2ms();
        //if(period_2ms_count == 0)imu_task_handler_2ms();
        if(period_2ms_count == 1)pid_timer_handler_multirate_2ms();

        if(period_5ms_count == 0)Key_Sample();
        if(period_5ms_count == 1)ttop_funtion_new();
   
        if(period_10ms_count == 0)ttop_encoder_task_5ms();

        period_1ms_count++;
        if(period_1ms_count>=1)period_1ms_count = 0;
        period_2ms_count++;
        if(period_2ms_count>=2)period_2ms_count = 0;
        period_5ms_count++;
        if(period_5ms_count>=5)period_5ms_count = 0;
        period_10ms_count++;
        if(period_10ms_count>=5)period_10ms_count = 0;

        /* [liangan] 原 GPS 导航/存储任务已注释 */
        //if(++s_gps_nav_div20 >= 20u)
        //{
        //    s_gps_nav_div20 = 0u;
        //    gps_nav_task_100ms();
        //}
        //
        //if(++s_gps_storage_div100 >= 100u)
        //{
        //    s_gps_storage_div100 = 0u;
        //    //gps_storage_task_100ms();
        //}

    }

    // if(enable_flag)
    // {

    //     if(period_count == 0)ttop_encoder_task_5ms();
    //     if(period_count == 1)Key_Sample();//ttop_key_task_5ms();
    //     if(period_count == 2)imu_task_handler_2ms();
    //     if(period_count == 3)ttop_pid_task_5ms();
    //     period_count++;
    //     if(period_count>=5)period_count = 0;
    // }



    //if (!pid_flag){pid_flag = 1; count_pid++;}//5MSPID 
    
    //if(!encoder_flag){encoder_flag = 1;}//5MSENCODER
    //if(!key_flag){key_flag = 1;}//5MSENCODER

    //pid_timer_handler_10ms();// PID 5ms 处理函数,命名后边改
    //encoder_tim5_demo_update_5ms();// 编码器速度 5ms 更新函数
    //motor_control_task_5ms();     // 电机速度环控制任务 5ms 调用函数

}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH1);




}
// **************************** PIT中断函数 ****************************


// **************************** 外部中断函数 ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH4_REQ8_P33_7))           // 通道4中断
    {
        exti_flag_clear(ERU_CH4_REQ8_P33_7);
        camera_vsync_handler_1();                   // 摄像头1 场中断统一调用函数
    }

    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // 通道0中断
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
        // 注意: 该外部中断为摄像头场中断 如果使用摄像头 请不要在此处添加代码
        // 注意: 该外部中断为摄像头场中断 如果使用摄像头 请不要在此处添加代码
        // 注意: 该外部中断为摄像头场中断 如果使用摄像头 请不要在此处添加代码




    }

}

// 摄像头pclk引脚默认占用了 1通道和5通道来触发DMA 因此不建议在该中断函数中放置代码
IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // 通道1中断
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);
        // 注意: 该外部中断为摄像头PCLK 如果使用摄像头 请不要在此处添加代码
        // 注意: 该外部中断为摄像头PCLK 如果使用摄像头 请不要在此处添加代码
        // 注意: 该外部中断为摄像头PCLK 如果使用摄像头 请不要在此处添加代码




    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // 通道5中断
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);
        // 注意: 该外部中断为摄像头PCLK 如果使用摄像头 请不要在此处添加代码
        // 注意: 该外部中断为摄像头PCLK 如果使用摄像头 请不要在此处添加代码
        // 注意: 该外部中断为摄像头PCLK 如果使用摄像头 请不要在此处添加代码




    }
}

IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // 通道2中断
    {
        exti_flag_clear(ERU_CH2_REQ7_P00_4);
        // 注意: 该外部中断为摄像头PCLK 如果使用摄像头 请不要在此处添加代码
        // 注意: 该外部中断为摄像头PCLK 如果使用摄像头 请不要在此处添加代码
        // 注意: 该外部中断为摄像头PCLK 如果使用摄像头 请不要在此处添加代码



    }
    if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // 通道6中断
    {
        exti_flag_clear(ERU_CH6_REQ9_P20_0);
        // 注意: 该外部中断为摄像头PCLK 如果使用摄像头 请不要在此处添加代码
        // 注意: 该外部中断为摄像头PCLK 如果使用摄像头 请不要在此处添加代码
        // 注意: 该外部中断为摄像头PCLK 如果使用摄像头 请不要在此处添加代码




    }
}

IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    if(exti_flag_get(ERU_CH3_REQ3_P10_3))           // 通道3中断
    {
        exti_flag_clear(ERU_CH3_REQ3_P10_3);
        camera_vsync_handler_2();                   // 摄像头2 场中断统一调用函数
    }

    if(exti_flag_get(ERU_CH7_REQ16_P15_1))          // 通道7中断
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
        // 注意: 该外部中断为摄像头场中断 如果使用摄像头 请不要在此处添加代码
        // 注意: 该外部中断为摄像头场中断 如果使用摄像头 请不要在此处添加代码
        // 注意: 该外部中断为摄像头场中断 如果使用摄像头 请不要在此处添加代码



    }
}
// **************************** 外部中断函数 ****************************


// **************************** DMA中断函数 ****************************
IFX_INTERRUPT(dma_ch6_isr, 0, DMA_INT_PRIO_1)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_dma_handler_1();                         // 摄像头1 DMA采集统一调用函数
}

IFX_INTERRUPT(dma_ch7_isr, 0, DMA_INT_PRIO_2)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_dma_handler_2();                         // 摄像头2 DMA采集统一调用函数
}
// **************************** DMA中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认用于调试串口
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



#if DEBUG_UART_USE_INTERRUPT                        // 如果启用了 debug 串口中断
        debug_interrupr_handler();                  // 调用 debug 串口接收处理函数 数据会被 debug 串口缓冲区读取
#endif                                              // 如果修改了 DEBUG_UART_INDEX 宏定义 需要将对应的串口中断函数
    
}


// 串口1默认用于接收摄像头串口通讯
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套




}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    g_uart1_rx_isr_count++;
    // UART1 保持兼容：接收机模式走无线回调，其它场景走摄像头回调。
    if (wireless_type == RECEIVER_UART)
    {
        wireless_module_uart_handler();
    }
    else
    {
        camera_uart_handler_1();                    // 摄像头串口通讯统一调用函数
    }
}

// 串口2默认用于接收无线转串口模块
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    //uart_control_callback();                        // 小驱动板串口回调（UART2）
    //wireless_module_uart_handler();


}
// 串口3默认用于接收GPS定位模块
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    /* [liangan] 原 GPS 回调已替换为摄像头 UART 接收 */
    //gnss_uart_callback();                           // GNSS串口回调函数
    liangan_uart_callback();



}

// 串口通讯错误中断
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
