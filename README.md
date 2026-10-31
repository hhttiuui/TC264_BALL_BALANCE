# TC264 板球平衡（摄像头 + 舵机连杆）

在智能汽车竞赛平衡摩托的代码基础上改出来的**板球平衡系统**：摄像头识别小球
位置，通过串口把坐标传给 TC264，TC264 用舵机带动连杆调整平台倾角，把小球
送到指定位置。

底盘的平衡摩托逻辑（姿态解算、串级 PID、GNSS 导航、12 种赛事模式）原样保留，
只是把**航向外环换成了小球位置外环**。

```
摄像头 ──UART3 460800──▶ liangan.c ──▶ g_liangan_ball_filt（滤波后位置）
                                            │
                              Yaw 外环（8ms）  ← 目标位置 g_liangan_target
                                            │
                              角度环（4ms）──▶ 角速度环（2ms）
                                            │
                                    舵机 P33_9 (300Hz)
                                            │
                                      连杆 → 平台倾角 → 小球
```

## 硬件平台

| 项目 | 说明 |
|---|---|
| MCU | Infineon **TC264D**（TriCore 双核，200 MHz） |
| 开发环境 | AURIX Development Studio + TASKING VX-toolset |
| 姿态 | 逐飞 IMU660RX（SPI） |
| 显示 | IPS200 屏幕（SPI） |
| 执行 | 舵机 300 Hz PWM（P33_9）→ 连杆 → 平台 |
| 视觉 | 摄像头模块，串口输出小球坐标 |
| 驱动 | 逐飞小驱动板（UART2，460800） |
| 定位 | TAU1201 GNSS（可选，原竞赛功能保留） |

## 目录结构

```
Seekfree_TC264_Opensource_Library/
├── user/
│   ├── cpu0_main.c            入口 + 主循环
│   ├── isr.c                  ★ 时间片调度（1ms 中断，5 槽）
│   └── isr_config.h
│
├── code/
│   ├── liangan.c/h            ★ 摄像头串口接收 + 连杆五种模式（444 行）
│   ├── Ttop_funtion.c/h       12 种赛事模式分派
│   ├── common_flie.c/h        多速率 PID、AHRS、舵机输出（Yaw 环接小球位置）
│   ├── quaternion_ahrs.c/h    Mahony 四元数姿态解算
│   ├── motor_controller.c/h   电机速度环
│   ├── encoder_count.c/h      TIM2 正交编码器
│   ├── roll_slope_speed.c/h   坡度补偿
│   ├── small_driver_uart_control.c/h  小驱动板 UART
│   ├── ttop_rc_control.c/h    遥控器通道映射
│   ├── ins_dead_reckoning.c/h 航迹推算
│   ├── ins_path_follow.c/h    路径示教 + Pure Pursuit
│   ├── GPS.c/h  gps_*.c/h     GNSS 解析与导航
│   ├── flash_storage.c/h      DFLASH 参数持久化
│   ├── Menu.c/h  Key.c/h      IPS200 菜单 + 4 键
│   ├── ttop_kemu1/3.c/h       科目路径（7 段 / 8 段）
│   ├── top_8_turn_2.c/h       8 字绕行
│   ├── ttop_speed_change.c/h  变速控制
│   └── top_stop.c/h           里程到点停机
│
├── libraries/                 第三方库（勿改，见 NOTICE.md）
├── .cproject / .project       ADS 工程文件
├── Lcf_Tasking_Tricore_Tc.lsl TASKING 链接脚本（勿删）
├── 航向角PID计算方式总结.md    三层 PID 推导（仍适用于新的位置环）
└── 尽量不要使用的引脚.txt      TC264 引脚避坑清单
```

## 运行调度

主循环只跑非实时任务（菜单、UI、GNSS 解析），实时任务全在 `user/isr.c`
的 `cc61_pit_ch0_isr` 里，1 ms 触发、内部分频：

| 周期 | 任务 | 说明 |
|---|---|---|
| 1 ms | `imu_task_handler_2ms()` | 读 IMU + Mahony 解算（dt=0.001，名字写 2ms） |
| 2 ms | `pid_timer_handler_multirate_2ms()` | 内部再分 2/4/8 ms 三层 |
| 5 ms | `Key_Sample()` | 按键采样 |
| 5 ms | `ttop_funtion_new()` | 按 `funtion_kemu` 分派赛事模式 |
| 5 ms | `ttop_encoder_task_5ms()` | 编码器 + 坡度补偿 + 电机速度环 |
| 20 ms | `gps_nav_task_100ms()` | GNSS 导航（名字写 100ms，实际 20ms） |
| 事件 | `liangan_uart_callback()` | UART3 收到摄像头帧时 |

上电后约 2 秒（`enable_count > 2000`）才开始跑任务。

## 连杆控制：五种模式

`g_liangan_sweep_mode` 由菜单切换：

| 值 | 模式 | 行为 |
|---|---|---|
| 0 | `LIANGAN_MODE_CLOSED` | **位置闭环**。摄像头位置反馈 → Yaw 外环 → 舵机，按公式扫目标 |
| 1 | `LIANGAN_MODE_OPEN` | 段表开环，用 EX1 参数直接控制角度 |
| 2 | `LIANGAN_MODE_RC_DIR` | 遥控器 CH0 直驱角度（896±500 → ±3°） |
| 3 | `LIANGAN_MODE_RC_REC` | 遥控直驱 + 录制到 buffer |
| 4 | `LIANGAN_MODE_RC_PLAY` | 从 buffer 回放 |

闭环模式之外，`pid_target` 由 `liangan.c` 直接写，Yaw 外环不参与。

可调参数（菜单里改，存 Flash）：目标位置 `g_liangan_target`、中心偏移
`g_liangan_center_offset`、扫描偏置 `g_liangan_sweep_ofs1/2`、
扫描周期 `g_liangan_sweep_time_ms`。

## 摄像头串口协议

接在 **UART3**（原 GPS 接口），兼容两种帧：

| 格式 | 长度 | 波特率 | 字段 |
|---|---|---|---|
| 旧 | 4 字节 | 115200 | `AA XH XL FF` |
| 新 | 6 字节 | 460800 | `AA XH XL SpeH SpeL FF` |

- `XH<<8 \| XL` → 小球位置（0~800）
- `SpeH<<8 \| SpeL` → 小球速度（int16，单位 ×10）

解析在 `liangan.c`，由 `user/isr.c` 的 UART3 接收中断调用
`liangan_uart_callback()`。

## 12 种赛事模式

底盘部分完整保留竞赛时的模式体系，`funtion_kemu` 由菜单设定：

| # | 模式 | # | 模式 |
|---|---|---|---|
| 1 | 直线速度 200（CASCADE） | 7 | kemu3（8 段路径） |
| 2 | 变速（CASCADE_EX3） | 8 | 遥控器速度 only |
| 3 | 遥控 + 录制 + 回放（DAMPED_TWO） | 9 | 惯导 — 示教录制 |
| 4 | 同 3（DAMPED_THREE） | 10 | 惯导 — Pure Pursuit 回放 |
| 5 | kemu1（7 段）+ 变速 | 11 | 遥控专属 KM3 录制/回放 |
| 6 | 8 字绕行 | 12 | 预留（DAMPED_FOUR） |

板球平衡主要用 MODE1 这类简单模式，让底盘保持静止，专心调连杆闭环。

## 主要引脚

| 功能 | 引脚 | 出处 |
|---|---|---|
| 舵机 PWM | **P33_9**（ATOM0_CH1，300 Hz） | `common_flie.h` |
| 编码器 CH1 / CH2 | **P33_7 / P33_6**（TIM2 正交） | `encoder_count.c` |
| 小驱动板 UART | UART2，460800 | `small_driver_uart_control.h` |
| **摄像头 UART** | **UART3，460800** | `liangan.c` |
| 按键 KEY1~4 | P13_0 / P13_3 / P13_1 / P13_2 | `Key.h` |
| LED1~4 | P20_9 / P20_8 / P21_5 / P21_4 | `gps_key_led.h` |
| 蜂鸣器 | P33_10 | `gps_display.h` |
| 测试脚 | P21_2 P21_3 P22_0 P22_1 P22_2 P22_3 | `cpu0_main.c` |

## 关键参数

| 参数 | 值 | 出处 |
|---|---|---|
| 编码器每米计数 | 18813 | `Ttop_funtion.h` |
| 舵机频率 / 脉宽 | 300 Hz / 1000~2000 µs（中位 500） | `common_flie.h` |
| 小球位置范围 | 0~800 | `liangan.c` |
| RC 录制点数上限 | 2038（与 RC Yaw 共用 Flash 格式） | `liangan.c` |
| PID 三层周期 | 2 / 4 / 8 ms | `common_flie.c` |
| PID 参数组数 | 16 组，存 Flash | `common_flie.h` |

## 构建

需要 **AURIX Development Studio**（含 TASKING 工具链），GCC 编不了。

1. ADS → `File → Import → Existing Projects into Workspace`，选本目录
2. 右键工程 → `Build Project`
3. 首次编译自动生成 `Debug/`

命令行：

```powershell
cd Debug
make
```

## 命名陷阱

这个工程从竞赛代码改来，有些函数名和注释跟不上实际：

| 名字 | 实际行为 |
|---|---|
| `imu_task_handler_2ms()` | 实际 **1 ms** 调用，内部 `dt = 0.001` |
| `gps_nav_task_100ms()` | 实际 **20 ms** 调用 |
| `SMALL_DRIVER_RX` | 定义成 `UART2_TX_P10_5`，名字和引脚方向相反 |
| `pit_init(CCU60_CH1, 2000)` | 2 ms 定时器，但中断体是空的 |
| `Debug/` | 不是「调试版本」，是 ADS 的构建输出目录 |

`ttop_pid_task_5ms()` 只剩一个被注释掉的声明——全工程没有实现，PID 早在
`pid_timer_handler_multirate_2ms()` 里了。

## 许可

[GNU GPL v3](LICENSE)。因为 `libraries/` 下的逐飞 TC264 开源库是 GPLv3，
链接它的程序必须同样以 GPLv3 分发。第三方组件见 [NOTICE.md](NOTICE.md)。
