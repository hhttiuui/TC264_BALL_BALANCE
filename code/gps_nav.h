/*
 * gps_nav.h
 *
 *  Created on: 2026-03-28
 *      Author: hhttiuui
 *
 *  GPS 导航模块 —— 航向角转换 + 导航控制
 *
 *  核心功能：
 *  1. 将 GPS 航向角（正北零度、顺时针正、0~360°）
 *     转换为 IMU 航向角（上电前方零度、逆时针正、±720° 连续角）
 *  2. 根据 GPS 存储点计算目标航向，与 IMU 实际航向做差，
 *     输出转向增量到 g_yaw_target_deg，驱动 PID 链路执行转向
 *
 *  航向角定义对比：
 *  ┌──────────┬────────────────────┬──────────────────────┐
 *  │          │ GPS                │ IMU (AHRS yaw)       │
 *  ├──────────┼────────────────────┼──────────────────────┤
 *  │ 零度方向 │ 正北               │ 上电前方 (+X)        │
 *  │ 正方向   │ 顺时针             │ 逆时针               │
 *  │ 范围     │ 0° ~ 360°          │ ±720° 连续角         │
 *  └──────────┴────────────────────┴──────────────────────┘
 *
 *  转换公式：imu_yaw = -( gps_heading - gps_heading_0 )  →  wrap [-180, 180)
 */

#ifndef CODE_GPS_NAV_H_
#define CODE_GPS_NAV_H_

#include <stdint.h>
#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 可调参数（宏） ===================== */

/* 启动后校准阶段的超时 tick 数（1 tick = 100ms，100 即最多等 10 秒）
 * 校准期间只锁初始航向、不输出转向；超时后强制进入导航，避免原地干等 */
#ifndef GPS_NAV_CALIB_TICKS
#define GPS_NAV_CALIB_TICKS                     (100u)
#endif

/* 到达目标点的判定半径（米），当 GPS 实时位置距目标点 < 此距离时认为已到达 */
#ifndef GPS_NAV_ARRIVE_RADIUS_M
#define GPS_NAV_ARRIVE_RADIUS_M                 (0.6f)
#endif

/* yaw 每 100ms 最大变化量（限速平滑），单位 deg/100ms */
#ifndef GPS_NAV_YAW_SLEW_RATE_DEG_100MS
#define GPS_NAV_YAW_SLEW_RATE_DEG_100MS         (2.0f)
#endif

/* 航向差死区（deg），差值绝对值小于此值时不修正，防止小幅度抖动 */
#ifndef GPS_NAV_HEADING_DEADZONE_DEG
#define GPS_NAV_HEADING_DEADZONE_DEG            (3.0f)
#endif

/* 用连续两个 GPS 点计算当前运动航向时，要求的最小位移（米） */
#ifndef GPS_NAV_MIN_MOVE_FOR_HEADING_M
#define GPS_NAV_MIN_MOVE_FOR_HEADING_M          (2.0f)
#endif

/* GPS 对 IMU yaw 漂移的慢速校正参数 */
#ifndef GPS_NAV_DRIFT_CORR_GAIN_100MS
#define GPS_NAV_DRIFT_CORR_GAIN_100MS           (0.12f)
#endif

#ifndef GPS_NAV_DRIFT_CORR_MAX_STEP_DEG_100MS
#define GPS_NAV_DRIFT_CORR_MAX_STEP_DEG_100MS   (1.0f)
#endif

#ifndef GPS_NAV_DRIFT_CORR_DEADZONE_DEG
#define GPS_NAV_DRIFT_CORR_DEADZONE_DEG         (1.0f)
#endif

/* 相邻两个 100ms tick 之间 GPS 位置跳变超过此值（米）即判为野值丢弃 */
#ifndef GPS_NAV_OUTLIER_MAX_JUMP_M
#define GPS_NAV_OUTLIER_MAX_JUMP_M              (5.0f)
#endif

/* ===================== 调试/状态结构体 ===================== */
typedef struct
{
    float gps_heading_0_deg;      /* GPS->IMU 转换零点，会被慢速漂移校正动态修正 */
    float gps_target_heading_deg; /* 当前目标点方向（GPS坐标系，0~360） */
    float gps_motion_heading_deg; /* 当前车体运动方向（由连续 GPS 点计算） */
    float imu_heading_est_deg;    /* IMU 当前航向换算到 GPS 坐标系后的估计值 */
    float imu_target_deg;         /* 转换后的 IMU 坐标系目标航向 */
    float heading_delta_deg;      /* 航向差（正=需左转，负=需右转） */
    float drift_error_deg;        /* GPS 航向 与 IMU 航向之间的偏差 */
    float drift_correction_deg;   /* 本周期实际施加的慢速校正量 */
    uint8_t target_point_idx;      /* 当前正在追踪的目标点索引 */
    uint8_t nav_done;               /* 1=所有点已走完 */
    uint8_t heading_0_valid;        /* 初始航向是否已锁定 */
    uint8_t gps_heading_valid;      /* 当前 GPS 航向是否有效 */
} gps_nav_status_t;

/* ===================== 对外接口 ===================== */

/*
 * 核心转换函数：GPS 航向角 → IMU 航向角
 *
 * 参数：
 *   gps_heading_deg  —— GPS 原始航向（正北零度，顺时针正，0~360°）
 *   gps_heading_0_deg —— 上电时刻记录的 GPS 初始航向（正北基准）
 *
 * 返回值：
 *   IMU 坐标系下的等效航向（上电前方零度，逆时针正，[-180, 180)）
 */
float gps_nav_heading_to_imu(float gps_heading_deg, float gps_heading_0_deg);

/*
 * 反向转换函数：IMU 航向角 → GPS 航向角（调试用）
 */
float gps_nav_imu_to_heading(float imu_yaw_deg, float gps_heading_0_deg);

/*
 * 初始化 GPS 导航模块
 * 说明：
 *   - 清空导航状态
 *   - 上电后默认不启动导航，需调用 gps_nav_start()
 */
void gps_nav_init(void);

/*
 * 启动 GPS 导航
 * 说明：
 *   - 复位目标点索引/完成标志/航向锁定状态
 *   - 重置校准计数（s_calib_remain = GPS_NAV_CALIB_TICKS）
 *   - 后续由 gps_nav_task_100ms() 每100ms自动执行
 */
void gps_nav_start(void);

/*
 * 停止 GPS 导航
 * 说明：
 *   - 停止 gps_nav_task_100ms() 的导航输出更新
 *   - 保留当前导航状态，便于后续继续 start
 */
void gps_nav_stop(void);

/*
 * 锁定上电初始航向
 * 参数：gps_heading_deg —— 当前 GPS 航向（正北基准，0~360°）
 * 说明：应在 GPS 信号稳定后调用一次
 */
void gps_nav_lock_heading_0(float gps_heading_deg);

/*
 * 100ms 周期导航任务（10Hz，与GPS模块输出频率匹配）
 *
 * 工作流程：
 *   1. 用 GPS 实时经纬度 + 目标点经纬度，算出 GPS 坐标系目标航向
 *   2. 将目标航向转换到 IMU 坐标系
 *   3. 与 show_yaw_deg（IMU 实测）做差 → 航向差 delta
 *   4. 限速平滑后写入 g_yaw_target_deg
 *
 * 返回：0=导航进行中，1=所有目标点已到达
 */
uint8_t gps_nav_task_100ms(void);

/*
 * 获取导航状态（只读，用于屏幕显示/调试）
 */
const gps_nav_status_t* gps_nav_status_get(void);

/*
 * 查询导航是否完成
 */
uint8_t gps_nav_is_done(void);

/*
 * 查询校准阶段是否结束
 * 返回：1=校准已完成（或超时强制结束），0=仍在校准
 */
uint8_t gps_nav_calib_done(void);

/*
 * 查询校准阶段还剩多少个 100ms tick（0 表示已结束，可用于屏幕倒计时）
 */
uint8_t gps_nav_calib_remain(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_GPS_NAV_H_ */
