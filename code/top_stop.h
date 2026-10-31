#ifndef TOP_STOP_H_
#define TOP_STOP_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================ 赛事功能：里程到点停机（独立模块） ============================
// 说明：
// - 该模块只依赖编码器总计数与基础驱动接口，用于在达到目标里程后停机。
// - ENCODER_COUNTS_PER_M: 由上层统一定义（建议在 `Ttop_funtion.c/.h` 统一维护），本模块不再重复写 18813。

void ttop_stop_init(void);

// 返回 1 表示已进入“全局停机”状态，上层周期任务应停止 PID/规划输出。
uint8 ttop_stop_is_active(void);

// 每周期调用（如 5ms），内部判断是否到点并执行停机（电机 duty=0、舵机回中）。
void ttop_stop_check_and_apply(void);

#ifdef __cplusplus
}
#endif

#endif // TOP_STOP_H_
