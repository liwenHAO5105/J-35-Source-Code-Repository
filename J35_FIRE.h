// =============================================================================
//  J35_FIRE.h — 引擎火警告警 (2026-10-05)
// -----------------------------------------------------------------------------
//  数据源: DCS 损伤系统回调 ed_fm_on_damage -> j35FireOnDamage
//    发动机损伤单元 ENGINE_L=11 / ENGINE_R=12, 完整度跌破阈值即判定起火(锁存)。
//  灭火: 发动机开关拨到"灭火"位(见 J35_SW g_sw.fired) -> j35FireTick,
//    灭火剂作用 J35_FIRE_EXT_S 秒后解除该发火警告。
//  输出: 座舱 draw args 1900(左发火) / 1901(右发火),
//    座舱 J35HUD_Bridge 用 get_cockpit_draw_argument_value 读取并播放中文女声。
//  (2026-10-04 三迁: 9030/9031 越界 -> 9019/9020 撞 TVC 矢量喷口(9020~9023) ->
//   座舱通道 1900/1901, 与外部模型动画彻底隔离)
// =============================================================================
#pragma once
#ifndef J35_FIRE_H
#define J35_FIRE_H

#include <stddef.h>

struct EdDrawArgument;

// DCS -> EFM: 机体部件损伤 (从 ed_fm_on_damage 调用)
void j35FireOnDamage(int element, double integrity);

// 每仿真帧 (从 ed_fm_simulate 调用); extL/extR = 灭火手柄已动作
void j35FireTick(double dt, int extL, int extR);

// 地勤修复后全部复位 (从 ed_fm_repair 调用)
void j35FireReset(void);

// 写座舱 draw args 1900/1901 (从 ed_fm_set_fc3_cockpit_draw_args_v2 调用)
void j35FireWriteCockpit(float *array, size_t size);

#endif // J35_FIRE_H
