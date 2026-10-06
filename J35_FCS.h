// =============================================================================
//  J35_FCS.h  —  飞控模式切换 (Flight Control System)
// =============================================================================
#pragma once

void   j35FcsInit(void);          // 出生/任务重开时调用
void   j35FcsReset(void);         // 强制回到 AUTO
void   j35FcsTick(void);          // 限频读文件(20Hz), ed_fm_simulate 里调用
double j35FcsLimitPitch(double cmd, double alpha, double nz, double qbar, double mass);
double j35FcsLimitRoll(double cmd, double rollRate);
int    j35FcsGetMode(void);       // 0=AUTO, 1=OVERRIDE
