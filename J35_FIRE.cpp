// =============================================================================
//  J35_FIRE.cpp — 引擎火警告警 (2026-10-05)
// =============================================================================
#include "J35_FIRE.h"
#include <FM/wHumanCustomPhysicsAPI.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#define J35_FIRE_DMG_THRESH  0.80   // 发动机单元完整度跌破此值 = 起火(直接命中口径)
#define J35_FIRE_EXT_S       2.0    // 灭火剂作用秒数, 到点解除火警
#define J35_FIRE_ARG_L       1900   // 座舱 draw arg: 左发火(2026-10-04 三迁: 9030越界->9019撞TVC->座舱1900)
#define J35_FIRE_ARG_R       1901   // 座舱 draw arg: 右发火

static bool   g_fire[2]     = { false, false };   // 锁存的起火状态
static double g_extTimer[2] = { 0.0, 0.0 };       // 灭火剂作用计时

static double clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }

static void fireDbg(const char *fmt, ...)
{
    char b[512];
    va_list ap; va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    char p[300];
    const char *tmp = getenv("TEMP");
    if (!tmp) tmp = ".";
    snprintf(p, sizeof(p), "%s\\J35_EFM_dbg.log", tmp);
    FILE *f = fopen(p, "a");
    if (f) { fputs("[FIRE] ", f); fputs(b, f); fputc('\n', f); fclose(f); }
}

// ---- DCS -> EFM: 部件损伤 ----
void j35FireOnDamage(int element, double integrity)
{
    int idx = -1;
    if (element == 11) idx = 0;          // ENGINE_L
    else if (element == 12) idx = 1;     // ENGINE_R
    if (idx < 0) return;

    double f = clamp01(integrity);
    if (f < J35_FIRE_DMG_THRESH && !g_fire[idx])
    {
        g_fire[idx] = true;
        g_extTimer[idx] = 0.0;
        fireDbg("ENGINE %d FIRE! integrity=%.2f -> drawarg %d = 1",
                idx + 1, f, idx == 0 ? J35_FIRE_ARG_L : J35_FIRE_ARG_R);
    }
}

// ---- 每仿真帧: 灭火解除 ----
void j35FireTick(double dt, int extL, int extR)
{
    int ext[2] = { extL ? 1 : 0, extR ? 1 : 0 };
    for (int i = 0; i < 2; i++)
    {
        if (g_fire[i] && ext[i])
        {
            g_extTimer[i] += dt;
            if (g_extTimer[i] >= J35_FIRE_EXT_S)
            {
                g_fire[i] = false;
                g_extTimer[i] = 0.0;
                fireDbg("ENGINE %d fire out after extinguisher (%.1fs)", i + 1, J35_FIRE_EXT_S);
            }
        }
        else if (!ext[i])
        {
            g_extTimer[i] = 0.0;
        }
    }
}

// ---- 地勤修复复位 ----
void j35FireReset(void)
{
    for (int i = 0; i < 2; i++)
    {
        g_fire[i] = false;
        g_extTimer[i] = 0.0;
    }
}

// ---- 写座舱 draw args ----
void j35FireWriteCockpit(float *array, size_t size)
{
    if (!array) return;
    if (size > (size_t)J35_FIRE_ARG_R)
    {
        array[J35_FIRE_ARG_L] = g_fire[0] ? 1.0f : 0.0f;
        array[J35_FIRE_ARG_R] = g_fire[1] ? 1.0f : 0.0f;
    }
}
