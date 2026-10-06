// =============================================================================
//  J35_FCS.cpp  —  飞控模式切换 (Flight Control System)
// -----------------------------------------------------------------------------
//  模式:
//    AUTO (558, 默认):   过载限幅 + 迎角软限制 + 速率限幅 -> F-18 手感, 不黑视
//    OVERRIDE (559):     无限制, 最大性能, 可以拉出大过载
//  通道: 座舱 Lua 写 J35DATA\J35_FCS_cmd.txt, 本 DLL 20Hz 读
// =============================================================================
#include "J35_FCS.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stddef.h>
#include <windows.h>
#include <stdarg.h>
#include <direct.h>

// ---- SavedGames 目录查找(与 J35_MDC.cpp 同款) ----
static const char *j35FcsSavedGamesDir(void)
{
    static char buf[600] = {0};
    if (buf[0]) return buf;
    if (!GetEnvironmentVariableA("DCS_SAVED_GAMES", buf, sizeof(buf)))
        GetEnvironmentVariableA("DCS_USER_SAVED_GAMES", buf, sizeof(buf));
    if (!buf[0])
    {
        HKEY hKey;
        char regBuf[600] = {0};
        DWORD regLen = sizeof(regBuf);
        if (RegOpenKeyExA(HKEY_CURRENT_USER,
            "Software\\Eagle Dynamics\\DCS World", 0, KEY_READ, &hKey) == ERROR_SUCCESS)
        {
            if (RegQueryValueExA(hKey, "Path", NULL, NULL, (LPBYTE)regBuf, &regLen) == ERROR_SUCCESS
                && regBuf[0])
                snprintf(buf, sizeof(buf), "%s", regBuf);
            RegCloseKey(hKey);
        }
    }
    if (!buf[0])
    {
        char up[400] = {0};
        if (GetEnvironmentVariableA("USERPROFILE", up, sizeof(up)) && up[0])
            snprintf(buf, sizeof(buf), "%s\\Saved Games\\DCS", up);
    }
    if (!buf[0]) return NULL;
    size_t n = strlen(buf);
    if (n > 0 && buf[n - 1] != '\\' && buf[n - 1] != '/')
        strncat(buf, "\\", sizeof(buf) - strlen(buf) - 1);
    strncat(buf, "J35DATA\\", sizeof(buf) - strlen(buf) - 1);
    _mkdir(buf);
    return buf;
}

static unsigned long long j35FcsMs(void)
{
    return (unsigned long long)GetTickCount64();
}

static void j35FcsDbg(const char *fmt, ...)
{
    static FILE *s_fp = (FILE *)1;
    if (s_fp == (FILE *)1)
    {
        const char *d = j35FcsSavedGamesDir();
        if (!d) { s_fp = NULL; return; }
        char p[700];
        snprintf(p, sizeof(p), "%sJ35_FCS_dbg.log", d);
        s_fp = fopen(p, "a");
        if (!s_fp) return;
    }
    if (!s_fp) return;
    va_list ap;
    va_start(ap, fmt);
    vfprintf(s_fp, fmt, ap);
    va_end(ap);
    fputs("\n", s_fp);
    fflush(s_fp);
}

// ---- 常量 ----
#define J35_FCS_IN_FILE  "J35_FCS_cmd.txt"
#define J35_FCS_MS       50

// 限幅参数
#define FCS_NZ_MAX       7.5    // AUTO 最大正过载
#define FCS_NZ_MIN      -3.0    // AUTO 最大负过载
#define FCS_ALPHA_MAX    0.55   // AUTO 最大迎角 rad (~31.5°)
#define FCS_ROLL_RATE    3.5    // AUTO 最大滚转速率 rad/s (~200°/s)

static struct J35FcsState
{
    long   inSeq;
    int    mode;                 // 0=AUTO, 1=OVERRIDE
    unsigned long long tOut;
    unsigned nTick;
} g_fcs = {0};

void j35FcsInit(void)
{
    memset(&g_fcs, 0, sizeof(g_fcs));
    g_fcs.mode = 0;
    j35FcsDbg("FCS: init -> AUTO");
}

void j35FcsReset(void)
{
    memset(&g_fcs, 0, sizeof(g_fcs));
    g_fcs.mode = 0;
    const char *d = j35FcsSavedGamesDir();
    if (d)
    {
        char p[700];
        snprintf(p, sizeof(p), "%s%s", d, J35_FCS_IN_FILE);
        remove(p);
    }
    j35FcsDbg("FCS: reset -> AUTO (删 %s)", J35_FCS_IN_FILE);
}

static void j35FcsReadIn(void)
{
    const char *d = j35FcsSavedGamesDir();
    if (!d) return;
    char p[700];
    snprintf(p, sizeof(p), "%s%s", d, J35_FCS_IN_FILE);
    FILE *fp = fopen(p, "r");
    if (!fp) return;
    char line[160];
    line[0] = 0;
    if (fgets(line, sizeof(line), fp))
    {
        char tag[16] = {0};
        long seq = 0; int m = 0;
        if (sscanf(line, "%15s %ld %d", tag, &seq, &m) == 3 &&
            strcmp(tag, "J35FCS1") == 0 && seq > 0 && seq != g_fcs.inSeq)
        {
            g_fcs.inSeq = seq;
            if (m >= 0 && m <= 1 && m != g_fcs.mode)
            {
                g_fcs.mode = m;
                j35FcsDbg("FCS: mode -> %s (seq=%ld)",
                          m == 0 ? "AUTO" : "OVERRIDE", seq);
            }
        }
    }
    fclose(fp);
}

void j35FcsTick(void)
{
    unsigned long long now = j35FcsMs();
    if (now - g_fcs.tOut < (unsigned long long)J35_FCS_MS) return;
    g_fcs.tOut = now;
    g_fcs.nTick++;
    j35FcsReadIn();
}

// ---------------------------------------------------------------------------
//  俯仰限幅 — 核心
//  cmd: 原始杆位(-1~1), 返回限幅后杆位
//  alpha: 当前迎角 rad, nz: 当前过载, qbar: 动压 Pa, mass: 质量 kg
// ---------------------------------------------------------------------------
double j35FcsLimitPitch(double cmd, double alpha, double nz, double qbar, double mass)
{
    if (g_fcs.mode == 1) return cmd;  // OVERRIDE: 无限制

    double out = cmd;

    // 1) 过载限幅: nz 接近上限时削减拉杆
    if (nz > 0.0)
    {
        double margin = FCS_NZ_MAX - nz;
        if (margin < 0.0) margin = 0.0;
        // margin=0 -> 完全禁止拉杆; margin>=2 -> 无削减
        double gScale = margin / 2.0;
        if (gScale > 1.0) gScale = 1.0;
        if (out > 0.0) out *= gScale;  // 只限制拉杆(正杆), 推杆不管
    }
    if (nz < 0.0)
    {
        double margin = nz - FCS_NZ_MIN;  // 负值, 越大越接近下限
        if (margin < 0.0) margin = 0.0;
        double gScale = margin / 2.0;
        if (gScale > 1.0) gScale = 1.0;
        if (out < 0.0) out *= gScale;  // 只限制推杆(负杆)
    }

    // 2) 迎角软限制: 接近最大迎角时削减拉杆
    if (alpha > 0.0)
    {
        double aMargin = FCS_ALPHA_MAX - alpha;
        if (aMargin < 0.0) aMargin = 0.0;
        double aScale = aMargin / 0.15;  // 0.15 rad 缓冲区 (~8.6°)
        if (aScale > 1.0) aScale = 1.0;
        if (out > 0.0) out *= aScale;
    }

    return out;
}

// ---------------------------------------------------------------------------
//  滚转限幅 — AUTO 模式下限制滚转速率
// ---------------------------------------------------------------------------
double j35FcsLimitRoll(double cmd, double rollRate)
{
    if (g_fcs.mode == 1) return cmd;

    double out = cmd;
    // 滚转速率限幅: 当前速率接近上限时削减滚转杆
    double rate = fabs(rollRate);
    if (rate > 0.0)
    {
        double margin = FCS_ROLL_RATE - rate;
        if (margin < 0.0) margin = 0.0;
        double rScale = margin / 1.0;  // 1 rad/s 缓冲区
        if (rScale > 1.0) rScale = 1.0;
        // 同向滚转才限(反向回杆不限)
        if ((cmd > 0.0 && rollRate > 0.0) || (cmd < 0.0 && rollRate < 0.0))
            out *= rScale;
    }
    return out;
}

int j35FcsGetMode(void)
{
    return g_fcs.mode;
}
