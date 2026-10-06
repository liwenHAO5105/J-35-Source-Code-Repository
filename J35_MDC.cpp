// =============================================================================
//  J35_MDC.cpp  —  arg183 爆索(MDC / Miniature Detonating Cord)实现
// -----------------------------------------------------------------------------
//  与 J35_EFM.cpp 共享工具函数(j35SavedGamesDir / j35Ms / j35Dbg / clamp),
//  编译时同进一个 DLL, 链接不冲突(都是 static, 本文件用前缀 j35Mdc 避名撞)。
// =============================================================================
#include "J35_MDC.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stddef.h>
#include <windows.h>
#include <stdarg.h>
#include <direct.h>

// ---- 从 J35_EFM.cpp 引入的外部声明(它们在那里是 static, 这里必须重新实现) ----
// 不能直接 extern static 函数, 所以本文件自带 SavedGames 目录查找 + 时间 + 日志,
// 逻辑与 J35_EFM.cpp 里的同名函数等价(同款路径, 同款写法)。

// ★ 2026-09-29: 通道文件统一搬到 <SavedGamesDCS>\J35DATA\ 子目录,
//   返回的 buf 已带 "J35DATA\" 后缀(并负责 _mkdir 建目录)。
static const char *j35MdcSavedGamesDir(void)
{
    static char buf[600] = {0};
    if (buf[0]) return buf;
    // 1) 环境变量
    if (!GetEnvironmentVariableA("DCS_SAVED_GAMES", buf, sizeof(buf)))
        GetEnvironmentVariableA("DCS_USER_SAVED_GAMES", buf, sizeof(buf));
    // 2) 注册表
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
            {
                // Path 值类似 "C:\Users\xxx\Saved Games\DCS"
                snprintf(buf, sizeof(buf), "%s", regBuf);
            }
            RegCloseKey(hKey);
        }
    }
    // 3) 兜底: %USERPROFILE%\Saved Games\DCS
    if (!buf[0])
    {
        char up[400] = {0};
        if (GetEnvironmentVariableA("USERPROFILE", up, sizeof(up)) && up[0])
            snprintf(buf, sizeof(buf), "%s\\Saved Games\\DCS", up);
    }
    if (!buf[0]) return NULL;
    // 统一收尾: 保证尾部有反斜杠 -> 追加 J35DATA\ -> 建目录
    size_t n = strlen(buf);
    if (n > 0 && buf[n - 1] != '\\' && buf[n - 1] != '/')
        strncat(buf, "\\", sizeof(buf) - strlen(buf) - 1);
    strncat(buf, "J35DATA\\", sizeof(buf) - strlen(buf) - 1);
    _mkdir(buf); // 已存在则失败, 无害
    return buf;
}

static unsigned long long j35MdcMs(void)
{
    return (unsigned long long)GetTickCount64();
}

static void j35MdcDbg(const char *fmt, ...)
{
    // 写到 Saved Games\DCS\J35DATA\J35_MDC_dbg.log (与主 DLL 的 j35Dbg 独立, 避免名字冲突)
    static FILE *s_fp = (FILE *)1; // 哨兵: 第一次打开
    if (s_fp == (FILE *)1)
    {
        const char *d = j35MdcSavedGamesDir();
        if (!d) { s_fp = NULL; return; }
        char p[700];
        snprintf(p, sizeof(p), "%sJ35_MDC_dbg.log", d);
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
#define J35_MDC_IN_FILE  "J35_MDC_cmd.txt"  // 座舱 Lua 写, 本 DLL 读
#define J35_MDC_MS       50                   // 读限频(20Hz)
#define J35_MDC_ARG      183                  // 爆索参数号

static struct J35MdcState
{
    long   inSeq;                  // 文件行号(只认新行)
    double v;                      // 爆索值(0 = 无裂纹, 1 = 触发)
    unsigned long long tOut;       // 读限频
    unsigned nTick;                // 本函数被调用次数
} g_mdc = {0};

// 出生/任务重开: 归零 + 删通道文件
void j35MdcReset(void)
{
    memset(&g_mdc, 0, sizeof(g_mdc));
    const char *d = j35MdcSavedGamesDir();
    if (!d) return;
    char p[700];
    snprintf(p, sizeof(p), "%s%s", d, J35_MDC_IN_FILE);
    remove(p);
    j35MdcDbg("MDC: reset (arg%d, 删 %s)", J35_MDC_ARG, J35_MDC_IN_FILE);
}

static void j35MdcReadIn(void)
{
    const char *d = j35MdcSavedGamesDir();
    if (!d) return;
    char p[700];
    snprintf(p, sizeof(p), "%s%s", d, J35_MDC_IN_FILE);
    FILE *fp = fopen(p, "r");
    if (!fp) return;
    char line[160];
    line[0] = 0;
    if (fgets(line, sizeof(line), fp))
    {
        char tag[16] = {0};
        long seq = 0; double v = 0.0;
        if (sscanf(line, "%15s %ld %lf", tag, &seq, &v) == 3 &&
            strcmp(tag, "J35MDC1") == 0 && seq > 0 && seq != g_mdc.inSeq)
        {
            g_mdc.inSeq = seq;
            double nv = v;
            if (nv < 0.0) nv = 0.0;
            if (nv > 1.0) nv = 1.0;
            if (fabs(nv - g_mdc.v) >= 0.001)
            {
                j35MdcDbg("MDC: arg%d -> %.3f (seq=%ld)", J35_MDC_ARG, nv, seq);
                g_mdc.v = nv;
            }
        }
    }
    fclose(fp);
}

void j35MdcTick(void)
{
    unsigned long long now = j35MdcMs();
    if (now - g_mdc.tOut < (unsigned long long)J35_MDC_MS) return;
    g_mdc.tOut = now;
    g_mdc.nTick++;
    j35MdcReadIn();
}

void j35MdcWrite(float *array, size_t size)
{
    if (!array) return;
    if (size > (size_t)J35_MDC_ARG)
        array[J35_MDC_ARG] = (float)g_mdc.v;
}
