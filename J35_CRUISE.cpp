// =============================================================================
//  J35_CRUISE.cpp  —  巡航导弹目标坐标运行时注入 (原型)
//  详见 J35_CRUISE.h 头部说明。
// =============================================================================
#include "J35_CRUISE.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <windows.h>
#include <stdarg.h>
#include <direct.h>

// ---- 与 J35_MDC.cpp 同款: 自带 SavedGames 目录查找 + 日志 (避免跨文件 extern static) ----
static const char *j35CrsSavedGamesDir(void)
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

static void j35CrsDbg(const char *fmt, ...)
{
    static FILE *s_fp = (FILE *)1;
    if (s_fp == (FILE *)1)
    {
        const char *d = j35CrsSavedGamesDir();
        if (!d) { s_fp = NULL; return; }
        char p[700];
        snprintf(p, sizeof(p), "%sJ35_CRUISE_dbg.log", d);
        s_fp = fopen(p, "a");
    }
    if (!s_fp) return;
    char b[512];
    va_list ap; va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    fprintf(s_fp, "[%llu] %s\n", (unsigned long long)GetTickCount64(), b);
    fflush(s_fp);
}

// ---- WeaponBlocks.dll 偏移 (DCS 当前版本逆向值; 版本更新需重扫) ----
#define WB_OFF_VFTABLE   0x3a9930   // wCruiseAutopilot::vftable
#define WB_OFF_ADD_POS   0x162d20   // wCruiseAutopilot::add_pos(Vector3d)
#define WB_OFF_SIMULATE  0x168730   // vft[3] = simulate, 校验用

typedef void (__thiscall *AddPosFn)(void *pThis, const double *vec3);

static unsigned char *g_wbBase     = NULL;
static unsigned char *g_vftVA      = NULL;
static AddPosFn       g_addPos     = NULL;
static unsigned char *g_simulateVA = NULL;
static int            g_wbOk       = 0;

static int j35CrsResolve(void)
{
    if (g_wbOk) return 1;
    HMODULE h = GetModuleHandleA("WeaponBlocks.dll");
    if (!h) { j35CrsDbg("WeaponBlocks.dll 未加载"); return 0; }
    g_wbBase     = (unsigned char *)h;
    g_vftVA      = g_wbBase + WB_OFF_VFTABLE;
    g_addPos     = (AddPosFn)(g_wbBase + WB_OFF_ADD_POS);
    g_simulateVA = g_wbBase + WB_OFF_SIMULATE;
    g_wbOk = 1;
    j35CrsDbg("WeaponBlocks.dll base=%p vft=%p add_pos=%p", g_wbBase, g_vftVA, g_addPos);
    return 1;
}

// 候选对象校验: 首 qword == vftVA 且 vft[3] == simulateVA
// (候选指针本身在已提交可读区域内, 虚表在 DLL .rdata 恒定有效, 无需 SEH)
static int j35CrsValidate(void *cand)
{
    unsigned char **obj = (unsigned char **)cand;
    if (obj[0] != g_vftVA) return 0;
    unsigned char **vft = (unsigned char **)obj[0];
    if (vft[3] != g_simulateVA) return 0;
    return 1;
}

// 堆扫描: 遍历所有已提交私有内存, 对每个命中对象调 add_pos
static int j35CrsInjectAll(const double *vec3)
{
    int hits = 0;
    unsigned char *addr = NULL;
    MEMORY_BASIC_INFORMATION mbi;
    while (VirtualQuery(addr, &mbi, sizeof(mbi)))
    {
        addr = (unsigned char *)mbi.BaseAddress + mbi.RegionSize;
        if (mbi.State != MEM_COMMIT) continue;
        if (mbi.Type  != MEM_PRIVATE) continue;
        if (mbi.Protect & (PAGE_NOACCESS | PAGE_GUARD)) continue;
        if (!(mbi.Protect & (PAGE_READWRITE | PAGE_WRITECOPY | PAGE_EXECUTE_READWRITE | PAGE_READONLY)))
            continue;

        unsigned char *p   = (unsigned char *)mbi.BaseAddress;
        unsigned char *end = p + mbi.RegionSize - 8;
        for (; p < end; p += 8)
        {
            if (*(unsigned char **)p != g_vftVA) continue;
            if (!j35CrsValidate(p)) continue;
            j35CrsDbg("命中 wCruiseAutopilot @ %p, add_pos(%.1f, %.1f, %.1f)",
                      p, vec3[0], vec3[1], vec3[2]);
            g_addPos(p, vec3);
            hits++;
        }
    }
    return hits;
}

// ---- 命令文件通道 ----
static unsigned long g_lastSeq = 0;

void j35CruiseTick(void)
{
    // 2Hz 限频
    static unsigned long long tLast = 0;
    unsigned long long now = GetTickCount64();
    if (now - tLast < 500) return;
    tLast = now;

    if (!j35CrsResolve()) return;

    const char *d = j35CrsSavedGamesDir();
    if (!d) return;
    char path[700];
    snprintf(path, sizeof(path), "%sJ35_TGT_cmd.txt", d);
    FILE *f = fopen(path, "r");
    if (!f) return;
    char line[256] = {0};
    if (!fgets(line, sizeof(line) - 1, f)) { fclose(f); return; }
    fclose(f);

    // J35TGT1 <seq> <x> <y> <z>
    char magic[32] = {0};
    unsigned long seq = 0;
    double x = 0, y = 0, z = 0;
    if (sscanf(line, "%31s %lu %lf %lf %lf", magic, &seq, &x, &y, &z) != 5) return;
    if (strcmp(magic, "J35TGT1") != 0) return;
    if (seq == 0 || seq == g_lastSeq) return;
    g_lastSeq = seq;

    double vec3[3] = { x, y, z };
    j35CrsDbg("收到目标 seq=%lu 世界坐标 (%.1f, %.1f, %.1f), 开始堆扫描", seq, x, y, z);
    int hits = j35CrsInjectAll(vec3);
    j35CrsDbg("注入完成: %d 个巡航自驾实例", hits);
}
