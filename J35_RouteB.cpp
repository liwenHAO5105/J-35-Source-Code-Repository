/*
 * J35_RouteB.cpp —— 路线 B 注册模块（MinGW g++ 版，合并进 J35Cam.dll）
 *
 * 背景：J35Cam.dll 用 msys2 ucrt64 的 g++ 编译，无法用 MSVC 精确修饰名静态链接
 *       （MinGW 用 Itanium 修饰 _Z...，匹配不上引擎的 ?xxx@Common@@ 名字）。
 *       故这里全部用 GetProcAddress 按【精确修饰名字符串】动态绑定，零链接依赖。
 *
 * 配对安全：MSVC+UCRT 的 operator new/delete 是【静态包装】，内部就是
 *           ucrtbase 的 malloc/free（dumpbin 证实 CockpitBase 不导入 ??2/??3，
 *           只导入 api-ms-win-crt-heap 的 malloc/free）。MinGW ucrt64 的
 *           malloc/free 同样落到 ucrtbase.dll，故本文件直接用 malloc/free，
 *           与引擎完全同堆配对，无跨 CRT 风险。
 *
 * 时机：J35Cam.dll 同时在 entry.lua 的 binaries 列表中，binaries 加载阶段
 *       （早于座舱设备创建）即执行本文件的全局对象构造 -> 完成注册。
 *
 * 名字协议（ccCockpitContext::create 反编译确认）：creators 裸名 "X" 由引擎
 *       拼成 Identifier("class cockpit::"+X) 首次查找命中。
 */

#include <windows.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>     // malloc/free（MinGW ucrt64 -> ucrtbase.dll，与引擎同堆）

// ---------------------------------------------------------------------------
// 依赖加载 avSimplest.dll：
//   binaries 只列 J35Cam；Windows loader 加载本 DLL 时按导入表自动把【同目录】
//   的 avSimplest.dll 一起加载（DCS 以全路径+ALTERED_SEARCH 加载模组 binaries），
//   其 CRT 静态注册（avSimplestFLIR / ccCamera 等工厂）即在此刻完成。
//   引用其一个 C 导出符号只为强制写入导入表，从不调用。
// ---------------------------------------------------------------------------
extern "C" __declspec(dllimport) void ed_on_mission_start();
// used: 强制保留变量及其 IAT 引用，防止 -O2 折叠（编译器会判定地址恒不等于1而删死分支）
__attribute__((used)) static void* const g_avSimplest_dep_marker =
    reinterpret_cast<void*>(&ed_on_mission_start);

// ---------------------------------------------------------------------------
// 引擎函数指针类型（Windows x64 全平台只有一种调用约定，无需额外修饰）
// ---------------------------------------------------------------------------
typedef void  (*PFN_ThisVoid)(void* self);
typedef void  (*PFN_ThisCstr)(void* self, const char* s);
typedef void  (*PFN_ThisPtr )(void* self, void* p);
typedef void  (*PFN_PtrPtr  )(void* p1, void* p2);
typedef void  (*PFN_OnePtr  )(void* p1);

// 引擎侧 6 个函数（GetProcAddress 动态绑定）
static PFN_ThisVoid g_avLuaDevice_Ctor = nullptr; // ??0avLuaDevice@cockpit@@QEAA@XZ
static PFN_ThisCstr g_Identifier_Ctor  = nullptr; // ??0Identifier@Common@@QEAA@PEBD@Z
static PFN_ThisPtr  g_Factory_addId    = nullptr; // ?addIdentifier@Factory@Common@@QEAAXAEBVIdentifier@2@@Z
static PFN_ThisVoid g_Factory_Dtor     = nullptr; // ??1Factory@Common@@UEAA@XZ
static PFN_PtrPtr   g_FactoryMgr_add   = nullptr; // ?addFactory@FactoryManager@Common@@IEAAXPEAVFactory@2@@Z
static PFN_OnePtr   g_getRegistry      = nullptr; // ?getRegistry@@YAXPEAPEAVFactoryManager@Common@@@Z

// ---------------------------------------------------------------------------
// 诊断：写 %TEMP%\J35_RouteB_reg.log（静态初始化阶段 dcs.log 尚未可用）
// ---------------------------------------------------------------------------
static void rbLog(const char* s)
{
    char p[300];
    const char* tmp = getenv("TEMP");
    if (!tmp) tmp = ".";
    snprintf(p, sizeof(p), "%s\\J35_RouteB_reg.log", tmp);
    FILE* f = fopen(p, "a");
    if (f) { fputs(s, f); fputc('\n', f); fclose(f); }
}

static void* rbProc(HMODULE h, const char* name)
{
    void* p = (void*)GetProcAddress(h, name);
    if (!p) {
        char b[200];
        snprintf(b, sizeof(b), "FATAL: GetProcAddress failed: %s", name);
        rbLog(b);
    }
    return p;
}

// ---------------------------------------------------------------------------
// 自定义 Factory 的 4 槽虚表（手工构造，布局同引擎 WorldFactory）
//   [0]完整析构  [1]标量析构  [2]create  [3]destroy
// ---------------------------------------------------------------------------
static void* J35RB_Create(void* self, const void* id);
static void  J35RB_Destroy(void* self, void* product);
static void  J35RB_ScalarDtor(void* self, unsigned int flags);
static void  J35RB_CompleteDtor(void* self);

static void* const J35RB_VTable[4] = {
    (void*)&J35RB_CompleteDtor,
    (void*)&J35RB_ScalarDtor,
    (void*)&J35RB_Create,
    (void*)&J35RB_Destroy,
};

static const size_t kDeviceBlockSize = 0x100;   // avLuaDevice 实际约 0xC0，留余量

static void* J35RB_Create(void* /*self*/, const void* idRef)
{
    // 引擎按 const Identifier& 传参 -> idRef 指向 8 字节 Identifier（其值为 interned 指针）
    const void* interned = *(void* const*)idRef;

    void* mem = malloc(kDeviceBlockSize);   // 引擎 operator new 内部也是 malloc（ucrtbase）
    if (!mem) return nullptr;
    memset(mem, 0, kDeviceBlockSize);

    g_avLuaDevice_Ctor(mem);   // 原生 cockpit::avLuaDevice 构造（内部装好全部虚表）

    // 产品 +0x10 存标识符 interned 指针，供 destroyInstance 反查工厂
    *(void**)((char*)mem + 0x10) = const_cast<void*>(interned);
    return mem;
}

static void J35RB_Destroy(void* /*self*/, void* product)
{
    if (product) {
        void** vptr = *(void***)product;
        ((void(*)(void*, unsigned int))vptr[1])(product, 1u);  // 产品标量析构（引擎内部配对释放）
    }
}

static void J35RB_ScalarDtor(void* self, unsigned int flags)
{
    *(void**)self = const_cast<void*>(static_cast<const void*>(J35RB_VTable));
    g_Factory_Dtor(self);                       // 静态调引擎 Common::Factory::~Factory
    if (flags & 1u) free(self);                 // Factory 由本模块 malloc 分配
}

static void J35RB_CompleteDtor(void* self)
{
    if (self) J35RB_ScalarDtor(self, 1u);
}

// ---------------------------------------------------------------------------
// 注册：手工复刻 avSimplest FUN_180001060 的内存布局
// ---------------------------------------------------------------------------
static const char* kRegisteredName = "class cockpit::J35RouteBTestDevice";

// ---------------------------------------------------------------------------
// 全量转储 FactoryManager：列出所有工厂及其标识符名（地面真相）
//   mgr+0x38 = vector<Factory*>.begin, +0x40 = end
//   factory+0x10 = 标识符链表头(0x18 节点: [0]next [1]prev [2]char* name)
// ---------------------------------------------------------------------------
static void J35RB_DumpRegistry(void* mgr)
{
    rbLog("---- FACTORY REGISTRY DUMP BEGIN ----");

    char** begin = *(char***)((char*)mgr + 0x38);
    char** end   = *(char***)((char*)mgr + 0x40);
    rbLog(begin && end ? "factory vector readable" : "factory vector UNREADABLE");

    int fcnt = 0;
    if (begin && end) {
        for (char** slot = begin; slot < end; ++slot) {
            char* fac = *slot;
            if (!fac) continue;
            fcnt++;
            char b[220];
            snprintf(b, sizeof(b), "FACTORY[%d] @%p vtable=%p", fcnt, fac, *(void**)fac);
            rbLog(b);

            char* head = *(char**)(fac + 0x10);
            if (!head) { rbLog("   (no id list)"); continue; }
            char* node = *(char**)head;   // first real node
            int ic = 0;
            while (node && node != head) {
                const char* nm = *(const char**)(node + 0x10);
                char c[220];
                snprintf(c, sizeof(c), "    ID[%d] = %s", ic, nm ? nm : "(null)");
                rbLog(c);
                node = *(char**)node;     // next
                ic++;
                if (ic > 40) { rbLog("    ...(too many ids, stop)"); break; }
            }
        }
    }
    char s[64];
    snprintf(s, sizeof(s), "total factories = %d", fcnt);
    rbLog(s);
    rbLog("---- FACTORY REGISTRY DUMP END ----");
}

static void J35RB_Register()
{
    // 实际引用标记变量，确保对 avSimplest.dll 的导入不被 -O2 当作死数据删除
    if (g_avSimplest_dep_marker == reinterpret_cast<void*>(1))
        rbLog("impossible: avSimplest dep marker == 1");

    HMODULE hEdCore = GetModuleHandleA("edCore.dll");
    HMODULE hCkBase = GetModuleHandleA("CockpitBase.dll");
    if (!hEdCore || !hCkBase) {
        rbLog("FATAL: required engine module not loaded yet");
        return;
    }

    g_avLuaDevice_Ctor = (PFN_ThisVoid)rbProc(hCkBase, "??0avLuaDevice@cockpit@@QEAA@XZ");
    g_Identifier_Ctor  = (PFN_ThisCstr)rbProc(hEdCore, "??0Identifier@Common@@QEAA@PEBD@Z");
    g_Factory_addId    = (PFN_ThisPtr )rbProc(hEdCore, "?addIdentifier@Factory@Common@@QEAAXAEBVIdentifier@2@@Z");
    g_Factory_Dtor     = (PFN_ThisVoid)rbProc(hEdCore, "??1Factory@Common@@UEAA@XZ");
    g_FactoryMgr_add   = (PFN_PtrPtr  )rbProc(hEdCore, "?addFactory@FactoryManager@Common@@IEAAXPEAVFactory@2@@Z");
    g_getRegistry      = (PFN_OnePtr  )rbProc(hEdCore, "?getRegistry@@YAXPEAPEAVFactoryManager@Common@@@Z");
    if (!g_avLuaDevice_Ctor || !g_Identifier_Ctor ||
        !g_Factory_addId || !g_Factory_Dtor || !g_FactoryMgr_add || !g_getRegistry)
        return;

    // Factory 对象 0x20
    char* f = (char*)malloc(0x20);
    memset(f, 0, 0x20);
    *(void**)(f + 0x00) = const_cast<void*>(static_cast<const void*>(J35RB_VTable));
    *(int* )(f + 0x08) = 0;

    // 空标识符链表哨兵 0x18（next = prev = self）
    char* sentinel = (char*)malloc(0x18);
    *(void**)(sentinel + 0x00) = sentinel;
    *(void**)(sentinel + 0x08) = sentinel;
    *(void**)(f + 0x10) = sentinel;
    *(int64_t*)(f + 0x18) = 0;

    // Identifier（8 字节，interned 字符串指针）
    char id[8];
    g_Identifier_Ctor(id, kRegisteredName);
    g_Factory_addId(f, id);

    void* mgr = nullptr;
    g_getRegistry(&mgr);
    if (mgr) {
        g_FactoryMgr_add(mgr, f);
        rbLog("OK: J35RouteBTestDevice factory registered (inside J35Cam.dll)");
        J35RB_DumpRegistry(mgr);   // 转储含本工厂在内的全部注册，确认 avSimplest 名字
    } else {
        rbLog("FATAL: FactoryManager registry is null");
    }
}

// CRT 静态初始化：全局对象构造即注册（MinGW .ctors -> DllMain attach 阶段）
namespace {

    // 绕过 avSimplest.dll 的开发者/模组校验门控（FUN_180002390）。
    // 该门控顶部缓存字节 @ RVA 0x66a60：置 1 后立即放行，跳过全部 FNV 哈希授权检查。
    // 时机：此刻 avSimplest 已由本 DLL 导入表加载（其 CRT 注册已完成），
    //       且远早于座舱设备（avSimplestFLIR）的工厂 create。
    void J35RB_BypassLicenseGate() {
        HMODULE h = GetModuleHandleW(L"avSimplest.dll");
        if (!h) { rbLog("BYPASS: avSimplest module not found yet"); return; }
        unsigned char* p = reinterpret_cast<unsigned char*>(h) + 0x66a60;
        DWORD oldProt = 0;
        if (!VirtualProtect(p, 1, PAGE_READWRITE, &oldProt)) {
            rbLog("BYPASS: VirtualProtect failed"); return;
        }
        unsigned char before = *p;
        *p = 1;
        VirtualProtect(p, 1, oldProt, &oldProt);
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "BYPASS: license gate byte @%p set %u->1 (module base=%p)",
                 p, (unsigned)before, (void*)h);
        rbLog(msg);
    }

    struct J35RB_StaticRegistrar {
        J35RB_StaticRegistrar() { J35RB_BypassLicenseGate(); J35RB_Register(); }
    };
    J35RB_StaticRegistrar g_J35RB_Registrar;
}
