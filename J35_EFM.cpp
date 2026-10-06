#include "J35_EFM.h"
#include <stddef.h>
#include <FM/wHumanCustomPhysicsAPI.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <windows.h>
#include <time.h>
#include <direct.h>          // _mkdir (J35DATA 子目录)
#include "J35_HUDMath.h"   // 2026-09-28 HUD 锁定框世界<->机体系换算(J35HM::WorldToBodyDir/BodyToAzEl)
#include "J35_MDC.h"        // 2026-09-28 arg183 爆索(MDC)通道
#include "J35_FCS.h"        // 2026-09-29 飞控模式切换(AUTO/OVERRIDE)
#include "J35_CRUISE.h"     // 2026-09-30 巡航导弹目标坐标注入(堆扫描 wCruiseAutopilot + add_pos)
#include "J35_FIRE.h"       // 2026-10-05 引擎火警(损伤回调 -> 外部arg9030/9031, 灭火解除)

static void j35DbgRaw(const char *s)
{
    // ★ 2026-09-13: 每次加载 FM(每个会话/每个架次)重开日志 —— 旧版纯 append, 跑久了滚到几十 MB。
    static bool firstLine = true;
    char p[300];
    const char *tmp = getenv("TEMP");
    if (!tmp) tmp = ".";
    snprintf(p, sizeof(p), "%s\\J35_EFM_dbg.log", tmp);
    FILE *f = fopen(p, firstLine ? "w" : "a");
    firstLine = false;
    if (f) { fputs(s, f); fputc('\n', f); fclose(f); }
}

// ---- 诊断: DCS 到底来问哪些引擎参数(块内偏移 -> 调用次数 / 最后返回的值) ----
static unsigned g_pqCnt[2][40];
static double   g_pqVal[2][40];
static void j35Dbg(const char *fmt, ...)
{
    char b[512];
    va_list ap; va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    j35DbgRaw(b);
}
static unsigned long long j35Ms(void)
{
    return (unsigned long long)GetTickCount64();
}

// --------------------------- 控制极性（可调） -----------------------------
#define SIGN_PITCH   (+1.0)   // 拉杆(输入为负? 拉杆负-1)抬头；若相反改成 -1.0
#define SIGN_ROLL    (+1.0)   // 右压杆右滚；若相反改成 -1.0
#define SIGN_YAW     (-1.0)   // 右舵向右偏航；若相反改成 +1.0

// ★ TVC 矢量喷口方向 (2026-09-29): 用户 EDM 约定
//   arg9020=右发偏航(-1右偏/+1左偏)  arg9021=右发俯仰(-1下偏/+1上偏)
//   arg9023=左发偏航(-1右偏/+1左偏)  arg9022=左发俯仰(-1下偏/+1上偏)
// 物理: 拉杆抬头=尾气向上偏(喷口上偏); 蹬右舵机头右=尾气向右偏(喷口右偏)。
// 若游戏里反了只改这两个宏。
#define SIGN_TVC_PITCH (+1.0) // 拉杆 -> 喷口上偏(+1)
#define SIGN_TVC_YAW   (-1.0) // 右舵(+1) -> 喷口右偏(-1)
#define TVC_DIFF_ROLL  (0.4)  // 差动俯仰辅助滚转系数(0=纯同步)

// ★★★ 2026-10-06 真实推力矢量力矩参数（此前只有喷口动画 9020~9023，不产生操纵力矩）★★★
#define TVC_DEFL_MAX   (0.35) // 喷口最大物理偏角 rad (~20°)
#define TVC_ARM        (5.5)  // 喷口转轴到重心的纵向有效力臂 m
#define TVC_HALF_SPAN  (1.3)  // 双发喷口横向半间距 m（差动俯仰形成滚转力偶用）

// --------------------------- 基本几何与气动参数 ---------------------------
static const double kS     = 62.0;    // 机翼参考面积 m^2  (J-35.lua wing_area)
static const double kB     = 14.7;    // 翼展   m           (J-35.lua wing_span)
static const double kCbar  = 4.22;    // 平均气动弦 m  (S/b 均值弦)
static const double kAR    = kB*kB/kS;
static const double kPi    = 3.14159265358979323846;

// 发动机：双发。表值为整机总推力（对应 J-35.lua SFM engine 表）。
static const double kMilThrustTotal = 166000.0;   // SL 总军推 N
static const double kAbThrustTotal  = 290600.0;   // SL 总加力 N
static const double kEngY           = -0.30;      // 推力线在重心下方 0.30 m -> 加油门抬头力矩
static const double kCmAlpha        = -0.34;      // /rad 俯仰静稳定(CG 前于气动中心); 配平前馈也要用

static const double kFuelMax     = 9500.0;   // 内油上限 kg（J-35.lua M_fuel_max）
static const double kDrySFC      = 2.3e-5;   // 军推耗油率 kg/(N*s)
static const double kAbSFC       = 6.0e-5;   // 加力耗油率 kg/(N*s)

// ------------------------------ 小工具 -------------------------------------
static double clamp(double v, double a, double b){ return v<a?a:(v>b?b:v); }
static double sat01(double v){ return clamp(v,0.0,1.0); }
static double lerpd(double a,double b,double t){ return a+(b-a)*t; }
static double smin(double a,double b){ return a<b?a:b; }
static double smax(double a,double b){ return a>b?a:b; }

struct Vec3 { double x,y,z; Vec3():x(0),y(0),z(0){} Vec3(double X,double Y,double Z):x(X),y(Y),z(Z){} };

// 一维分段线性插值（外推钳制）
static double tableAt(double x, const double xs[], const double ys[], int n)
{
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n-1]) return ys[n-1];
    int i = 0;
    while (i < n-2 && x > xs[i+1]) i++;
    double t = (x - xs[i])/(xs[i+1]-xs[i]);
    return lerpd(ys[i], ys[i+1], t);
}

// ---------------------------- 马赫网格数据 ---------------------------------
static const int    kN = 15;
static const double kM[] = {0.0,0.20,0.40,0.60,0.75,0.85,0.90,0.95,1.00,1.10,1.20,1.40,1.60,1.90,2.40};
// 升力线斜率 /rad
static const double kCla[]= {4.90,4.90,4.75,4.55,4.35,4.05,3.60,3.05,2.65,2.25,2.00,1.80,1.68,1.55,1.45};
// 最大升力系数
static const double kClmax[]={1.60,1.60,1.58,1.50,1.45,1.40,1.30,1.20,1.15,1.10,1.05,1.00,0.90,0.75,0.55};
// 失速攻角 deg
static const double kAStall[]={25.0,25.0,25.0,24.0,23.0,21.0,20.0,19.0,18.0,16.0,17.0,15.0,13.0,12.0,9.0};
// 零升阻力系数
static const double kCd0[]={0.0165,0.0165,0.0165,0.0168,0.0172,0.0182,0.0220,0.0300,0.0380,0.0420,0.0440,0.0430,0.0420,0.0415,0.0410};

// 升力曲线：线性区 -> 圆角逼近 CLmax -> 失速后
static double computeCL(double alpha, double mach, double flapFrac, double gearDown, double & aPostOut)
{
    double Cla  = tableAt(mach,kM,kCla,kN);
    double CLmx = tableAt(mach,kM,kClmax,kN);
    double aStallDeg = tableAt(mach,kM,kAStall,kN);
    // 襟翼增升（仅亚音速有效）
    double fv = smax(0.0, 1.0 - mach/0.80);
    if (flapFrac > 0.001)
    {
        Cla   += 0.70*flapFrac*fv;
        CLmx  += 0.55*flapFrac*fv;
    }
    if (gearDown) CLmx *= 0.985;

    double aLin  = (0.80*CLmx)/Cla;                 // 线性段结束 rad
    double aStall= aStallDeg*(kPi/180.0);           // 失速点 rad
    if (aStall < aLin*1.15) aStall = aLin*1.15;
    double aPost = aStall + 0.35;                    // 完全深失速 rad (~20°)
    double aa = fabs(alpha);
    int sign = (alpha>=0)?1:-1;
    double CL;
    if (aa <= aLin)
        CL = Cla*aa;
    else if (aa <= aStall)
    {
        double t = (aa-aLin)/(aStall-aLin);
        CL = 0.80*CLmx + 0.20*CLmx*t;
    }
    else if (aa <= aPost)
    {
        double t = (aa-aStall)/(aPost-aStall);
        CL = CLmx*(1.0 - 0.55*t);
    }
    else
    {
        // ★ 2026-09-29 大迎角段(支持眼镜蛇/钟摆): 常规升力余量 + 平板正压力模型
        //   平板法向力 CN ≈ 2*sin(a)^2, 分解回升力方向 CL = CN*cos(a)。
        //   叠加后 60° 时 CL≈1.9(仍有可观升力), 90° 时 CL≈0.72(近似纯阻力板),
        //   不再是旧版固定的 0.45*CLmx —— 旧模型 25° 后升力塌掉, 物理上拉不出钟摆。
        double base = CLmx*0.45;
        double sa = sin(aa), ca2 = cos(aa);
        double CN = 2.0*sa*sa;
        double CLfp = CN*ca2;
        CL = base + CLfp;
        if (CL < CLmx*0.45) CL = CLmx*0.45;
    }
    aPostOut = aPost;
    return CL*sign;
}

// ---------------------------- 全局状态 -------------------------------------
static double g_stickPitch = 0.0, g_stickRoll = 0.0, g_stickYaw = 0.0; // 当前杆位(限速平滑后, 供气动/动画)
static double g_reqPitch = 0.0, g_reqRoll = 0.0, g_reqYaw = 0.0;       // 目标杆位(离散键=±1 / 轴=量值)
static double g_pitchAx = 0.0, g_rollAx = 0.0, g_yawAx = 0.0;          // 模拟轴最新原始值
static bool   g_pitchAnalog=false, g_rollAnalog=false, g_yawAnalog=false;
static int    g_pitchKey = 0, g_rollKey = 0, g_yawKey = 0;             // 离散键保持 -1/0/+1
// 杆位速率限制(满行程约0.4s): 键盘按键不再"一碰满舵", 明显降低灵敏度与突发机动
static const double CTL_SLEW_RATE = 2.5;
static double g_pitchTrim = 0.0, g_rollTrim = 0.0;   // 配平(等效杆位; 方向舵配平 2026-09-17 已删)
static double g_trimA    = 0.01;  // ★ 俯仰配平基准(迎角域 rad): 1g 前馈 + 姿态保持(2026-09-18 重写)
static double g_rollAtt  = 0.0;   // 当前滚转角 rad(镜像 g_los.rol, 同上; 自动配平算转弯目标过载用)
static double g_dbgAoa   = 0.0;   // ★ 诊断 2026-09-18: 攻角 rad(日志用)
static double g_dbgNz    = 0.0;   // ★ 诊断 2026-09-18: 法向过载(日志用)
static double g_throttleAxis   = 0.0;   // 0..1（含加力段）

// ★ 2026-09-29 眼镜蛇/钟摆机动状态机 (put_560 触发, EFM 接管俯仰)
//   phase: 0=IDLE 1=PULL 2=HOLD 3=RECOVER
//   timer: 当前相位已用时间 s
static struct CobraState { int phase; double timer; } g_cobra = {0, 0.0};

// ★ 2026-10-06 反推(倒车): 座舱 put_xxx 写 J35_REV_cmd.txt(0关/1开),
//   本 DLL 20Hz 读。只在地面 + 低速生效; 开启时推力反向(负 Fx),
//   让飞机能在地面倒滑。速度>15m/s 自动断开(安全)。
static bool g_revOn = false;
#define J35_REV_MAX_V   15.0    // 反推允许的最大地速 m/s(≈54km/h), 超过自动断开
#define J35_REV_FRAC    0.35    // 反推推力 = 当前总推力的 35%

// 发动机运行/起动状态(g_engState / g_engOn / g_engLit ...) 见下方"发动机模型"一节:
// 冷舱出生时两台都在 ENG_OFF(N1=0, 不燃烧, 无推力), 只有收到 311/312/309 才起转。
static unsigned long long g_lastTgl[4096] = {0};                       // 开关命令去抖
static bool pulseOK(unsigned cmd)                                      // 去抖: 同一命令 350ms 内只认一次
{
    unsigned long long nw = j35Ms();
    if (cmd < 4096)
    {
        if (nw - g_lastTgl[cmd] < 350) return false;
        g_lastTgl[cmd] = nw;
    }
    return true;
}

static double g_gearState  = 1.0;   // 1 放下
static double g_gearTarget = 1.0;
static double g_flapState  = 0.0;   // 0 收起, 1 全放(中间可停起飞位)
static double g_flapTarget = 0.0;
// ★ 2026-09-22 最终确认 襟翼三档手柄(座舱 put_579=起飞位 / put_580=降落位, 互斥):
//   档位目标 0.0=收起 / J35_FLAP_TO_FRAC=起飞位 / 1.0=降落(全放)。
//   气动效应(computeCL/阻力)按 flapFrac 线性缩放, 起飞位取全放的一半。
#define J35_FLAP_TO_FRAC 0.5
// 1=键盘/ACL 改了襟翼档位, 需经座舱参数回调回写 arg579/580;
// 座舱 J35FL1 文件通道报到后清 0(定义在前: ACL 代码也要用)。
static int g_flapExtSync = 0;
static double g_brakeOpen  = 0.0;
static double g_brakeTarget= 0.0;
static double g_wheelBrake = 0.0;
static double g_nwsYaw     = 0.0;   // ★ 2026-09-19 前轮转向角 rad(正 = 机头向左, 见 updateNWS)
static bool   g_nwsLock    = false; // ★ 2026-09-22 前轮转向锁(true=锁定不转向, 由 iCmd437 控制; 座舱 put_574 开关)

static double g_n1[2]   = {0.0,0.0};
static double g_n1cmd[2]= {0.0,0.0};

// ============== 发动机起动状态机: 状态量 (逻辑函数在下面"发动机模型"一节) ==============
// 用户口径: 地面 未起动 = 0 | 地面 已起动 = 20(慢车, N1=0.20) | 空中 = 已起动
// 冷舱出生时两台都在 ENG_OFF —— N1 真的是 0, 不燃烧、不产推力、不耗油;
// 只有收到 311/312/309 起动命令才走 带转 -> 点火 -> 加速到慢车。
enum EngState
{
    ENG_OFF = 0,      // 停车: 不燃烧; 地面停转, 空中只有风车转速
    ENG_CRANK,        // 起动机带转: 不燃烧, N1 -> 0.06
    ENG_LIGHTOFF,     // 点火: 燃烧开始, N1 0.06 -> 0.20(慢车)
    ENG_RUNNING,      // 正常运转: N1 由油门控制
    ENG_SHUTDOWN      // 停车中: 火渐灭, N1 -> 0, 到位后回 ENG_OFF
};

static const double kN1Idle       = 0.20;   // 慢车 N1(与 n1CmdFromAxis(0) 一致 -> 座舱 20%)
static const double kN1Crank      = 0.06;   // 起动机带转转速
static const double kCrankTime    = 3.0;    // 带转到 0.06 的时间(秒)
static const double kLightOffTime = 3.5;    // 点火后加速到慢车的时间(秒), 合计约 6.5s
static const double kShutdownTime = 4.0;    // 停车后的惰转时间(秒)

static int    g_engState[2]    = { ENG_OFF, ENG_OFF };
static double g_engPhase[2]    = { 0.0, 0.0 };      // 当前阶段计时(s)
static double g_engShutFrom[2] = { 0.0, 0.0 };      // 停车时的起始 N1
static bool   g_engLit[2]      = { false, false };  // 燃烧室已点火(推力/油耗/EGT 看这个)
static bool   g_engOn[2]       = { false, false };  // 镜像: 是否处于 ENG_RUNNING(诊断/兼容)
static double g_fuel    = 0.0;       // kg
static double g_burnNow = 0.0;       // 本帧消耗(交还 DCS 减重)

static double g_rho   = 1.225;
static double g_sos   = 340.0;
static double g_alt   = 0.0;
static double g_surfH    = 0.0;   // 正下方地表海拔(ed_fm_set_surface 给, 海面=0)
static double g_surfHObj = 0.0;   // 正下方含物体的地表海拔(在航母上空=甲板高!)
static Vec3  g_velBody;              // 机体系对地速度(由 DCS body 回调给出)
static Vec3  g_windBody;             // 机体系风速
static Vec3  g_windWorld;
static double g_omega[3] = {0,0,0};  // 机体角速度 rad/s
static double g_mass     = 23000.0;
static Vec3  g_cg;
static double g_Ixx=140000.0, g_Iyy=330000.0, g_Izz=390000.0;

// 单帧回调暂存
static Vec3  g_cF;                   // 合力(N，机体系)
static Vec3  g_cM;                   // 合力矩(N*m，机体系)
static double g_aoaDCS=0.0, g_aosDCS=0.0;

// ====================== 战损失控模型 (2026-09-22) ============================
//  DCS 命中机体部件 -> ed_fm_on_damage(Element, integrity): 1=完好, 0=摧毁。
//  任意机身部件(不含起落架轮 83/84/85)完整度跌破 J35_DMG_TRIGGER,
//  或 3 个以上机身部件中度损伤, 即进入【不可逆失控】:
//    双发停车 -> 舵面权限在 J35_DMG_RAMP 秒内衰减到 10% -> 稳定/阻尼力矩减弱
//    -> 叠加不规则滚转/偏航/俯仰发散力矩 -> 侧滑、翻滚、下坠, 无法改出。
#define J35_DMG_TRIGGER   0.50    // 部件完整度阈值: 低于此值直接触发失控
#define J35_DMG_RAMP      4.0     // 失控发展时间 s (0 -> 全失控)
static bool   g_immortal  = false; // 无敌选项: 为 true 时损伤不触发失控
static double g_dmgCell[256];      // 各部件见过的最差完整度(索引与 Damage.lua 对齐)
static bool   g_dmgCellSet[256];   // 对应槽位是否收到过损伤回调
static bool   g_dmgOn = false;     // 已进入失控(不可逆)
static double g_dmgT  = 0.0;       // 失控计时 s
static double g_dmgK  = 0.0;       // 失控程度 0..1 (computeForces 用)
static double g_dmgPh = 0.0;       // 发散力矩相位种子(每次触发不同, 避免千篇一律)

// ---------------------------- 发动机模型 ------------------------------------
// 油门轴 0..1 -> N1 指令: 0.20(慢车)..1.00(军推)..2.00(最大加力)
static double n1CmdFromAxis(double a)
{
    a = clamp(a, 0.0, 1.0);
    if (a <= 0.5)
        return 0.20 + 1.60*a;                    // 0.20..1.00
    return 1.00 + 2.0*(a-0.5);                   // 1.00..2.00
}

// ★ 2026-09-30 座舱油门杆动画映射(arg546): 油门轴 0..1 -> EDM 动画值 0..1
//   用户口径: 慢车(轴0)->0, 行程一半(轴0.25)->0.4, 军推100%(轴0.5)->0.75,
//             过加力卡位->0.78, 全加力(轴1.0)->1.0。
//   轴0.5..0.55 留 0.75->0.78 的小坡当过卡位手感, 避免动画值跳变。
static double throttleLeverAnim(double a)
{
    a = clamp(a, 0.0, 1.0);
    if (a <= 0.25) return 1.6 * a;                       // 0    .. 0.4
    if (a <= 0.50) return 0.4  + 1.4*(a - 0.25);         // 0.4  .. 0.75
    if (a <= 0.55) return 0.75 + 0.6*(a - 0.50);         // 0.75 .. 0.78 (卡位过渡)
    return 0.78 + (0.22/0.45)*(a - 0.55);                // 0.78 .. 1.0
}

// 发动机总推力 N（两台之和；再乘以马赫/高度因子）
static double engineThrustTotal(void)
{
    // ★ 2026-09-13: 只有"已点火"的发动机才产推力 —— 起动机带转(ENG_CRANK)、
    //   停车后火已灭(SHUTDOWN 末段)时 N1 虽然不为 0 也不出力;
    //   否则冷舱出生会自己往前滑, 座舱侧"未起动"的口径也守不住。
    double n1 = 0.5*((g_engLit[0]?g_n1[0]:0.0) + (g_engLit[1]?g_n1[1]:0.0));
    double f;
    if (n1 <= 0.05) f = 0.0;
    else if (n1 < 1.0)
    {
        double x = clamp((n1-0.05)/0.95, 0.0, 1.0);
        f = kMilThrustTotal*(0.01 + 0.03*x + 0.96*x*x);
    }
    else
    {
        double x = clamp(n1-1.0, 0.0, 1.0);
        f = kMilThrustTotal + (kAbThrustTotal-kMilThrustTotal)*(x*(2.0-x));
    }
    // 高度效应
    double altF = pow(smax(g_rho,0.002)/1.225, 0.75);
    // 马赫效应（冲压改善加力、军推在高马赫衰减）
    double mach = smax(1.0e-6, 340.0/g_sos);
    double mfac;
    if (n1 > 1.0)
    {
        // 加力：M<0.9 缓升，0.9~1.6 维持，之后衰减
        mfac = tableAt(mach, (const double[]){0.0,0.6,0.9,1.1,1.4,1.8,2.5},
                              (const double[]){1.00,1.05,1.12,1.10,1.02,0.86,0.68}, 7);
    }
    else
    {
        // 军推：高马赫进气损失
        mfac = tableAt(mach, (const double[]){0.0,0.8,1.0,1.3,1.8,2.5},
                              (const double[]){1.00,0.98,0.95,0.82,0.55,0.35}, 6);
    }
    return f*altF*mfac;
}

// ★ 2026-09-18 修"玩家无加力马赫环": RELATED_THRUST 按 API 口径 = 推力/军推(干推力)
//   最大值: 军推=1.0, 加力段必须 >1.0 (全加力 = kAb/kMil ≈ 1.75)。
//   DCS 渲染管线(engines_nozzles 的 AB 火焰/马赫环)以"相对推力>1"判定加力点燃;
//   旧公式 (n1-0.05)/1.95 全程 ≤1.0, 玩家侧永远点不着火(AI 走 SFM 值正确所以 AI 有环)。
//   曲线与 engineThrustTotal() 的 f/kMilThrustTotal 完全一致(单发口径)。
static double relThrustDry(double n1)
{
    if (n1 <= 0.05) return 0.0;
    if (n1 < 1.0)
    {
        double x = clamp((n1-0.05)/0.95, 0.0, 1.0);
        return 0.01 + 0.03*x + 0.96*x*x;          // 慢车..军推: 0 -> 1.0
    }
    double x = clamp(n1-1.0, 0.0, 1.0);
    return 1.0 + (kAbThrustTotal/kMilThrustTotal - 1.0) * x * (2.0 - x);  // 加力: 1.0 -> ~1.75
}

static double engineFuelFlowTotal(void)
{
    double f = engineThrustTotal();
    double n1 = 0.5*(g_n1[0]+g_n1[1]);
    double sfc = (n1>1.0)?kAbSFC:kDrySFC;
    return f*sfc;
}

// ===================== 发动机起动状态机 (2026-09-13 新增) ====================
// 用户口径: 地面 未起动 = 0 | 地面 已起动 = 20(慢车, N1=0.20) | 空中 = 已起动
//
// 为什么必须放在 DLL 里:
//   旧版 ed_fm_cold_start() 把两台 g_engOn 直接置 true, 于是冷舱出生后 N1 由
//   updateEngines 自己爬到慢车 0.21(EFM 调试日志 n1=0.00 -> 0.03 -> ... -> 0.22,
//   期间没有任何起动命令) —— 座舱侧"未起动"状态必然被吃掉。现在:
//     冷舱出生 -> ENG_OFF, N1 真的是 0, 不燃烧、不产推力、不耗油;
//     收到 311/312/309 -> ENG_CRANK(起动机带转 0 -> 0.06, 不燃烧)
//                       -> ENG_LIGHTOFF(点火, 0.06 -> 0.20 加速到慢车)
//                       -> ENG_RUNNING(油门控制, 慢车 0.20 = 座舱 20%)
//     收到 310/313/314 -> ENG_SHUTDOWN(惰转 4 秒掉到 0) -> ENG_OFF
//   全过程约 6.5 秒: 座舱里看到 RPM 从 0 慢慢爬到 20, 到位后起动按钮跳 1。
//   (状态量 ENG_OFF/CRANK/... 与 kN1*/kCrankTime 等常量声明在上方"全局状态"一节)

// 起动请求(309 双发 / 311 左 / 312 右): 已在起动/已运转则忽略, 天然幂等
static void engStartRequest(int i)
{
    if (g_engState[i] == ENG_CRANK || g_engState[i] == ENG_LIGHTOFF || g_engState[i] == ENG_RUNNING)
    {
        j35Dbg("ENG%d START ignored: already state=%d n1=%.3f", i+1, g_engState[i], g_n1[i]);
        return;
    }
    double from = g_n1[i];
    g_engState[i] = ENG_CRANK;
    g_engPhase[i] = 0.0;
    g_engLit[i]   = false;
    g_engOn[i]    = false;
    if (g_n1[i] > kN1Crank) g_n1[i] = kN1Crank;   // 空中小转速风车 -> 从当前转速接着带转
    j35Dbg("ENG%d START: CRANK from n1=%.3f -> 约 %.1fs 后到慢车 %.2f",
           i+1, from, kCrankTime+kLightOffTime, kN1Idle);
}

// 停车请求(310 双发 / 313 左 / 314 右)
static void engStopRequest(int i)
{
    if (g_engState[i] == ENG_OFF || g_engState[i] == ENG_SHUTDOWN) return;
    g_engShutFrom[i] = smax(g_n1[i], 0.05);
    g_engState[i] = ENG_SHUTDOWN;
    g_engPhase[i] = 0.0;
    g_engOn[i]    = false;
    j35Dbg("ENG%d STOP: SHUTDOWN from n1=%.3f", i+1, g_n1[i]);
}

static void updateEngines(double dt)
{
    double V = sqrt(g_velBody.x*g_velBody.x+g_velBody.y*g_velBody.y+g_velBody.z*g_velBody.z);
    for (int i=0;i<2;i++)
    {
        double cmd = n1CmdFromAxis(g_throttleAxis);   // 油门指令: 慢车 0.20 .. 加力 2.00
        switch (g_engState[i])
        {
        case ENG_OFF:
        {
            g_engLit[i] = false;
            g_engOn[i]  = false;
            g_n1cmd[i]  = 0.0;
            // 空中高速时气流带转(风车, 不燃烧); 地面 V 小 -> N1 归 0
            double windmill = clamp((V-60.0)/240.0, 0.0, 1.0)*0.12;
            g_n1[i] += (windmill - g_n1[i])*(1.0-exp(-dt/6.0));
            break;
        }
        case ENG_CRANK:
        {
            g_engLit[i] = false;
            g_engPhase[i] += dt;
            double t = clamp(g_engPhase[i]/kCrankTime, 0.0, 1.0);
            g_n1[i]    = kN1Crank*t;          // 起动机把转子带起来(不燃烧 -> 不出推力)
            g_n1cmd[i] = kN1Crank;
            if (g_engPhase[i] >= kCrankTime)
            {
                g_engState[i] = ENG_LIGHTOFF;
                g_engPhase[i] = 0.0;
                g_engLit[i]   = true;         // 点火
                j35Dbg("ENG%d LIGHTOFF: n1=%.3f", i+1, g_n1[i]);
            }
            break;
        }
        case ENG_LIGHTOFF:
        {
            g_engLit[i] = true;
            g_engPhase[i] += dt;
            double t = clamp(g_engPhase[i]/kLightOffTime, 0.0, 1.0);
            g_n1[i]    = kN1Crank + (kN1Idle-kN1Crank)*t;   // 加速到慢车
            g_n1cmd[i] = kN1Idle;
            if (g_engPhase[i] >= kLightOffTime)
            {
                g_engState[i] = ENG_RUNNING;
                g_engPhase[i] = 0.0;
                g_engOn[i]    = true;
                j35Dbg("ENG%d RUNNING: 慢车 n1=%.3f (座舱 %.0f%%)", i+1, g_n1[i], g_n1[i]*100.0);
            }
            break;
        }
        case ENG_RUNNING:
        {
            g_engLit[i] = true;
            g_engOn[i]  = true;
            g_n1cmd[i]  = cmd;
            double tau = (cmd>1.0)?1.2:2.2;
            g_n1[i] += (g_n1cmd[i]-g_n1[i])*(1.0-exp(-dt/tau));
            break;
        }
        case ENG_SHUTDOWN:
        {
            g_engPhase[i] += dt;
            double t = clamp(g_engPhase[i]/kShutdownTime, 0.0, 1.0);
            g_n1[i]     = g_engShutFrom[i]*(1.0-t);   // 惰转掉转
            g_engLit[i] = (g_n1[i] > 0.10);           // 火灭了推力/油耗立刻归零
            g_n1cmd[i]  = 0.0;
            if (g_engPhase[i] >= kShutdownTime)
            {
                g_engState[i] = ENG_OFF;
                g_engPhase[i] = 0.0;
                g_n1[i]       = 0.0;
                g_engLit[i]   = false;
                j35Dbg("ENG%d OFF: 停车完成", i+1);
            }
            break;
        }
        default: break;
        }

        // 燃油耗尽 -> 熄火(顺手修掉"油烧光了还在产推力")
        if (g_engLit[i] && g_fuel <= 1.0 && g_engState[i] != ENG_SHUTDOWN)
        {
            j35Dbg("ENG%d FLAMEOUT: fuel=%.1f kg", i+1, g_fuel);
            engStopRequest(i);
        }
    }
}

// ---------------------------- 气动力主计算 ----------------------------------
static void computeForces(double dt)
{
    double V = sqrt(g_velBody.x*g_velBody.x+g_velBody.y*g_velBody.y+g_velBody.z*g_velBody.z);

    // 引擎推力（作用线在重心下方 -> 抬头力矩）
    double FxEng = engineThrustTotal();
    // ★ 2026-10-06 反推(倒车): 地面 + 起落架放下 + 低速时把推力反向,
    //   只保留当前推力的 35%(反推折效); 离地或超速自动断开(防空中误开)。
    if (g_revOn)
    {
        double aglRev = g_alt - g_surfH;
        bool onGnd = (g_gearState > 0.5) && (aglRev < 5.0);
        if (!onGnd || V > J35_REV_MAX_V)
        {
            g_revOn = false;
            j35Dbg("REV: 自动断开(%s)", onGnd ? "超速" : "离地");
        }
        else
        {
            FxEng = -FxEng * J35_REV_FRAC;   // 负推力 = 倒退
        }
    }
    g_cF.x += FxEng;
    g_cM.z += -kEngY*FxEng;

    double burn = engineFuelFlowTotal();
    g_burnNow = smin(burn*dt, g_fuel);
    g_fuel -= g_burnNow;

    if (V < 15.0)   // 低速(<15m/s)不产生气动力，避免地面抖动
        return;

    double mach = V/g_sos;
    double rho  = smax(g_rho, 0.001);
    double qbar = 0.5*rho*V*V;

    // 攻角 / 侧滑（机体系空速 = 对地速度 - 风）
    double ax = g_velBody.x - g_windBody.x;
    double ay = g_velBody.y - g_windBody.y;
    double az = g_velBody.z - g_windBody.z;
    double alpha = atan2(-ay, smax(ax, 0.5));
    double beta  = atan2(az, sqrt(ax*ax+ay*ay));

    double flap = g_flapState;
    double gear = (g_gearState>0.5)?1.0:0.0;
    double sbk  = g_brakeOpen;
    double aLim = 0.5;
    double CL   = computeCL(alpha, mach, flap, gear, aLim);
    // 战损: 翼面/机身破洞 -> 升力损失(全失控时 -30%)
    CL *= 1.0 - 0.30*g_dmgK;

    // 阻力：零升 + 诱导(k*CL^2) + k4 + 附加物
    double e = 0.86;
    double k = 1.0/(kPi*e*kAR);
    // 跨音速诱导阻力隆起
    if (mach>0.85 && mach<1.12)
    {
        double tt = 1.0-fabs(mach-0.985)/0.135;
        if (tt>0) k *= (1.0+0.75*tt);
    }
    double Cd = tableAt(mach,kM,kCd0,kN);
    Cd += k*CL*CL + 0.018*CL*CL*CL*CL;
    // ★ 2026-09-29 大迎角平板阻力: 眼镜蛇减速核心 —— 60°+ 时机身等同迎风平板,
    //   CD_fp = CN*sin(a) ≈ 2*sin^3(a), 90° 时 Cd≈2 产生猛烈减速。
    {
        double aa2 = fabs(alpha);
        if (aa2 > aLim*0.8)   // aLim=aPost(深失速点), 只在失速后生效
        {
            double sa = sin(aa2);
            Cd += 2.0*sa*sa*sa;
        }
    }
    Cd += 0.014*flap;
    Cd += 0.022*gear;
    Cd += 0.09*sbk*sbk + 0.012*sbk;
    Cd += 0.06*g_dmgK;                    // 战损: 结构破洞附加阻力
    double D = Cd*qbar*kS;
    double L = CL*qbar*kS;

    // 单位空速向量 & 升力方向（垂直空速、位于X-Y对称面内）
    Vec3 u(ax/V, ay/V, az/V);
    double ca = cos(alpha);
    Vec3 Ld(sin(alpha), ca, 0.0);
    // 侧力（恢复，配合偏航稳定；β 小量线性）
    double FzSide = -0.012*beta*qbar*kS;   // Cybeta ~ -0.7 /rad 折算

    g_cF.x += -D*u.x + L*Ld.x;
    g_cF.y += -D*u.y + L*Ld.y;
    g_cF.z += -D*u.z + FzSide;

    // --------------------------- 力矩 -------------------------------------
    double qSc = qbar*kS*kCbar;
    double qSb = qbar*kS*kB;
    double alphaClamp = clamp(alpha,-0.9,0.9);
    // 杆位叠加配平后进气动（配平等效为杆位偏置）
    double effP = clamp(g_stickPitch+g_pitchTrim,-1.0,1.0);
    double effR = clamp(g_stickRoll +g_rollTrim, -1.0,1.0);
    double effY = clamp(g_stickYaw,  -1.0,1.0);   // 方向舵配平已删(2026-09-17 应用户要求)
    double stickP = clamp(SIGN_PITCH*effP,-1.0,1.0);
    double stickR = clamp(SIGN_ROLL*effR,-1.0,1.0);
    double stickY = clamp(SIGN_YAW*effY,-1.0,1.0);

    // ★ 2026-09-29 飞控模式: AUTO 限幅(不黑视/F-18手感), OVERRIDE 无限制
    {
        double nzNow = g_dbgNz;  // 上一帧过载(诊断值, 已有)
        stickP = j35FcsLimitPitch(stickP, alpha, nzNow, qbar, g_mass);
        stickR = j35FcsLimitRoll(stickR, g_omega[0]);
    }

    // ★★★ 2026-09-29 眼镜蛇/钟摆机动(put_560 一键触发, EFM 接管俯仰) ★★★
    //   状态机: IDLE -> PULL(满拉杆冲大迎角) -> HOLD(保持90°+) -> RECOVER(推杆回正) -> IDLE
    //   触发条件: 空中 + 速度>180m/s + 襟翼收 + 起落架收 + 无战损
    //   PULL: 绕开 FCS 限幅, 强制满拉杆直到迎角 >= 95°(或超时 4s)
    //   HOLD: 保持满拉杆 0.3s(让迎角冲到峰值)
    //   RECOVER: 推杆(-0.8)直到迎角 < 15°(或超时 3s), 然后恢复玩家控制
    if (g_cobra.phase != 0)
    {
        double alphaDeg = alpha * 57.29578;
        g_cobra.timer += dt;

        // 接管俯仰杆位(绕开 FCS 限幅)
        if (g_cobra.phase == 1)        // PULL
        {
            stickP = 1.0;              // 满拉杆
            // 同时把油门推满加力
            g_throttleAxis = 1.0;
            if (alphaDeg >= 95.0 || g_cobra.timer > 4.0)
            {
                g_cobra.phase = 2;     // -> HOLD
                g_cobra.timer = 0;
                j35Dbg("COBRA: PULL->HOLD aoa=%.1f", alphaDeg);
            }
        }
        else if (g_cobra.phase == 2)   // HOLD
        {
            stickP = 1.0;              // 保持满拉
            g_throttleAxis = 1.0;
            if (g_cobra.timer > 0.3)
            {
                g_cobra.phase = 3;     // -> RECOVER
                g_cobra.timer = 0;
                j35Dbg("COBRA: HOLD->RECOVER aoa=%.1f", alphaDeg);
            }
        }
        else if (g_cobra.phase == 3)   // RECOVER
        {
            stickP = -0.8;             // 推杆回正
            g_throttleAxis = 1.0;
            if (alphaDeg < 15.0 || g_cobra.timer > 3.0)
            {
                g_cobra.phase = 0;     // -> IDLE
                g_cobra.timer = 0;
                j35Dbg("COBRA: RECOVER->IDLE aoa=%.1f", alphaDeg);
            }
        }
    }

    // ★★★ 2026-09-18 重写: 俯仰配平(FBW 式) —— 修"一拉杆就自动压杆 / 爬不上去"
    //   教训(两次实测日志):
    //     v1(09-17) 1g 积分配平: 松杆后若还有剩余过载(nz>1), 它会主动压低机头去卸
    //       过载 -> 玩家感受就是"一拉杆飞机就自动压杆"; 更糟的是它一旦积分到低头
    //       限位就再也解不开(nz 变负后门槛关闭 -> 冻结), 实测 at=-0.150 卡死整场,
    //       飞机被顶到负攻角飞行 -> 机头持续下沉、爬不上去。
    //     v2(本次) 拆成两块, 都【没有主动低头的能力】:
    //       1) 1g 前馈(纯代数, 无状态): 当前质量/速度下 1g 需要的迎角 + 推力线力矩
    //          补偿(加油门抬头 -> 配平要减)。松杆默认就落在 ~1g, 不再零升下坠。
    //       2) 姿态保持(慢积分): 松杆准稳态时用俯仰角速度反推 —— 机头自己往下漂就
    //          加抬头配平把漂移掐断; 漂移停了积分也停, 【不会跟拉杆爬升对着干】。
    //   配平基准 g_trimA 限幅 [-0.03,+0.12] rad(约 -1.7°~+6.9°), 明显低头配平给不出来。
    //   介入条件: 松杆(|stick|<0.05) + 已离地(V>70) + 攻角/角速度正常(准稳态)。
    double alphaFF = 0.0;                                    // 1g 前馈配平迎角
    {
        double ClaNow = smax(tableAt(mach,kM,kCla,kN), 0.5); // 当前马赫升力线斜率
        alphaFF  = (g_mass*9.81)/smax(qbar*kS*ClaNow, 1.0);  // 1g 需要的迎角
        alphaFF += kEngY*FxEng/(qSc*(-kCmAlpha));            // 推力线力矩补偿(抬头->减配平)
        alphaFF  = clamp(alphaFF, 0.0, 0.12)*sat01((V-50.0)/40.0); // 限幅 + 地面滑跑不介入
        // ★ 战损后冻结自动配平: 失控过程中不许 FBW 配平帮飞机"自救"
        if (!g_dmgOn && V > 70.0 && fabs(g_stickPitch) < 0.05 && fabs(alpha) < 0.25 &&
            fabs(g_omega[2]) < 0.30)
        {
            g_trimA -= 0.15*g_omega[2]*dt;                   // 姿态保持: 掐断低头/抬头漂移
        }
        if (!g_dmgOn) g_trimA += (alphaFF - g_trimA)*0.01*dt; // 极慢回中到 1g 前馈值(~100s)
        g_trimA  = clamp(g_trimA, -0.03, 0.12);
    }
    g_dbgAoa = alpha;                                        // ★ 诊断(日志): 攻角
    g_dbgNz  = (L + FxEng*sin(alpha))/smax(g_mass*9.81, 1000.0); // ★ 诊断: 法向过载

    // ★ 战损权限系数: 舵面操纵(副翼/平尾/方向舵) -> 10%, 静稳定/阻尼 -> 45%。
    //   舵打不动 + 自身稳不住, 再叠加下方的不规则发散力矩, 飞机必然侧滑翻滚。
    double ctlF = 1.0 - 0.90*g_dmgK;
    double staF = 1.0 - 0.55*g_dmgK;

    // 俯仰：静稳定 + 全动平尾 + 俯仰阻尼
    // 权限已降到 ~F/A-18 手感: 满杆俯仰率 ~50-60°/s(250m/s), 阻尼增强抑制摆动
    // ★ 2026-09-29 大迎角改造(支持眼镜蛇/钟摆):
    //   1) 静稳定性在失速后衰减 -> 真实飞机深失速时静稳定度大幅下降(机头上仰发散),
    //      旧版 alphaClamp 硬夹 ±0.9 rad 之后还在线性增长恢复力矩, 眼镜布拉不动;
    //   2) 舵权限在大迎角加大 -> 眼镜蛇需要平尾在低速仍有足够力矩推过顶点。
    double aa3 = fabs(alpha);
    double stabFade = 1.0;
    if (aa3 > aLim)   // aLim=aPost 深失速点
    {
        // 深失速后静稳定度按 1/(1+k*(a-aPost)) 衰减, 90° 时约剩 20%
        double over = aa3 - aLim;
        stabFade = 1.0/(1.0 + 3.0*over);
    }
    double elevGain = 1.0;
    if (aa3 > aLim*0.8)
    {
        // 大迎角舵权限增益: 25°~90° 从 1.0 线性升到 3.0
        double t = (aa3 - aLim*0.8) / (1.57 - aLim*0.8);
        elevGain = 1.0 + 2.0*clamp(t, 0.0, 1.0);
    }
    double CmAlpha = kCmAlpha * stabFade;              // /rad 静稳定度, 大迎角衰减
    double alphaEff = clamp(alpha, -1.6, 1.6);         // ★ 放宽迎角夹紧(支持 90°+)
    double Mz_stab = qSc*CmAlpha*(alphaEff - g_trimA)*staF; // 配平基准 g_trimA(迎角域) 见上
    double Mz_elev = qSc*0.18*elevGain*stickP*ctlF;    // 平尾贡献(rad/杆, 大迎角增益)
    double qhat    = g_omega[2]*kCbar/(2.0*V);
    double Mz_damp = qSc*(-19.0)*qhat*staF;              // 俯仰阻尼
    // ★ 眼镜蛇: 大迎角时阻尼增强(抑制摆动发散)
    if (aa3 > aLim*0.8)
    {
        double dampGain = 1.0 + 1.5*clamp((aa3-aLim*0.8)/(1.57-aLim*0.8), 0.0, 1.0);
        Mz_damp *= dampGain;
    }
    g_cM.z += Mz_stab + Mz_elev + Mz_damp;

    // 偏航：方向稳定 + 方向舵 + 偏航阻尼 + 副翼不利偏航
    double CnBeta = 0.42;
    double My_beta = -CnBeta*qSb*beta*staF;
    double My_rud  = -0.10*qSb*stickY*ctlF;              // 方向舵权限(减 37%)
    double rhat    = g_omega[1]*kB/(2.0*V);
    double My_damp = qSb*(-0.55)*rhat*staF;
    double My_ail  = -0.012*qSb*stickR*ctlF;
    g_cM.y += My_beta + My_rud + My_damp + My_ail;

    // 滚转：副翼 + 滚转阻尼 + 上反恢复
    // 满杆滚转收敛 ~200°/s(250m/s), 接近 F/A-18 上限, 不再瞬甩
    double Mx_ail  = 0.105*qSb*stickR*ctlF;              // 副翼权限(减 ~48%)
    double phat    = g_omega[0]*kB/(2.0*V);
    double Mx_damp = qSb*(-1.0)*phat*staF;               // 滚转阻尼(增 25%)
    double Mx_dih  = -0.07*qSb*beta*staF;
    g_cM.x += Mx_ail + Mx_damp + Mx_dih;

    // ★★★ 2026-10-06 真实推力矢量(TVC)力矩 —— 过失速机动的关键 ★★★
    //   此前喷口只播动画(arg 9020~9023)却不产生任何力矩, 失速顶点 qbar→0 时
    //   平尾/副翼/方向舵全部失效 => 只能靠脚本甩眼镜蛇、无法自由转向。
    //   现补上正比于【推力】、与动压无关的三轴力矩: 偏转喷口把推力分出横向分量
    //   F·sin(δ), 在距重心 TVC_ARM 处形成力矩。高速时气动仍主导(TV约为气动1/3),
    //   低速/大迎角时 qbar 塌缩、TV 自然接管 => 顶点仍可俯仰/偏航/滚转,
    //   可做 Herbst(落叶飘/J转)等受控过失速机动。力矩符号一律对齐既有舵面,
    //   保证与平尾/方向舵/副翼同向。
    {
        static const double sKp = sin(TVC_DEFL_MAX)*TVC_ARM;                    // 俯仰系数
        static const double sKy = sin(TVC_DEFL_MAX)*TVC_ARM;                    // 偏航系数
        static const double sKd = sin(TVC_DEFL_MAX)*TVC_DIFF_ROLL*TVC_HALF_SPAN;// 差动滚转系数
        double Ft = FxEng*ctlF;                    // 战损后 TV 权限随之下降
        double sp = clamp(stickP, -1.0, 1.0);      // 喷口有物理偏角极限: 眼镜蛇会把
        double sy = clamp(stickY, -1.0, 1.0);      //   stickP 放到 5(气动靠qSc削掉),
        double sr = clamp(stickR, -1.0, 1.0);      //   但 TV 不随 qbar, 必须限幅防过转
        g_cM.z += Ft*sKp*sp;                       // 俯仰: 与平尾 Mz_elev 同号
        g_cM.y += -Ft*sKy*sy;                      // 偏航: 与方向舵 My_rud 同号(带负号)
        g_cM.x += Ft*sKd*sr;                       // 差动滚转: 与副翼 Mx_ail 同号
    }

    // ★★★ 战损发散力矩 (2026-09-22) ★★★
    //   每轴两个不同频率的正弦叠加 + 击中时刻播种的相位 -> 无规则、不重复的翻滚/
    //   偏航/俯仰发散。幅值与满杆权限同量级(滚转 ~0.10 qSb), 而此时舵面只剩 10%
    //   权限、阻尼只剩 45%, 玩家无论如何操作都压不住, 表现为"被击中后失控下坠"。
    if (g_dmgK > 0.0)
    {
        double t = g_dmgT;
        g_cM.x += g_dmgK*qSb*(0.050*sin(0.90*t+g_dmgPh)      + 0.050*sin(0.23*t+1.3));
        g_cM.y += g_dmgK*qSb*(0.045*sin(0.60*t+g_dmgPh*1.7)  + 0.030*sin(0.17*t));
        g_cM.z += g_dmgK*qSc*(0.060*sin(0.50*t+g_dmgPh*0.6)  + 0.040*sin(0.21*t+1.7));
    }
}

// ----------------------------- 状态动画 -------------------------------------
static void updateConfigAnims(double dt)
{
    // 起落架收放（约5秒）
    double g = g_gearTarget;
    double rate = 1.0/5.0;
    double d = g - g_gearState;
    if (fabs(d) > 1e-4)
    {
        double step = clamp(d, -rate*dt, rate*dt);
        g_gearState += step;
    }
    // 襟翼（约3秒）
    double f = g_flapTarget;
    rate = 1.0/3.0;
    d = f - g_flapState;
    if (fabs(d) > 1e-4)
    {
        double step = clamp(d, -rate*dt, rate*dt);
        g_flapState += step;
    }
    // 减速板（约1.5秒）
    double b = g_brakeTarget;
    rate = 1.0/1.5;
    d = b - g_brakeOpen;
    if (fabs(d) > 1e-4)
    {
        double step = clamp(d, -rate*dt, rate*dt);
        g_brakeOpen += step;
    }
}

// ----------------------- 操纵杆位限速平滑 ----------------------------------
// 键控=±1/轴控=量值 只是"目标", 实际杆位按 CTL_SLEW_RATE 限速逼近目标:
//   * 键盘点按只产生一小段杆量, 而非瞬间满舵 -> 灵敏度显著下降
//   * 长按 ~0.33s 后达到满杆, 极限机动仍可达到, 但不再"突猛"
static void slewCtl(double &cur, double req, double dt)
{
    double step = clamp(req - cur, -CTL_SLEW_RATE*dt, CTL_SLEW_RATE*dt);
    cur += step;
}

static void updateCtlSlew(double dt)
{
    slewCtl(g_stickPitch, g_reqPitch, dt);
    slewCtl(g_stickRoll,  g_reqRoll,  dt);
    slewCtl(g_stickYaw,   g_reqYaw,   dt);
}

// ----------------------- 前轮转向 NWS (2026-09-19 新增, 当日修正方向+拟真) -----
//  方向舵脚蹬直接驱动前轮转向角 (经 ed_fm_get_param 的 ED_FM_SUSPENSION_0_WHEEL_YAW
//  交给 DCS 地面地面物理):
//    * Z 左舵(cmd 201, g_stickYaw=-1) -> 前轮左偏, 机头左转;
//      X 右舵(cmd 203, g_stickYaw=+1) -> 前轮右偏, 机头右转。
//    * 仅起落架放下且速度 < 80 m/s 生效; 收起 / 超速后指令归 0, 前轮回中。
//    * 角度上限随速度收窄(拟真): <=10 m/s 满舵 ±65°(与 FM/config.lua 的
//      yaw_limit = math.rad(65) 同口径); 10~55 m/s 线性收到 ±8°(约 107 节);
//      55~80 m/s 再收到 0 —— 滑行灵活, 起飞高速滑跑只剩微量修正,
//      离地后前轮完全自由定位, 与真实飞机"高速滑跑基本不转前轮"一致。
//    * 作动速率随速度衰减(拟真液压作动筒): 低速 45°/s(65° 全行程 <1.5s),
//      55 m/s 以上 12°/s; 松舵后前轮以同样速率回中(机械拖曳角回中)。
//    * 符号口径: DCS 机体系右手系(+x前 +y上 +z右), 绕 +y 正转 = 机头向左,
//      即 WHEEL_YAW>0 = 前轮左偏; 而 g_stickYaw=-1 是蹬左舵(Z) ——
//      故 J35_NWS_SIGN 取 -1: 左舵 -> WHEEL_YAW>0 -> 机头左。
static const double J35_NWS_SIGN = -1.0;    // 方向符号(见上; 2026-09-19 按实测链路定标)
static bool g_nwsHold = false;              // ★ NWS 接通标志: true=前轮受脚蹬控制
                                             //   (get_param 的 WHEEL_SELF_ATTITUDE 据此切换
                                             //    受控/自由定位; 见下方"混合模式"注释)
// ★ 2026-09-22 前轮转向锁文件通道: 座舱 Lua 写 J35_NWS_cmd.txt
static const char *j35SavedGamesDir(void);   // 前置声明(定义在下方 LOS 段)
//   格式: J35NWS1 <seq> <lock>   lock: 0=可转向 1=锁定
//   座舱 put_574 开关(0=锁定 1=转向) 变化时写, EFM 每 50ms 读一次。
static unsigned long long g_nwsReadT = 0;
static long g_nwsInSeq = 0;
static void j35NwsReadIn(void)
{
    unsigned long long now = j35Ms();
    if (now - g_nwsReadT < 50) return;
    g_nwsReadT = now;
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%sJ35_NWS_cmd.txt", d);
    FILE *fp = fopen(p, "r");
    if (!fp) return;
    char line[64];
    if (fgets(line, sizeof(line), fp))
    {
        char tag[16] = {0}; long seq = 0; int lk = 0;
        if (sscanf(line, "%15s %ld %d", tag, &seq, &lk) == 3 &&
            strcmp(tag, "J35NWS1") == 0 && seq != g_nwsInSeq)
        {
            g_nwsInSeq = seq;
            bool old = g_nwsLock;
            g_nwsLock = (lk != 0);
            if (old != g_nwsLock)
                j35Dbg("NWS lock -> %s (seq=%ld)", g_nwsLock ? "LOCKED" : "unlocked", seq);
        }
    }
    fclose(fp);
}

static void updateNWS(double dt)
{
    j35NwsReadIn();   // ★ 2026-09-22 读座舱 NWS 锁文件(50ms 限频)
    double V = sqrt(g_velBody.x*g_velBody.x + g_velBody.y*g_velBody.y + g_velBody.z*g_velBody.z);
    const double D2R = 0.017453292519943295;

    // ---- 混合模式(拟真, 2026-09-19 二次拟真): ----
    //   起落架放下且 V<80 -> 前轮受控(NWS 接通, SELF_ATTITUDE=0);
    //   收腿中/离地/超速 -> 前轮自由定位(SELF_ATTITUDE=1, 断开 NWS),
    //   轮子随运动方向贴合 —— 真实飞机断开转向后前轮就是自由脚轮。
    g_nwsHold = (g_gearState > 0.5 && V < 80.0 && !g_nwsLock);

    // ---- 角度上限(°): 10 m/s 以下 ±65° -> 55 m/s ±8° -> 80 m/s 0° ----
    double maxDeg;
    if      (V <= 10.0) maxDeg = 65.0;
    else if (V <= 55.0) maxDeg = 65.0 - (65.0 - 8.0) * (V - 10.0) / (55.0 - 10.0);
    else if (V <  80.0) maxDeg =  8.0 * (80.0 - V) / (80.0 - 55.0);
    else                maxDeg =  0.0;

    // 脚蹬量 -> 目标角: 低速线性; 10 m/s 以上叠加平方柔化(小舵量更细腻,
    // 键盘满舵也靠速率限制兜底) —— 拟真 NWS "低速大角度/高速微调"手感。
    double pedal  = clamp(g_stickYaw, -1.0, 1.0);
    double demand = pedal;
    if (V > 10.0)
        demand = pedal * fabs(pedal) * (1.0 - 0.35) + pedal * 0.35; // 平方+线性混合
    double tgt = 0.0;
    if (g_nwsHold)
        tgt = J35_NWS_SIGN * demand * maxDeg * D2R;

    // ---- 作动速率(°/s): 低速 45 -> 55 m/s 以上 12; 回中加速 1.6 倍 ----
    //   (真实拖曳距结构松舵后回中比打轮快; 加上 config.lua 的
    //    wheel_axle_offset=0.08, 物理自回正与作动筒回中叠加)
    double rateDeg;
    if      (V <= 10.0) rateDeg = 45.0;
    else if (V <= 55.0) rateDeg = 45.0 - (45.0 - 12.0) * (V - 10.0) / (55.0 - 10.0);
    else                rateDeg = 12.0;
    double rate = rateDeg * D2R;
    if (fabs(tgt) < fabs(g_nwsYaw) && tgt * g_nwsYaw >= 0.0)
        rate *= 1.6;                                  // 同向回中加速
    g_nwsYaw += clamp(tgt - g_nwsYaw, -rate*dt, rate*dt);
}

// =============================================================================
//                         DCS 接口导出函数
// =============================================================================
J35EFM_API void ed_fm_add_local_force(double &x,double &y,double &z,
                                      double &pos_x,double &pos_y,double &pos_z)
{
    x = g_cF.x; y = g_cF.y; z = g_cF.z;
    pos_x = g_cg.x; pos_y = g_cg.y; pos_z = g_cg.z;
}

J35EFM_API void ed_fm_add_global_force(double &x,double &y,double &z,
                                       double &pos_x,double &pos_y,double &pos_z)
{
    x=0;y=0;z=0;pos_x=g_cg.x;pos_y=g_cg.y;pos_z=g_cg.z;
}

J35EFM_API void ed_fm_add_local_moment(double &x,double &y,double &z)
{
    x = g_cM.x; y = g_cM.y; z = g_cM.z;
}

J35EFM_API void ed_fm_add_global_moment(double &x,double &y,double &z)
{
    x=0;y=0;z=0;
}

// =============================================================================
//  真视线【角度换算】就放在 DLL 里做  (2026-09-14, 与 HUD 桥第二十一版配套)
// -----------------------------------------------------------------------------
//  为什么: 数据链设备(J35DL.lua)每 0.1s 给出"被锁那一架"的【相对机头的水平面方位 +
//          仰角】(从世界坐标算出来的真值, 与姿态无关); HUD 桥原来是自己拿座舱 base_data
//          的 pitch/roll 把它转回机体系 —— 而那个姿态是 30 Hz 的采样值。本 DLL 每个
//          物理步都知道精确姿态, 所以把这一步搬进来:
//
//      J35DL.lua --写--> J35_DL_los.txt --读--> 【本 DLL: 换算】--写--> J35_DL_body.txt
//                                                                        |
//                                             J35HUD_Bridge.lua <--读------+
//
//  为什么不从座舱"下命令"把数据送进来: 实测走不通 —— performClickableAction 发的命令
//  根本不进 EFM (J35_EFM_dbg.log 里从没出现过 c=546, 而 cockpit_click.lua 每帧都在发
//  546 同步油门手柄); dispatch_action 的广播能到, 但带的 value 不可靠(如 c=3001 收到
//  +16.375, 那是个页面按钮)。文件是本模组既有的做法(数据链链路文件 / 头盔链路文件都是
//  文件), 且 DLL 与 Lua 同在 DCS.exe 进程里, 没有跨进程锁的问题。
//
//  换算口径与 J35HUD_Bridge.lua 的 CFG.los_to_body 逐行一致(那边是飞过的版本):
//      ang_to_dir  : f = cos(el)cos(az), u = sin(el), r = cos(el)sin(az)
//      hrz_to_body : 先绕"右"轴俯仰、再绕"前"轴滚转(理由见 Bridge 里那段注释)
//  姿态用 ed_fm_set_current_state_body_axis 给的 yaw/pitch/roll(弧度, 物理步率最新);
//  yaw 用不上 —— 进来的方位角本来就是"相对机头"的。
// =============================================================================

#define J35_LOS_IN_FILE   "J35_DL_los.txt"    // 座舱 Lua 写, 本 DLL 读
#define J35_LOS_OUT_FILE  "J35_DL_body.txt"   // 本 DLL 写, HUD 桥读
#define J35_LOS_MS        20                  // 读+算+写的限频(20ms = 50Hz, 盖过座舱 30Hz)

static struct J35LosState
{
    char  dir[420];             // Saved Games\DCS\J35DATA\ (结尾带反斜杠)
    bool  dirDone, dirOk;
    // ---- 输入(数据链真视线) ----
    long  inSeq;                // 输入文件里的序号: 一变就是"有新数据"
    int   inOk;                 // 1 = 数据链报了"被锁那一架"的真视线
    double laz, lel, rng;       // 方位(rad, 相对机头) / 仰角(rad) / 斜距(m)
    unsigned long long tIn;     // 上次看到新序号的时间(ms)
    // ---- 姿态(rad, 物理步率最新) ----
    double yaw, pit, rol;
    bool   attOk;
    // ---- 输出(机体系方向: f 前 / u 上 / r 右) ----
    double f, u, r;
    double tLast;               // 上次跑本段的时间(ms)
    unsigned long long t0;      // 第一次跑的时刻(ms) -> 心跳基准
    unsigned nRun, nNew, nBad;
} g_los = {0};

// Saved Games 目录: 先认"哪一份里装着本模组"(Mods\aircraft\J-35, 模组必装最稳), 认不到
//   再退到"哪一份里有 Logs\dcs.log"(writedir 必有)。取不到就整段功能自动关闭。
// ★ 2026-09-29: 所有 J35*.txt 通道统一搬到 <SavedGamesDCS>\J35DATA\ 子目录,
//   返回的 dir 已带 "J35DATA\" 后缀; 两个变体都建子目录(无害空目录), 只用探测命中的那份。
static const char *j35SavedGamesDir(void)
{
    if (g_los.dirDone) return g_los.dirOk ? g_los.dir : NULL;
    g_los.dirDone = true;
    const char *up = getenv("USERPROFILE");
    if (!up) { j35Dbg("LOS 目录: getenv(USERPROFILE) 取不到, 真视线换算关闭"); return NULL; }
    static const char *variants[2] = { "DCS", "DCS.openbeta" };
    // 两个变体都先建 J35DATA 子目录(无害空目录)
    for (int v = 0; v < 2; v++)
    {
        char mk[600];
        snprintf(mk, sizeof(mk), "%s\\Saved Games\\%s\\J35DATA", up, variants[v]);
        _mkdir(mk);
    }
    static const char *marks[2]    = { "Mods\\aircraft\\J-35", "Logs\\dcs.log" };
    static const int   markIsDir[2] = { 1, 0 };
    for (int m = 0; m < 2; m++)
    {
        for (int v = 0; v < 2; v++)
        {
            char probe[600];
            snprintf(probe, sizeof(probe), "%s\\Saved Games\\%s\\%s", up, variants[v], marks[m]);
            bool hit;
            if (markIsDir[m])
            {
                DWORD attr = GetFileAttributesA(probe);
                hit = (attr != INVALID_FILE_ATTRIBUTES) && (attr & FILE_ATTRIBUTE_DIRECTORY);
            }
            else
            {
                FILE *fp = fopen(probe, "r");
                hit = (fp != NULL);
                if (fp) fclose(fp);
            }
            if (hit)
            {
                snprintf(g_los.dir, sizeof(g_los.dir), "%s\\Saved Games\\%s\\J35DATA\\", up, variants[v]);
                g_los.dirOk = true;
                j35Dbg("LOS 目录 = %s  (认到 %s)", g_los.dir, marks[m]);
                return g_los.dir;
            }
        }
    }
    j35Dbg("LOS 目录: %s\\Saved Games 下 DCS / DCS.openbeta 都没有标志文件, 真视线换算关闭", up);
    return NULL;
}

// 航向水平系 -> 机体系(与 J35HUD_Bridge.lua 的 ang_to_dir / hrz_to_body 逐行一致)
static void j35AngToDir(double az, double el, double &f, double &u, double &r)
{
    double ce = cos(el);
    f = ce * cos(az);
    u = sin(el);
    r = ce * sin(az);
}
static void j35HrzToBody(double fh, double uh, double rh, double pit, double rol,
                         double &f, double &u, double &r)
{
    double cp = cos(pit), sp = sin(pit);
    double cr = cos(rol), sr = sin(rol);
    double f1 = fh * cp + uh * sp;
    double u1 = uh * cp - fh * sp;
    f = f1;
    u = u1 * cr + rh * sr;
    r = rh * cr - u1 * sr;
}

// 读座舱写来的真视线。座舱那边是"打开-写一行-关", 与本函数没有锁 -> 可能读到半行;
//   解析不全/数值离谱就【沿用上一份】, 并把 bad 计数给日志(不打断服务)。
static void j35LosReadIn(unsigned long long now)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_LOS_IN_FILE);
    FILE *fp = fopen(p, "r");
    if (!fp) return;                       // 文件还没出现(没装/没进座舱/刚开局) -> 什么都不做
    char line[256];
    line[0] = 0;
    if (!fgets(line, sizeof(line), fp)) { fclose(fp); return; }
    fclose(fp);

    char tag[16] = {0};
    long seq = 0;
    double ok = 0.0, laz = 0.0, lel = 0.0, rng = 0.0;
    if (sscanf(line, "%15s %ld %lf %lf %lf %lf", tag, &seq, &ok, &laz, &lel, &rng) != 6)
    { g_los.nBad++; return; }
    if (strcmp(tag, "J35LOS1") != 0)             { g_los.nBad++; return; }
    if (!(laz > -3.60 && laz < 3.60))            { g_los.nBad++; return; }   // |方位| < 206°
    if (!(lel > -1.70 && lel < 1.70))            { g_los.nBad++; return; }   // 仰角在 ±97° 内

    if (seq != g_los.inSeq)
    {
        g_los.inSeq = seq;
        g_los.tIn   = now;
        g_los.nNew++;
    }
    g_los.laz = laz;
    g_los.lel = lel;
    g_los.rng = rng;
    g_los.inOk = (ok > 0.5) ? 1 : 0;
}

// 把换算结果写给 HUD 桥:
//   J35BDY1 <输入序号> <ok> <f> <u> <r> <距上次新数据的毫秒> <本机俯仰°> <本机滚转°> <心跳ms>
//   最后两列是诊断(核对 DLL 的姿态与座舱看到的是不是同一套); 心跳列一不动 = 本 DLL 没在跑。
static void j35LosWriteOut(unsigned long long now)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_LOS_OUT_FILE);
    FILE *fp = fopen(p, "w");
    if (!fp) return;
    double age = g_los.tIn ? (double)(now - g_los.tIn) : 1.0e9;
    // ok 得把"姿态到位没有"一起算上: 拿不到姿态就换不出机体系方向, 不能让座舱把 f/u/r=0 当真值
    int okOut = (g_los.inOk && g_los.attOk) ? 1 : 0;
    fprintf(fp, "J35BDY1 %ld %d %.6f %.6f %.6f %.1f %.4f %.4f %llu\n",
            g_los.inSeq, okOut, g_los.f, g_los.u, g_los.r, age,
            g_los.pit * (180.0/kPi), g_los.rol * (180.0/kPi),
            (unsigned long long)(now - g_los.t0));
    fclose(fp);
}

static void j35LosLog(unsigned long long now)
{
    static unsigned long long tLog = 0;
    static int lastSt = -1;
    int st = g_los.inOk ? (g_los.attOk ? 2 : 1) : 0;
    if (st == lastSt && (now - tLog) < 1000) return;
    tLog = now;
    lastSt = st;
    double age = g_los.tIn ? (double)(now - g_los.tIn) : -1.0;
    j35Dbg("LOS %s seq=%ld ok=%d laz=%+.2f lel=%+.2f rng=%.0fm age=%.0fms | att pit=%+.2f rol=%+.2f (raw %+.4f/%+.4f) | body f/u/r=%+.4f/%+.4f/%+.4f new=%u bad=%u",
           g_los.dirOk ? "in:" : "in(无目录):", g_los.inSeq, g_los.inOk,
           g_los.laz * (180.0/kPi), g_los.lel * (180.0/kPi), g_los.rng, age,
           g_los.pit * (180.0/kPi), g_los.rol * (180.0/kPi), g_los.pit, g_los.rol,
           g_los.f, g_los.u, g_los.r, (unsigned)g_los.nNew, (unsigned)g_los.nBad);
}

// 每物理步调一次(限频): 读输入 -> 换算 -> 写输出
static void j35LosUpdate(void)
{
    unsigned long long now = j35Ms();
    if (g_los.t0 == 0) g_los.t0 = now;
    if (g_los.tLast > 0.0 && (now - (unsigned long long)g_los.tLast) < J35_LOS_MS) return;
    g_los.tLast = (double)now;
    g_los.nRun++;

    j35LosReadIn(now);

    if (g_los.inOk && g_los.attOk)
    {
        double hf, hu, hr;
        j35AngToDir(g_los.laz, g_los.lel, hf, hu, hr);
        j35HrzToBody(hf, hu, hr, g_los.pit, g_los.rol, g_los.f, g_los.u, g_los.r);
    }
    else if (!g_los.inOk)
    {
        // 没锁定 / 断链: 输出清零 —— 座舱侧靠 ok=0 判回退, 这里也别留上一架的残值
        g_los.f = g_los.u = g_los.r = 0.0;
    }

    j35LosWriteOut(now);
    j35LosLog(now);
}

// =============================================================================
//  对地吊舱(EOTS)通道  (2026-09-15 新增)
// -----------------------------------------------------------------------------
//  与上面"真视线"通道【同一套机关】, 只是服务对地目标(吊舱锁定的地面点):
//
//      座舱 Lua --写--> J35_DL_pod.txt --读--> 【本 DLL: 换算】--写--> J35_DL_pod_body.txt
//                                                                        |
//                                             座舱 Lua(HUD/MFD) <--读------+
//
//  与真视线通道的区别:
//    * 输入多一列 mode: 0=吊舱关  1=待机  2=跟踪  3=跟踪+激光
//    * 只有 mode>=2 且 ok=1 才输出方向(f/u/r), 其余时刻 okOut=0 且方向清零
//      (座舱画"对地目标框"看 okOut, 画"激光"标记看 mode==3)
//    * 输出多一列 rng(斜距, 米, 原样转发), 座舱直接拿来标距离
//
//  输入行(座舱写, 与 los_write 同款口径: az/el = 相对机头的水平航向系, 弧度):
//      J35POD1 <seq> <mode> <ok> <az> <el> <rng>
//  输出行(本 DLL 写):
//      J35PDB1 <seq> <mode> <okOut> <f> <u> <r> <rng> <age_ms> <pit°> <rol°> <心跳ms>
//
//  姿态/目录直接复用真视线通道的 g_los(两个通道同机同帧, 没必要各存一份)。
// =============================================================================

#define J35_POD_IN_FILE   "J35_DL_pod.txt"      // 座舱 Lua 写, 本 DLL 读
#define J35_POD_OUT_FILE  "J35_DL_pod_body.txt" // 本 DLL 写, 座舱读
#define J35_POD_MS        20                    // 限频 20ms = 50Hz(与真视线同拍)

static struct J35PodState
{
    // ---- 输入(吊舱给的对地真视线) ----
    long   inSeq;               // 序号: 一变就是"有新数据"
    int    inOk;                // 1 = 吊舱报了有效地面目标
    int    mode;                // 0 关 / 1 待机 / 2 跟踪 / 3 跟踪+激光
    double laz, lel, rng;       // 方位(rad, 相对机头) / 仰角(rad) / 斜距(m)
    unsigned long long tIn;     // 上次看到新序号的时间(ms)
    // ---- 输出(机体系方向: f 前 / u 上 / r 右) ----
    double f, u, r;
    double tLast;               // 上次跑本段的时间(ms)
    unsigned long long t0;      // 第一次跑的时刻(ms) -> 心跳基准
    unsigned nRun, nNew, nBad;
    int    lastLogMode;         // 只在模式变化时打日志
} g_pod = {0};

// 读座舱写来的吊舱视线(半行/离谱值 -> 沿用上一份, 只计 bad, 不打断服务)
static void j35PodReadIn(unsigned long long now)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_POD_IN_FILE);
    FILE *fp = fopen(p, "r");
    if (!fp) return;                       // 文件没出现(吊舱功能没接) -> 什么都不做
    char line[256];
    line[0] = 0;
    if (!fgets(line, sizeof(line), fp)) { fclose(fp); return; }
    fclose(fp);

    char tag[16] = {0};
    long seq = 0;
    int  mode = 0;
    double ok = 0.0, laz = 0.0, lel = 0.0, rng = 0.0;
    if (sscanf(line, "%15s %ld %d %lf %lf %lf %lf",
               tag, &seq, &mode, &ok, &laz, &lel, &rng) != 7) { g_pod.nBad++; return; }
    if (strcmp(tag, "J35POD1") != 0)       { g_pod.nBad++; return; }
    if (mode < 0 || mode > 3)              { g_pod.nBad++; return; }
    if (!(laz > -3.60 && laz < 3.60))      { g_pod.nBad++; return; }   // |方位| < 206°
    if (!(lel > -1.70 && lel < 1.70))      { g_pod.nBad++; return; }   // 仰角在 ±97° 内

    if (seq != g_pod.inSeq)
    {
        g_pod.inSeq = seq;
        g_pod.tIn   = now;
        g_pod.nNew++;
    }
    g_pod.laz  = laz;
    g_pod.lel  = lel;
    g_pod.rng  = rng;
    g_pod.mode = mode;
    g_pod.inOk = (ok > 0.5) ? 1 : 0;
}

// 把换算结果写给座舱。okOut 把"姿态到位 + 吊舱在跟踪"一起算上:
//   没在跟踪(mode<2)时 f/u/r 必须是 0, 免得座舱把残值当地面目标画出来。
static void j35PodWriteOut(unsigned long long now)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_POD_OUT_FILE);
    FILE *fp = fopen(p, "w");
    if (!fp) return;
    double age = g_pod.tIn ? (double)(now - g_pod.tIn) : 1.0e9;
    int okOut = (g_pod.inOk && g_los.attOk && g_pod.mode >= 2) ? 1 : 0;
    fprintf(fp, "J35PDB1 %ld %d %d %.6f %.6f %.6f %.1f %.1f %.4f %.4f %llu\n",
            g_pod.inSeq, g_pod.mode, okOut, g_pod.f, g_pod.u, g_pod.r, g_pod.rng, age,
            g_los.pit * (180.0/kPi), g_los.rol * (180.0/kPi),
            (unsigned long long)(now - g_pod.t0));
    fclose(fp);
}

// 只在模式变化时打一行(50Hz 刷日志没意义)
static void j35PodLog(void)
{
    if (g_pod.mode == g_pod.lastLogMode) return;
    g_pod.lastLogMode = g_pod.mode;
    static const char *names[4] = { "关", "待机", "跟踪", "跟踪+激光" };
    j35Dbg("POD 模式 -> %d(%s) seq=%ld ok=%d laz=%+.2f° lel=%+.2f° rng=%.0fm | body f/u/r=%+.4f/%+.4f/%+.4f new=%u bad=%u",
           g_pod.mode, names[g_pod.mode & 3], g_pod.inSeq, g_pod.inOk,
           g_pod.laz * (180.0/kPi), g_pod.lel * (180.0/kPi), g_pod.rng,
           g_pod.f, g_pod.u, g_pod.r, (unsigned)g_pod.nNew, (unsigned)g_pod.nBad);
}

// 每物理步调一次(限频): 读输入 -> 换算 -> 写输出
static void j35PodUpdate(void)
{
    unsigned long long now = j35Ms();
    if (g_pod.t0 == 0) g_pod.t0 = now;
    if (g_pod.tLast > 0.0 && (now - (unsigned long long)g_pod.tLast) < J35_POD_MS) return;
    g_pod.tLast = (double)now;
    g_pod.nRun++;

    // j35PodReadIn(now);  // 2026-09-27 停用: TGP 核心并入本 DLL, g_pod 改由 j35TgpFeedPod 内部灌

    if (g_pod.inOk && g_los.attOk && g_pod.mode >= 2)
    {
        double hf, hu, hr;
        j35AngToDir(g_pod.laz, g_pod.lel, hf, hu, hr);
        j35HrzToBody(hf, hu, hr, g_los.pit, g_los.rol, g_pod.f, g_pod.u, g_pod.r);
    }
    else
    {
        // 没在跟踪 / 断链 / 姿态没到位: 方向清零(okOut=0 座舱自己会回退)
        g_pod.f = g_pod.u = g_pod.r = 0.0;
    }

    j35PodWriteOut(now);
    j35PodLog();
}

// =============================================================================
//  HUD STT 锁定框世界系换算通道  (2026-09-28 新增)
// -----------------------------------------------------------------------------
//  与上面"真视线"通道同一套机关, 但多走一趟"机头系 -> 世界系 -> 机体系"完整闭环,
//  让锁定框钉在世界里的目标上、不随飞机姿态漂移:
//
//      座舱 Lua(J35HUD_Bridge) --写--> J35_STT_req.txt --读--> 【本 DLL: 换算】--写--> J35_STT_body.txt
//                                                                              |
//                                                              座舱 Lua(HUD) <--读------+
//
//  与真视线通道的区别:
//    * 输入是雷达 STT 的(az, el, rng) 机头系角度(rad), 先经 J35HM::HudAzElToWorldDir
//      转世界系单位视线(钉死), 再经 J35HM::WorldToBodyDir 转回机体系投屏;
//    * 姿态用 g_los.yaw/pit/roll(物理步率最新), 不再走 Export 的 LoGetADIPitchBankYaw;
//    * 输出是机体系方向(f/u/r) + 距离, 座舱直接投 HUD(不再二次旋转)。
//  文件协议:
//    请求  J35_STT_req.txt  = "J35STT1 <seq> <az_rad> <el_rad> <rng_m>"
//    应答  J35_STT_body.txt = "J35STB1 <seq> <ok> <f> <u> <r> <rng>"
// =============================================================================

#define J35_STT_IN_FILE   "J35_STT_req.txt"
#define J35_STT_OUT_FILE  "J35_STT_body.txt"
#define J35_STT_MS        20   // 限频 20ms = 50Hz

static struct J35SttState
{
    long  inSeq;
    int   inOk;
    double az, el, rng;       // 输入: 机头系角度 rad + 距离 m
    unsigned long long tIn;
    double f, u, r;           // 输出: 机体系方向(前/上/右)
    unsigned long long t0;
    double tLast;
    unsigned nRun, nNew, nBad;
} g_stt = {0};

// 读座舱写来的 STT 机头系角度
static void j35SttReadIn(unsigned long long now)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_STT_IN_FILE);
    FILE *fp = fopen(p, "r");
    if (!fp) return;
    char line[256]; line[0] = 0;
    if (!fgets(line, sizeof(line), fp)) { fclose(fp); return; }
    fclose(fp);

    char tag[16] = {0};
    long seq = 0;
    double ok = 0.0, az = 0.0, el = 0.0, rng = 0.0;
    if (sscanf(line, "%15s %ld %lf %lf %lf %lf", tag, &seq, &ok, &az, &el, &rng) != 6)
    { g_stt.nBad++; return; }
    if (strcmp(tag, "J35STT1") != 0) { g_stt.nBad++; return; }
    if (!(az > -3.60 && az < 3.60))  { g_stt.nBad++; return; }
    if (!(el > -1.70 && el < 1.70))  { g_stt.nBad++; return; }

    if (seq != g_stt.inSeq)
    {
        g_stt.inSeq = seq;
        g_stt.tIn   = now;
        g_stt.nNew++;
    }
    g_stt.az   = az;
    g_stt.el   = el;
    g_stt.rng  = rng;
    g_stt.inOk = (ok > 0.5) ? 1 : 0;
}

// 写换算结果给 HUD 桥
static void j35SttWriteOut(unsigned long long now)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_STT_OUT_FILE);
    FILE *fp = fopen(p, "w");
    if (!fp) return;
    int okOut = (g_stt.inOk && g_los.attOk) ? 1 : 0;
    fprintf(fp, "J35STB1 %ld %d %.6f %.6f %.6f %.1f\n",
            g_stt.inSeq, okOut, g_stt.f, g_stt.u, g_stt.r, g_stt.rng);
    fclose(fp);
}

static void j35SttLog(unsigned long long now)
{
    static unsigned long long tLog = 0;
    static int lastSt = -1;
    int st = g_stt.inOk ? (g_los.attOk ? 2 : 1) : 0;
    if (st == lastSt && (now - tLog) < 1000) return;
    tLog = now; lastSt = st;
    double age = g_stt.tIn ? (double)(now - g_stt.tIn) : -1.0;
    j35Dbg("STT %s seq=%ld ok=%d az=%+.2f el=%+.2f rng=%.0fm age=%.0fms | att y/p/r=%+.2f/%+.2f/%+.2f | body f/u/r=%+.4f/%+.4f/%+.4f new=%u bad=%u",
           g_los.dirOk ? "in:" : "in(无目录):", g_stt.inSeq, g_stt.inOk,
           g_stt.az * (180.0/kPi), g_stt.el * (180.0/kPi), g_stt.rng, age,
           g_los.yaw * (180.0/kPi), g_los.pit * (180.0/kPi), g_los.rol * (180.0/kPi),
           g_stt.f, g_stt.u, g_stt.r, (unsigned)g_stt.nNew, (unsigned)g_stt.nBad);
}

// 每物理步调一次(限频): 读输入 -> J35HM 世界系闭环 -> 写输出
static void j35SttUpdate(void)
{
    unsigned long long now = j35Ms();
    if (g_stt.t0 == 0) g_stt.t0 = now;
    if (g_stt.tLast > 0.0 && (now - (unsigned long long)g_stt.tLast) < J35_STT_MS) return;
    g_stt.tLast = (double)now;
    g_stt.nRun++;

    j35SttReadIn(now);

    if (g_stt.inOk && g_los.attOk)
    {
        J35HM::Attitude att;
        att.heading_deg = g_los.yaw * (180.0/kPi);
        att.pitch_deg   = g_los.pit * (180.0/kPi);
        att.roll_deg    = g_los.rol * (180.0/kPi);

        // 1) 机头系(az,el) -> 世界系单位视线(钉死目标)
        J35HM::Vec3 w = J35HM::HudAzElToWorldDir(g_stt.az, g_stt.el, att);
        // 2) 世界系 -> 机体系(前/上/右)
        J35HM::Vec3 b = J35HM::WorldToBodyDir(w, att);
        g_stt.f = b.x;
        g_stt.u = b.y;
        g_stt.r = b.z;
    }
    else if (!g_stt.inOk)
    {
        g_stt.f = g_stt.u = g_stt.r = 0.0;
    }

    j35SttWriteOut(now);
	j35SttLog(now);
}

// =============================================================================
//  对地吊舱核心 (TGP)                                  2026-09-27 并入 EFM
// -----------------------------------------------------------------------------
//  前身是桌面独立 LiteningTGP.dll(ffi 方案), 现整体并入本 DLL:
//    * 云台/增稳/跟踪/激光测距全在 EFM 侧闭环: 姿态直接用 g_los(真增稳),
//      斜距用 g_alt-g_surfH 平地求交(限 40km), 座舱 TGP/device.lua 只是命令/状态桥
//    * 输出直接灌 g_pod(替代旧 J35_DL_pod.txt 文件输入, j35PodReadIn 已停用):
//      HUD 对地目标框照旧读 g_pod (podcam 渲染钩子已删)
//
//  命令(座舱写 <SavedGames>\J35DATA\J35_TGP_cmd.txt, seq 变化才执行, 纯事件式):
//    J35TGPC <seq> <sx> <sy> <laser> <trackTgl> <zoomDir> <pwrTgl> <polTgl>
//      sx/sy: 云台步进方向 ±1(0=无; 步进中自动脱锁)   laser: -1=不变 0/1=熄/照
//      trackTgl/pwrTgl/polTgl: 1=翻转                  zoomDir: +1 变窄 -1 变宽
//  状态(本 DLL 10Hz 写 <SavedGames>\J35DATA\J35_TGP_out.txt, 座舱读了设 FLIR_* 参数):
//    J35TGPS <power> <az> <el> <fovRad> <pol> <laser> <rng> <tracking>
// =============================================================================

#define J35_TGP_CMD_FILE "J35_TGP_cmd.txt"
#define J35_TGP_OUT_FILE "J35_TGP_out.txt"
#define J35_TGP_OUT_MS   100

static const double TGP_AZ_LIM   = 170.0*kPi/180.0;
static const double TGP_EL_MIN   = -89.0*kPi/180.0;
static const double TGP_EL_MAX   =  20.0*kPi/180.0;
static const double TGP_SLEW_STP =   1.2*kPi/180.0;  // 每次按键步进(沿用旧 device.lua)
static const double TGP_MAX_RATE =   1.0;            // 云台限速 rad/s
static const double TGP_TRACK_G  =   4.0;            // 跟踪比例增益 1/s
static const double TGP_TRACK_RT =   0.5;            // 跟踪最大角速度 rad/s
static const double TGP_RNG_MAX  =   40000.0;
static const double TGP_FOV[3]   = { 28.6*kPi/180.0, 8.0*kPi/180.0, 2.0*kPi/180.0 }; // 宽/中/窄

static struct J35TgpState
{
    int    power, pol;                 // pol: 0=白热 1=黑热
    double az, el;                     // 当前云台角(rad, 相对机头, az+右 el+上)
    double cmdAz, cmdEl;               // 目标角
    int    tracking, laser;
    double trackAz, trackEl;           // 区域锁定点
    int    fovIdx;
    double gain, level, range;
    double prevPit, prevRol; int prevValid;
    long   lastSeq;
    unsigned long long tOut;
} g_tgp;

// 座舱命令(事件式): 只在 seq 变化时执行一次
static void j35TgpReadCmd(void)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_TGP_CMD_FILE);
    FILE *fp = fopen(p, "r");
    if (!fp) return;
    char line[256]; line[0] = 0;
    if (!fgets(line, sizeof(line), fp)) { fclose(fp); return; }
    fclose(fp);

    char tag[16] = {0};
    long seq = 0;
    double sx = 0.0, sy = 0.0;
    int laser = -1, tgl = 0, zoom = 0, pwr = 0, pol = 0;
    if (sscanf(line, "%15s %ld %lf %lf %d %d %d %d %d",
               tag, &seq, &sx, &sy, &laser, &tgl, &zoom, &pwr, &pol) != 9) return;
    if (strcmp(tag, "J35TGPC")) return;
    if (seq == g_tgp.lastSeq) return;
    g_tgp.lastSeq = seq;

    if (pwr)
    {
        g_tgp.power = !g_tgp.power;
        if (!g_tgp.power)
        {
            g_tgp.laser = 0; g_tgp.tracking = 0;
            g_tgp.az = g_tgp.cmdAz = 0.0;
            g_tgp.el = g_tgp.cmdEl = -12.0*kPi/180.0;   // 关机归中略下俯
        }
    }
    if (!g_tgp.power) return;

    if (g_tgp.tracking && (sx != 0.0 || sy != 0.0))
        g_tgp.tracking = 0;                             // 手动动云台 -> 脱锁(真机惯例)
    g_tgp.cmdAz = clamp(g_tgp.cmdAz + sx*TGP_SLEW_STP, -TGP_AZ_LIM, TGP_AZ_LIM);
    g_tgp.cmdEl = clamp(g_tgp.cmdEl + sy*TGP_SLEW_STP, TGP_EL_MIN, TGP_EL_MAX);

    if (zoom > 0) g_tgp.fovIdx = (g_tgp.fovIdx < 2) ? g_tgp.fovIdx + 1 : 2;
    if (zoom < 0) g_tgp.fovIdx = (g_tgp.fovIdx > 0) ? g_tgp.fovIdx - 1 : 0;
    if (pol) g_tgp.pol ^= 1;

    if (tgl)
    {
        g_tgp.tracking = !g_tgp.tracking;
        if (g_tgp.tracking)                             // 区域锁定: 锁住当前指向
        {
            g_tgp.trackAz = g_tgp.az;
            g_tgp.trackEl = g_tgp.el;
        }
    }
    if (laser >= 0) g_tgp.laser = laser;
}

static void j35TgpUpdate(double dt)
{
    if (!g_tgp.power) { g_tgp.range = 0; g_tgp.prevValid = 0; return; }

    // 增稳: 机体姿态增量反向补偿目标角, 保持视线世界固定(方向反了改符号)
    if (g_los.attOk)
    {
        if (g_tgp.prevValid)
        {
            g_tgp.cmdEl = clamp(g_tgp.cmdEl - (g_los.pit - g_tgp.prevPit), TGP_EL_MIN, TGP_EL_MAX);
            g_tgp.cmdAz = clamp(g_tgp.cmdAz - (g_los.rol - g_tgp.prevRol), -TGP_AZ_LIM, TGP_AZ_LIM);
        }
        g_tgp.prevPit = g_los.pit; g_tgp.prevRol = g_los.rol; g_tgp.prevValid = 1;
    }
    else g_tgp.prevValid = 0;

    // 跟踪(区域锁定): 目标角向锁定点限速拉回
    if (g_tgp.tracking)
    {
        double ex = g_tgp.trackAz - g_tgp.az;
        double ey = g_tgp.trackEl - g_tgp.el;
        g_tgp.cmdAz = clamp(g_tgp.cmdAz + clamp(ex*TGP_TRACK_G, -TGP_TRACK_RT, TGP_TRACK_RT)*dt, -TGP_AZ_LIM, TGP_AZ_LIM);
        g_tgp.cmdEl = clamp(g_tgp.cmdEl + clamp(ey*TGP_TRACK_G, -TGP_TRACK_RT, TGP_TRACK_RT)*dt, TGP_EL_MIN, TGP_EL_MAX);
    }

    // 当前角限速跟随目标角
    double st = TGP_MAX_RATE * dt;
    g_tgp.az += clamp(g_tgp.cmdAz - g_tgp.az, -st, st);
    g_tgp.el += clamp(g_tgp.cmdEl - g_tgp.el, -st, st);

    // 激光斜距: 视线(机体系 前/下/右)按姿态转到大地系, 平地求交
    if (!g_tgp.laser || !g_los.attOk) { g_tgp.range = 0; return; }
    double cf = cos(g_tgp.el)*cos(g_tgp.az);
    double dd = -sin(g_tgp.el);
    double dr = cos(g_tgp.el)*sin(g_tgp.az);
    double d1 = -cf*sin(g_los.pit) + dd*cos(g_los.pit);
    double d2 = d1*cos(g_los.rol) + dr*sin(g_los.rol);
    if (d2 < 0.01) { g_tgp.range = 0; return; }        // 打天/贴地平线 -> 无回波
    double agl = g_alt - g_surfH;
    if (agl < 0.0) agl = 0.0;
    g_tgp.range = clamp(agl / d2, 0.0, TGP_RNG_MAX);
}

// 灌给 pod 通道: HUD 目标框照旧读 g_pod
static void j35TgpFeedPod(unsigned long long now)
{
    g_pod.inSeq++;
    g_pod.tIn  = now;
    g_pod.laz  = g_tgp.az;
    g_pod.lel  = g_tgp.el;
    g_pod.rng  = g_tgp.range;
    g_pod.inOk = (g_tgp.power && g_tgp.tracking) ? 1 : 0;
    g_pod.mode = !g_tgp.power ? 0 : (g_tgp.tracking ? (g_tgp.laser ? 3 : 2) : 1);
}

static void j35TgpWriteOut(void)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_TGP_OUT_FILE);
    FILE *fp = fopen(p, "w");
    if (!fp) return;
    fprintf(fp, "J35TGPS %d %.6f %.6f %.6f %d %d %.1f %d\n",
            g_tgp.power, g_tgp.az, g_tgp.el, TGP_FOV[g_tgp.fovIdx],
            g_tgp.pol, g_tgp.laser, g_tgp.range, g_tgp.tracking);
    fclose(fp);
}

static void j35TgpTick(double dt)
{
    unsigned long long now = j35Ms();
    j35TgpReadCmd();
    j35TgpUpdate(dt);
    j35TgpFeedPod(now);
    if (now - g_tgp.tOut >= J35_TGP_OUT_MS)
    {
        g_tgp.tOut = now;
        j35TgpWriteOut();
    }
}


// =============================================================================
//                  定速巡航 put_584 / 锁定 put_585 (座舱 Lua <-> EFM 文件通道)
// -----------------------------------------------------------------------------
// 座舱侧: SYSTEM/cockpit_click.lua 点击 put_584 / put_585 时写
//     <SavedGames>\J35DATA\J35_AH_cmd.txt
//   J35AH1 <seq> <hold> <lock>
//     hold: 0/1 定速巡航期望状态(点一下取反); lock: 0/1 锁定(锁定时手动油门一律忽略)
//     (方向舵配平第4字段 2026-09-17 已删 —— 整个方向舵配平功能应用户要求移除)
//
//   ★ 手动油门规则(用户口径):
//     锁定时  : 轴 2004/2005/2006、键 161/162/163/164/1032/1033 全部忽略(油门锁死);
//     未锁定时: 任何手动油门输入 -> 立刻断开定速巡航, 油门还给飞行员。
//     (油门轴要做"真动了"判断: 引擎可能重复派发同一轴值, 不能一收到就当手动。)
//
//   hold 0->1 的那次接合: 记下当前真空速为目标速度; 双发全熄火自动断开。
//
//   回写 <SavedGames>\J35DATA\J35_AH_out.txt (20Hz):
//   J35AHO1 <seq> <hold> <lock> <thr> <vt> <v>
//     hold/lock = 本 DLL 的【实际】状态(手动油门断开后 hold=0, 座舱按它弹回按钮);
//     vt = 目标速度 m/s(未接合为 0), v = 当前真空速 m/s, thr = 当前油门 0..1。
//
//   控制律: PI + 输出限速(发动机有滞后, 抡太猛会绕着目标速度振荡),
//   只写 g_throttleAxis, 不碰杆。
// =============================================================================

#define J35_AH_IN_FILE   "J35_AH_cmd.txt"   // 座舱 Lua 写, 本 DLL 读
#define J35_AH_OUT_FILE  "J35_AH_out.txt"   // 本 DLL 写, 座舱读
#define J35_AH_MS        50                 // 读+回写限频(20Hz)

static struct J35AhState
{
    int    hold, holdReq, lastHoldReq;      // 实际 / 座舱要的 / 上一次要的(只在 0->1 边沿接合)
    int    lock, lockReq;
    double vt;                              // 目标真空速 m/s
    double thr0;                            // 接合时的油门(PI 基准)
    double integ;                           // 积分项(油门量纲, 限幅抗饱和)
    double axLast;                          // 油门轴最近一次的值(防重复派发误判"手动")
    int    axValid;
    long   inSeq, outSeq;
    unsigned long long tOut, tIn;
    unsigned long long nNew, nBad;
} g_ah;

// 出生/任务重开: 状态清零 + 删掉两份通道文件
//   (不删的话, 上一个任务残留的 "hold=1" 会在新任务里把定速巡航又点亮)
static void j35AhReset(void)
{
    memset(&g_ah, 0, sizeof(g_ah));
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_AH_IN_FILE);  remove(p);
    snprintf(p, sizeof(p), "%s%s", d, J35_AH_OUT_FILE); remove(p);
}

// 当前真空速 TAS m/s(机体对气流速度 = 对地速度 - 风)
static double j35AhTAS(void)
{
    double ax = g_velBody.x - g_windBody.x;
    double ay = g_velBody.y - g_windBody.y;
    double az = g_velBody.z - g_windBody.z;
    return sqrt(ax*ax + ay*ay + az*az);
}

static void j35AhReadIn(void)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_AH_IN_FILE);
    FILE *fp = fopen(p, "r");
    if (!fp) return;                        // 文件还没出现(没进座舱/还没点过) -> 保持现状
    char line[128];
    line[0] = 0;
    if (!fgets(line, sizeof(line), fp)) { fclose(fp); return; }
    fclose(fp);

    char tag[16] = {0};
    long seq = 0; int hold = 0, lock = 0;
    int nf = sscanf(line, "%15s %ld %d %d", tag, &seq, &hold, &lock);
    if (nf < 4)
    { g_ah.nBad++; return; }
    if (strcmp(tag, "J35AH1") != 0)         { g_ah.nBad++; return; }
    if (seq <= 0 || seq == g_ah.inSeq) return;   // 旧行/同一行: 不动
    g_ah.inSeq   = seq;
    g_ah.tIn     = j35Ms();
    g_ah.holdReq = (hold > 0) ? 1 : 0;
    g_ah.lockReq = (lock > 0) ? 1 : 0;
    g_ah.nNew++;
}

static void j35AhWriteOut(void)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_AH_OUT_FILE);
    FILE *fp = fopen(p, "w");
    if (!fp) return;
    g_ah.outSeq++;
    fprintf(fp, "J35AHO1 %ld %d %d %.4f %.1f %.1f\n",
            g_ah.outSeq, g_ah.hold, g_ah.lock, g_throttleAxis,
            g_ah.hold ? g_ah.vt : 0.0, j35AhTAS());
    fclose(fp);
}

static void j35AhTick(void)
{
    unsigned long long now = j35Ms();
    if (now - g_ah.tOut < (unsigned long long)J35_AH_MS) return;
    g_ah.tOut = now;

    j35AhReadIn();

    if (g_ah.holdReq && !g_ah.lastHoldReq)          // 接合(只在 0->1 边沿): 锁定当前速度
    {
        g_ah.hold  = 1;
        g_ah.vt    = j35AhTAS();
        g_ah.thr0  = g_throttleAxis;
        g_ah.integ = 0.0;
        j35Dbg("AH 接合: 目标 %.1f m/s (%.0f km/h), 起始油门 %.2f, lock=%d",
               g_ah.vt, g_ah.vt*3.6, g_ah.thr0, g_ah.lock);
    }
    else if (!g_ah.holdReq && g_ah.hold)             // 座舱按掉了
    {
        g_ah.hold = 0;
        j35Dbg("AH 断开(座舱), 油门留在 %.2f", g_throttleAxis);
    }
    g_ah.lastHoldReq = g_ah.holdReq;
    g_ah.lock        = g_ah.lockReq;

    // 双发都熄火: 没推力可调 -> 自动断开(座舱从回写文件看到 hold=0 会弹回按钮)
    if (g_ah.hold && !g_engLit[0] && !g_engLit[1])
    {
        g_ah.hold = 0;
        j35Dbg("AH 双发熄火 -> 自动断开");
    }

    j35AhWriteOut();
}

// PI 控制律: 只在 hold 时调 g_throttleAxis
static void j35AhControl(double dt)
{
    if (!g_ah.hold) return;
    double V = j35AhTAS();
    if (V < 20.0) return;                   // 地面/极低速: 不参控, 油门保持
    double err = g_ah.vt - V;               // m/s, 正 = 偏慢要加油门
    double P   = 0.010 * err;               // 比例: 慢 10 m/s -> +0.10 油门
    g_ah.integ += 0.0022 * err * dt;        // 积分: 抵消爬升/阻力变化
    if (g_ah.integ >  0.30) g_ah.integ =  0.30;   // 限幅(抗饱和)
    if (g_ah.integ < -0.30) g_ah.integ = -0.30;
    double want = clamp(g_ah.thr0 + P + g_ah.integ, 0.0, 1.0);
    // 输出限速: 油门最快 0.25/秒, 防发动机滞后诱发绕目标速度的振荡
    double d  = want - g_throttleAxis;
    double mx = 0.25 * dt;
    if      (d >  mx) want = g_throttleAxis + mx;
    else if (d < -mx) want = g_throttleAxis - mx;
    g_throttleAxis = clamp(want, 0.0, 1.0);
}

// =============================================================================
//        全自动着舰 put_586 (2026-09-18 v4: F/A-18 式 ACLS, 三轴 + 全自动)
// -----------------------------------------------------------------------------
// 座舱侧: SYSTEM/cockpit_click.lua 点击 put_586(命令 3052 / arg 586)时写
//     <SavedGames>\J35DATA\J35_ACL_cmd.txt:
//   J35AC1 <seq> <on>
//     on: 0/1 期望状态(点一下取反)
//
// ★ v4 玩法(用户要求 2026-09-18): 飞到航母【尾后 5 km】对准斜角甲板, 点一下
//   put_586, 然后【双手离开键盘】, 剩下的全程自动:
//     1) 接合即: 放起落架 + 全放襟翼 + 请求座舱把【尾勾放下】(回写 hook=1);
//     2) 侧向: 用 J35_AL_link.txt 给的期望坡度做【坡度保持】—— 先把机头指向
//        "预测触地点"(带航母运动前置), 太远/侧偏大时先进攻"尾后 5km 中线点";
//     3) 垂直: 用链路给的期望航迹倾角(3.5° 下滑道 + 波束修正, 已按风修正成
//        "空中"航迹角)做【俯仰保持】θ = γ + α (PD + 俯仰角速度阻尼);
//     4) 油门: PI 保持链路给的进近速度(默认 76 m/s ≈ 274 km/h), 太快先平飞减速;
//     5) 上舰: DCS 报 CARRIER_HOOKED(挂上拦阻索) -> 立刻收光油门 + 满刹车,
//        停稳(V<2)后交还玩家(断开时【不】抬油门 —— 在甲板上不能加速);
//     6) 逃逸: 过了触地点(x < -25 m)还没挂上索 -> 满油门 +6° 爬升, 4 秒后交还。
//
// 几何从哪来(EFM 拿不到本机/航母世界坐标):
//     Export 环境 J35_AL_Export.lua 每帧算好写 <SavedGames>\J35DATA\J35_AL_link.txt:
//     J35AL1 <seq> <bankDeg> <gammaDeg> <vt> <hAgl> <x> <dCross> <trk> <hdg>
//            <gs> <cvY> <cvHdg> <brc> <valid>
//   本 DLL 每 25ms 读一次(只取 seq 变新的那行, 超过 600ms 没更新算过期)。
//   valid=0 或没有链路 -> 自动退回 v3 老模式(只保俯仰 -3.5°, 不碰滚转)。
//
//   自动断开(任一): 玩家动杆(俯仰/滚转键或轴真变了) / 手动油门 / 定速巡航接合 /
//                   双发熄火 / 接地(V<40) / 上舰停稳 / 逃逸交还 / 座舱按掉。
//   断开时若油门在低位一律抬到 0.45 保命(上舰/接地断开例外)。
//   回写 <SavedGames>\J35DATA\J35_ACL_out.txt (40Hz):
//   J35ACO1 <seq> <on> <vt> <v> <hook> <mode> <x> <dCross> <hAgl>
//     on   = 本 DLL 实际状态(座舱按它弹回按钮);
//     hook = 1 -> 请座舱把尾勾放下(座舱 acl_update 会 click_switch(HOOK_SW,1));
//     mode = 0 进近 / 1 上舰减速 / 2 逃逸复飞
// =============================================================================

#define J35_ACL_IN_FILE  "J35_ACL_cmd.txt"   // 座舱 Lua 写, 本 DLL 读
#define J35_ACL_OUT_FILE "J35_ACL_out.txt"   // 本 DLL 写, 座舱读
#define J35_AL_IN_FILE   "J35_AL_link.txt"   // Export(制导) 写, 本 DLL 读
#define J35_ACL_MS       25                  // 通道读+回写限频(40Hz)
#define J35_AL_MAXAGE    600                 // 制导链路超过 600ms 没更新 = 过期
#define J35_ACL_GAMMA    (-0.0611)           // 老模式目标下滑角 rad(-3.5°)
#define J35_R2D          57.29578            // rad -> deg

enum { J35_ACL_AIR = 0, J35_ACL_DECK = 1, J35_ACL_BOLTER = 2 };

// 制导链路(Export 侧 J35_AL_Export.lua 算好的一行)
static struct J35AclLink
{
    long   inSeq;
    unsigned long long tIn;           // 上次看到新 seq 的时刻
    double bankDeg, gammaDeg, vt, hAgl, x, dCross, trk, hdg, gs, cvY, cvHdg, brc;
    int    valid;
    unsigned nNew, nBad;
} g_al;

static struct J35AclState
{
    int    on, onReq, lastOnReq;      // 实际 / 座舱要的 / 上一次要的(只在 0->1 边沿接合)
    int    mode;                      // J35_ACL_AIR / DECK / BOLTER
    int    linkOk;                    // 本次 tick 制导链路是否可用(几何 + 新鲜)
    double vt;                        // 目标真空速 m/s(有链路时用链路给的)
    double integ;                     // 油门 PI 积分(限幅抗饱和)
    double alphaF;                    // 低通攻角 rad(目标姿态用)
    double altPrev;                   // 上一帧真高(差分算实测下沉率/航迹角)
    double gamAct;                    // 低通后的"实测航迹角"(deg)
    double gamI;                      // 航迹角外环积分(deg): 补掉俯仰 PD 环的稳态低头偏差
    double axLast;                    // 油门轴最近值(防同值重发误判"手动")
    int    axValid;
    double paxLast, raxLast;          // 俯仰/滚转轴最近值(同上)
    int    paxValid, raxValid;
    int    guard;                     // 平飞保护状态: 0=正常下滑 1=太快 2=太慢 3=低高度
    int    hookReq;                   // 1 = 请求座舱把尾勾放下(v4 全自动)
    int    trapped;                   // DCS 报"挂上拦阻索"(v4 -> 上舰模式)
    unsigned long long tTrap;         // 挂上拦阻索的时刻
    unsigned long long tBolter;       // 逃逸(复飞)开始时刻
    unsigned long long tLog;          // 链路几何日志限频
    long   inSeq, outSeq;
    unsigned long long tOut;
} g_acl;

// 出生/任务重开: 状态清零 + 删掉三份通道文件(防上一个任务残留 on=1)
static void j35AclReset(void)
{
    memset(&g_acl, 0, sizeof(g_acl));
    memset(&g_al,  0, sizeof(g_al));
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_ACL_IN_FILE);  remove(p);
    snprintf(p, sizeof(p), "%s%s", d, J35_ACL_OUT_FILE); remove(p);
    snprintf(p, sizeof(p), "%s%s", d, J35_AL_IN_FILE);   remove(p);  // Export 侧 50ms 内会重写
}

static void j35AclReadIn(void)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_ACL_IN_FILE);
    FILE *fp = fopen(p, "r");
    if (!fp) return;                        // 文件还没出现(没进座舱/还没点过) -> 保持现状
    char line[128];
    line[0] = 0;
    if (!fgets(line, sizeof(line), fp)) { fclose(fp); return; }
    fclose(fp);

    char tag[16] = {0};
    long seq = 0; int on = 0;
    int nf = sscanf(line, "%15s %ld %d", tag, &seq, &on);
    if (nf < 3)                          return;
    if (strcmp(tag, "J35AC1") != 0)      return;
    if (seq <= 0 || seq == g_acl.inSeq) return;   // 旧行/同一行: 不动
    g_acl.inSeq = seq;
    g_acl.onReq = (on > 0) ? 1 : 0;
}

// ★ v4: 读 Export 侧制导链路(只认 seq 变新的那行; 文件不存在就保持 g_al 为空)
static void j35AclLinkRead(void)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_AL_IN_FILE);
    FILE *fp = fopen(p, "r");
    if (!fp) return;                        // Export 侧脚本没装 -> valid 永远 0
    char line[256];
    line[0] = 0;
    if (!fgets(line, sizeof(line), fp)) { fclose(fp); return; }
    fclose(fp);

    char tag[16] = {0};
    long seq = 0;
    double bank = 0, gam = 0, vt = 0, hAgl = 0, x = 0, cross = 0, trk = 0, hdg = 0,
           gs = 0, cvY = 0, cvHdg = 0, brc = 0;
    int    valid = 0;
    int nf = sscanf(line, "%15s %ld %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d",
                    tag, &seq, &bank, &gam, &vt, &hAgl, &x, &cross, &trk, &hdg,
                    &gs, &cvY, &cvHdg, &brc, &valid);
    if (nf < 15 || strcmp(tag, "J35AL1") != 0) { g_al.nBad++; return; }
    if (seq == g_al.inSeq) return;          // 同一行: 不动
    g_al.inSeq    = seq;
    g_al.tIn      = j35Ms();
    g_al.bankDeg  = bank;   g_al.gammaDeg = gam;   g_al.vt     = vt;
    g_al.hAgl     = hAgl;   g_al.x        = x;     g_al.dCross = cross;
    g_al.trk      = trk;    g_al.hdg      = hdg;   g_al.gs     = gs;
    g_al.cvY      = cvY;    g_al.cvHdg    = cvHdg; g_al.brc    = brc;
    g_al.valid    = (valid > 0) ? 1 : 0;
    g_al.nNew++;
}

static void j35AclWriteOut(void)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_ACL_OUT_FILE);
    FILE *fp = fopen(p, "w");
    if (!fp) return;
    g_acl.outSeq++;
    // 座舱侧只解析前 4 个数字(seq/on/vt/v); 后面是状态: hook 请求 + 模式 + 几何
    fprintf(fp, "J35ACO1 %ld %d %.4f %.1f %d %d %.1f %.1f %.1f\n",
            g_acl.outSeq, g_acl.on, g_acl.on ? g_acl.vt : 0.0, j35AhTAS(),
            g_acl.hookReq, g_acl.mode,
            g_acl.on ? g_al.x : 0.0, g_acl.on ? g_al.dCross : 0.0,
            g_acl.on ? g_al.hAgl : 0.0);
    fclose(fp);
}

// 断开(带原因; 座舱从回写看到 on=0 会把按钮弹回 OFF)
// ★ 2026-09-18 坠海教训: 因"玩家动杆/座舱按掉/定速巡航接合"断开时, 油门若在低位
//   一律抬到 0.45 保命 —— 否则键盘玩家一断开就是无动力失速(122 m/s 拉杆掉到 46 m/s 坠海)。
//   手动油门断开(玩家自己在操作油门)和已接地/上舰断开(在甲板上, 不能加速)不抬。
// ★ v4: keepThr=1 时原样保留油门与刹车(上舰停稳用)。
static void j35AclDisengage(const char *why, int keepThr = 0)
{
    if (!g_acl.on) return;
    g_acl.on       = 0;
    g_acl.hookReq  = 0;                    // 不再请求放尾勾(已放下的不收回)
    g_reqPitch = 0.0;                      // 杆还给玩家
    g_reqRoll  = 0.0;
    if (keepThr)
    {
        j35Dbg("ACL 断开: %s (保留油门/刹车, 甲板上不加速)", why);
        return;
    }
    if (strncmp(why, "玩家手动", 12) != 0 && strncmp(why, "已接地", 9) != 0
        && strncmp(why, "已上舰", 9) != 0 && g_throttleAxis < 0.30)
    {
        g_throttleAxis = 0.45;
        j35Dbg("ACL 断开: %s (油门过低, 已抬到 0.45 保命)", why);
        return;
    }
    j35Dbg("ACL 断开: %s", why);
}

static void j35AclTick(void)
{
    unsigned long long now = j35Ms();
    if (now - g_acl.tOut < (unsigned long long)J35_ACL_MS) return;
    g_acl.tOut = now;

    j35AclReadIn();
    j35AclLinkRead();

    // 制导链路是否可用(几何有效 + 600ms 内有更新)
    g_acl.linkOk = (g_al.valid && g_al.inSeq > 0 &&
                    (now - g_al.tIn) < (unsigned long long)J35_AL_MAXAGE) ? 1 : 0;

    if (g_acl.onReq && !g_acl.lastOnReq)          // 接合(只在 0->1 边沿)
    {
        double V = j35AhTAS();
        if (V < 50.0 || g_alt < 5.0)
            j35Dbg("ACL 接合被拒: 未离地(V=%.1f m/s, alt=%.0f m)", V, g_alt);
        else if (!g_engLit[0] && !g_engLit[1])
            j35Dbg("ACL 接合被拒: 双发都没点火");
        else
        {
            g_acl.on       = 1;
            g_acl.mode     = J35_ACL_AIR;
            g_acl.trapped  = 0;
            g_acl.hookReq  = 1;                    // ★ v4: 请求座舱放尾勾
            g_acl.vt       = g_acl.linkOk ? clamp(g_al.vt, 70.0, 85.0)
                                          : clamp(V, 72.0, 80.0);
            g_acl.integ    = 0.0;
            g_acl.alphaF   = g_dbgAoa;
            g_acl.altPrev  = g_alt;                 // 复位实测航迹角外环
            g_acl.gamAct   = 0.0;
            g_acl.gamI     = 0.0;
            g_gearTarget   = 1.0;                  // 着陆外型: 放起落架
            g_flapTarget   = 1.0;                  //            全放襟翼
            g_flapExtSync  = 1;                    // 回写座舱 put_580 拨钮
            if (g_ah.hold) { g_ah.hold = 0; j35Dbg("ACL: 定速巡航让位, 已断开"); }
            if (g_acl.linkOk)
                j35Dbg("ACL 接合(全自动三轴): 尾后 %.0f m, 侧偏 %+.1f m, 甲板顶 %.0f m, "
                       "舰艏向 %.1f / 着舰航向 %.1f, 目标 %.0f m/s (%.0f km/h); "
                       "起落架+襟翼+尾勾; 下滑道 -3.5°",
                       g_al.x, g_al.dCross, g_al.cvY, g_al.cvHdg, g_al.brc,
                       g_acl.vt, g_acl.vt*3.6);
            else
                j35Dbg("ACL 接合(★ 只保俯仰的旧模式): 目标 %.0f m/s (%.0f km/h), 起落架+襟翼",
                       g_acl.vt, g_acl.vt*3.6);
            if (!g_acl.linkOk)
                j35Dbg("ACL: ★ 没有可用的 J35_AL_link.txt -> 无法自动着舰. 检查 "
                       "<SavedGames>\\Scripts\\Export.lua 有没有加载 J35_AL_Export.lua");
        }
    }
    else if (!g_acl.onReq && g_acl.on)             // 座舱按掉了
    {
        j35AclDisengage("座舱按掉");
    }
    g_acl.lastOnReq = g_acl.onReq;

    // ---- 自动断开 / 模式推进 ----
    if (g_acl.on)
    {
        double V = j35AhTAS();
        if (g_acl.trapped && g_acl.mode == J35_ACL_AIR)      // ★ 挂上拦阻索 -> 上舰
        {
            g_acl.mode  = J35_ACL_DECK;
            g_acl.tTrap = now;
            j35Dbg("ACL ★ 已挂上拦阻索 -> 收光油门 + 满刹车, 停稳后交还玩家");
        }

        if (g_acl.mode == J35_ACL_DECK)
        {
            if (V < 2.0 || now - g_acl.tTrap > 6000)
                j35AclDisengage("已上舰停稳", 1);
        }
        else if (g_acl.mode == J35_ACL_BOLTER)
        {
            if (now - g_acl.tBolter > 4000) j35AclDisengage("逃逸复飞(没挂上)交还玩家");
        }
        else if (!g_engLit[0] && !g_engLit[1])  j35AclDisengage("双发熄火");
        else if (g_ah.hold)                     j35AclDisengage("定速巡航接合");
        else if (V < 40.0)                      j35AclDisengage("已接地(速度过低)");
        else if (V < 60.0 && g_velBody.y > -0.5) j35AclDisengage("已接地(不再下降)");
        // ★ 2026-09-18 第二次试飞实锤: 原来"x<-25 就复飞"导致飞机 66 m 高空飞过舰首
        //   时也会触发假复飞(本次 hAgl~66, 比甲板高 47m, 不可能挂上拦阻索)。
        //   改成只有【确实贴近甲板】(hAgl < 30m = 甲板高+11m 安全余量)还没挂上才复飞;
        //   飞得太高的"飞过舰首"留给玩家手动断开, 不再强加满油门拉起(实锤: 拉起后
        //   飞机会从 66m 飙到 116m, 完全偏离玩家预期)。
        // ★ 2026-09-18 第三次试飞实锤反转: BOLTER 应该是"飞到舰首位置还高(没机会挂索)"
        //   不是"贴海面还没挂上"。本次: x=-331m hAgl=74.9m(= 甲板上方 56m)早该判
        //   逃逸, 实际等到 x=-2114m hAgl=29.5m 才触发, 晚了整整 2 km, 飞机早就飞远了。
        //   改成: 接近或刚过舰首(x∈[-300, +100]) && hAgl 还在甲板上 6 m 以上 -> 逃逸。
        else if (g_acl.linkOk && g_al.x < 100.0 && g_al.x > -300.0 && g_al.hAgl > 25.0)
        {
            g_acl.mode    = J35_ACL_BOLTER;
            g_acl.tBolter = now;
            j35Dbg("ACL ★ 逃逸: 已贴近舰首(x=%.0f m)离甲板还有 %.0f m -> 满油门 +6° 拉起",
                   g_al.x, g_al.hAgl);
        }
    }

    // ---- 几何日志(1Hz): 校准甲板高度 / 下滑道 / 航向道用 ----
    if (g_acl.on && now - g_acl.tLog >= 1000)
    {
        g_acl.tLog = now;
        if (g_acl.linkOk)
            j35Dbg("ACL* mode=%d x=%+.0fm cross=%+.1fm hAgl=%.1fm(甲板顶%.1f 本机%.0f) "
                   "V=%.1f gs=%.1f trk=%.1f hdg=%.1f BRC=%.1f(cvHdg=%.1f) "
                   "bank=%+.1f gam=%+.1f surfObj=%.1f",
                   g_acl.mode, g_al.x, g_al.dCross, g_al.hAgl, g_al.cvY, g_alt,
                   j35AhTAS(), g_al.gs, g_al.trk, g_al.hdg, g_al.brc, g_al.cvHdg,
                   g_al.bankDeg, g_al.gammaDeg, g_surfHObj);
        else
            j35Dbg("ACL* mode=%d V=%.1f (无制导链路: 只保俯仰)", g_acl.mode, j35AhTAS());
    }

    j35AclWriteOut();
}

// 每物理步(50Hz): 滚转坡度保持 + 俯仰 PD + 油门 PI(只在 on 时动杆/油门)
// ★ v4 三个通道:
//     · 滚转: 只有拿到制导链路才动 —— 用链路的 bankCmd 做【坡度保持】
//             杆量 = 2.5*(期望坡度 - 实际坡度) - 4.0*滚转角速度, 限 ±0.6
//     · 俯仰: 目标航迹角 γ(链路给; 没链路就用 v3 的 -3.5° 渐变) + 低通攻角
//             杆量 = 3.0*(θ目标 - θ) - 6.0*俯仰角速度, 限 ±0.5
//     · 油门: PI 保进近速度。P 限 ±0.20, 积分 ±0.15, 基准 0.50; 太快段收光油门平飞减速
// ★ 平飞保护(v2/v3 的坠海教训全部保留): 太快平飞减速 / 太慢平飞防失速 / 离海面
//   <8m 平飞防触海。上舰与逃逸模式直接接管(见下)。
static void j35AclControl(double dt)
{
    if (!g_acl.on) return;

    // ---- 上舰模式: 收光油门 + 满刹车 + 不碰杆(拦阻索已经把飞机拉住了) ----
    if (g_acl.mode == J35_ACL_DECK)
    {
        g_throttleAxis = 0.0;
        g_wheelBrake   = 1.0;
        g_reqPitch     = 0.0;
        g_reqRoll      = 0.0;
        g_acl.integ    = -0.15;
        return;
    }

    double V = j35AhTAS();
    if (V < 30.0) return;                   // 地面/极低速: 不参控

    // ---- 逃逸模式: 满油门 + 6° 爬升 + 机翼扶平(跑出甲板别回头扎海) ----
    if (g_acl.mode == J35_ACL_BOLTER)
    {
        g_throttleAxis = 1.0;
        g_acl.alphaF += (g_dbgAoa - g_acl.alphaF) * clamp(dt/0.7, 0.0, 1.0);
        double thB = 6.0/J35_R2D + g_acl.alphaF;
        double elB = 3.0*(thB - g_los.pit) - 6.0*g_omega[2];
        g_reqPitch = clamp(elB, -0.3, 0.5);
        g_reqRoll  = clamp(-2.5*g_los.rol - 4.0*g_omega[0], -0.4, 0.4);
        return;
    }

    // ---- 目标速度: 有链路就用链路给的口径, 没链路用接合时的 ----
    double vt = g_acl.vt;
    if (g_acl.linkOk && g_al.vt > 40.0 && g_al.vt < 140.0) vt = g_al.vt;

    // ---- 平飞保护状态机(变化时记日志) ----
    // ★ 2026-09-18 第二次试飞实锤修正 guard=3:
    //   原来判据是"离海面 8 m"(g_alt-g_surfH) —— 可航母甲板本身就站在海面上 19 m,
    //   飞机要落甲板必然先穿过 0~19 m, 所以旧条件等于"等飞机扎进海里才保护"
    //   (日志实锤: 最后一次保护消息就是"离海面仅 8.0 m", 之后再无输出)。
    //   现在有制导链路时改成按【离甲板高度】+ 一条 1° 安全剖面判, 而且"太低"
    //   优先级高于"太快"(原来太快优先, 掉到海面附近还在平飞减速)。
    int guard = 0;
    int tooLow;
    // ★ 2026-09-18 第四次试飞坠海教训: 之前 "x<800 && hAgl<15" 在飞机过舰首 100m
    //   hAgl=9.1m 才触发, 已经砸穿甲板。新版让 PD(0.10)+死区(5m)让飞机温柔下沉,
    //   "太低"保护必须【提前】触发 —— 在飞机还没到舰首、但已经贴近甲板顶
    //   (甲板 19m + 安全 11m = 30m)时就紧急救场, 不要等飞机飞过去才发现。
    //   范围 x ∈ (-100, 200): 进近最后 200m 到刚过舰首 100m, 之前留给 PD 自己爬回下滑道。
    if (g_acl.linkOk) tooLow = (g_al.x > -100.0 && g_al.x < 200.0 && g_al.hAgl < 30.0);
    else              tooLow = (g_alt - g_surfH < 8.0);
    if      (tooLow)                 guard = 3;   // 掉到安全剖面以下 -> 复飞
    else if (V > vt + 15.0)          guard = 1;   // 太快 -> 平飞减速
    else if (V < vt - 10.0)          guard = 2;   // 太慢 -> 平飞防失速
    if (guard != g_acl.guard)
    {
        g_acl.guard = guard;
        if      (guard == 1) j35Dbg("ACL 保护: 太快(V=%.1f > 目标+15), 平飞收光油门减速", V);
        else if (guard == 2) j35Dbg("ACL 保护: 太慢(V=%.1f < 目标-10), 平飞防失速", V);
        else if (guard == 3) j35Dbg("ACL ★ 复飞保护: 离甲板仅 %.1f m(x=%.0f m) -> 满油门拉起",
                                    g_al.hAgl, g_al.x);
        else                 j35Dbg("ACL 保护解除: 恢复下滑 (V=%.1f)", V);
    }

    // ---- 油门通道 ----
    // ★ 太快段【收光油门】平飞减速 —— v2 的 PI 下限 0.15 会让平飞卡在 117 m/s
    //   永远减不进窗口(日志实锤)。出窗口后 PI 恢复接管。
    double want;
    if (guard == 1)
    {
        want = 0.0;                            // 平飞减速段: 怠速
        g_acl.integ = -0.15;                   // 出保护段时 PI 从低位接手
    }
    else if (guard == 3)
    {
        want = 1.0;                            // ★ 复飞保护: 满油门
        g_acl.integ = 0.0;
    }
    else
    {
        double err = vt - V;                   // m/s, 正 = 偏慢要加油门
        double P   = clamp(0.010 * err, -0.20, +0.20);
        g_acl.integ += 0.0022 * err * dt;
        if (g_acl.integ >  0.15) g_acl.integ =  0.15;
        if (g_acl.integ < -0.15) g_acl.integ = -0.15;
        want = clamp(0.50 + P + g_acl.integ, 0.0, 1.0);
    }
    double d  = want - g_throttleAxis;
    double mx = 0.25 * dt;                     // 输出限速 0.25/秒
    if      (d >  mx) want = g_throttleAxis + mx;
    else if (d < -mx) want = g_throttleAxis - mx;
    g_throttleAxis = clamp(want, 0.0, 1.0);

    // ---- 俯仰通道: 目标姿态 = 航迹角 + 低通攻角; PD + 角速度阻尼 ----
    double gamCmd;
    if (g_acl.linkOk)
        gamCmd = clamp(g_al.gammaDeg/J35_R2D, -12.0/J35_R2D, +7.0/J35_R2D);
    else
    {
        double g01 = clamp((vt + 12.0 - V) / 12.0, 0.0, 1.0);
        gamCmd = J35_ACL_GAMMA * g01;          // v3 老模式: 太快时 0(平飞), 达标 -3.5°
    }
    // ★ 2026-09-18 第六次试飞实锤(V 保护反弹 + K_PD 反向 + 几何 bug): 之前 guard=1 时
    //   强制 gamCmd=-1°(平飞), attempt#5 实锤 PD 控制失效。改后又加"x<1000m 强制平飞",
    //   attempt#6 实锤最后 1km V>91 时又把 PD 下沉命令吃掉 (飞机在 x=326m gam=-3.5°
    //   dh=4.65m 应该继续下沉到 dh=0, 但被强制平飞导致飞机 BOLTER 时仍 dh=+8m)。
    //   现在: 完全让 PD 控制自由执行 (throttle 通道收油门减速, 这里不重复),
    //         仅在【极度超速 + 接近舰首】时才强制平飞防擦舰超速。
    if (guard == 1 && g_al.x < 200.0 && V > vt + 25.0) gamCmd = -1.0/J35_R2D;
    if (guard == 2) gamCmd = 0.0;                              // 太慢: 平飞防失速
    if (guard == 3) gamCmd = +4.0/J35_R2D;                     // ★ 太低: 真复飞(拉起来 4°)
    g_acl.alphaF += (g_dbgAoa - g_acl.alphaF) * clamp(dt/0.7, 0.0, 1.0);   // 一阶低通(0.7s)

    // ---- ★ 航迹角外环积分(2026-09-18 第七次试飞实锤缩小) ----
    //  俯仰通道是纯 PD(3.0*(θcmd-θ) - 6.0*qz), 没有积分项: 在"怠速+起落架+襟翼"这种
    //  需要很大抬头配平量的状态, 稳态会留 1~3° 低头偏差。第二次试飞日志实锤:
    //    链路命令 γ=-5.5°, 实际 pit=-2.6° / aoa=+6.1° -> 真实 γ=-8.7°
    //  也就是飞机永远比指令陡约 3° -> 越落越低 -> 最后扎海。
    //  这里用【实测航迹角】做外环积分, 积分量直接补到 θ 目标上:
    //    实测 γ = asin( d(真高)/dt / V )  再低通(0.6s), 直到实测 = 指令才停。
    //  ★ 第七次试飞 attempt#7 实锤: gamI clamp ±8° 太大, 在飞机"自动配平 tr=+5.3°
    //    持续抬头"状态下, gamAct 偏离 gamCmd 累积 gamI 到 +8°, theta_cmd=+5° 拉起,
    //    但内环 PD 跟不上外环命令, 飞机 gam_act 仍 -7.5° (俯冲), 飞机 hAgl 反向飙升
    //    17m, BOLTER 时仍高 58m。缩 gamI clamp 到 ±2°, 让外环积分只补偿小偏差,
    //    大偏差由 Lua 端 K_VS=0.20 直接纠正 (DH 收敛 τ=2.7s)。
    {
        double vsNow = 0.0;
        if (dt > 1e-5) vsNow = (g_alt - g_acl.altPrev) / dt;
        g_acl.altPrev = g_alt;
        double gamActDeg = (V > 30.0) ? (asin(clamp(vsNow / V, -0.6, 0.6)) * J35_R2D) : 0.0;
        g_acl.gamAct += (gamActDeg - g_acl.gamAct) * clamp(dt/0.6, 0.0, 1.0);
        if (g_acl.mode == J35_ACL_AIR)     // ★ 接合后进近模式就积分(不再卡 LOS 新鲜度;
                                    //   单兵模式 linkOk 大段时间为 false, 上版本积分项从！来！没！跑！)
            g_acl.gamI = clamp(g_acl.gamI + 0.4*(gamCmd*J35_R2D - g_acl.gamAct)*dt,
                               -2.0, +2.0);                  // ★ 缩 gamI clamp 到 ±2°(从 ±8°)
        else
            g_acl.gamI = 0.0;
    }
    double thetaCmd = gamCmd + g_acl.gamI/J35_R2D + g_acl.alphaF;
    double elev = 3.0*(thetaCmd - g_los.pit) - 6.0*g_omega[2];
    g_reqPitch = clamp(elev, -0.5, 0.5);     // 不打满杆, 留安全余量

    // ---- 滚转通道: 坡度保持(只有制导链路才动滚转; 没链路完全不碰 = v3 行为) ----
    if (g_acl.linkOk)
    {
        double bankCmd = clamp(g_al.bankDeg/J35_R2D, -30.0/J35_R2D, +30.0/J35_R2D);
        if (g_alt - g_surfH < 25.0) bankCmd *= 0.5;   // 贴近海面/甲板: 机翼再扶平一点
        double rs = 2.5*(bankCmd - g_los.rol) - 4.0*g_omega[0];
        g_reqRoll = clamp(rs, -0.6, 0.6);
    }
    else
    {
        g_reqRoll = 0.0;
    }
}

// ===================== 系统开关 / 起动联锁 (put_591~601, 2026-09-17 新增) =====================
// 座舱侧 SYSTEM/cockpit_click.lua 轮询 arg591~601 的实际位置, 变化就写
//   <SavedGames>\J35DATA\J35_SW_cmd.txt:
//   J35SW1 <seq> <pw> <hyd> <fuel> <brk> <gen> <bat> <engL> <engR>
//     pw/hyd/fuel/brk/gen/bat = 总电源(593)/液压阀(597)/燃油阀(598)/刹车阀(599)/
//                               发电机(600)/蓄电池(601), 1 = 开
//     engL/engR = 左(591)/右(592)发开关位置: 0 = 停车, 0.5 = 起动, 1 = 灭火
//
// 联锁规则(用户口径 2026-09-17):
//   * 六个系统开关【全开】才允许起动 —— 发动机开关拨到起动位和键盘 309/311/312
//     起动命令同样受限, 少一个都起不来("否则飞机无法启动");
//   * 燃油阀 从开拨到关 -> 双发熄火(只在确认过"开"之后才认这次边沿,
//     热启动/空中出生没动过开关的发动机不受牵连);
//   * 刹车阀关(且开关文件已生效) -> 机轮刹车无效;
//   * 灭火位: 立即断油断火(N1 加倍速衰减), 拨回停车位之前禁止再起动。
// ==============================================================================================
#define J35_SW_IN_FILE  "J35_SW_cmd.txt"   // 座舱 Lua 写, 本 DLL 读
#define J35_SW_MS       50                 // 读限频(20Hz)

static struct J35SwState
{
    long   inSeq;                  // 座舱行号(旧行/同一行不动)
    int    pw, hyd, fuel, brk, gen, bat;   // 1 = 开
    int    fuelPrev;               // 上一次读到的燃油阀(-1 = 还没读到过, 用于 1->0 边沿)
    double engReq[2];              // 发动机开关位置 0 / 0.5 / 1
    double engPrev[2];             // 上一次读到的发动机开关位置(-1 = 还没读到过, 用于 边沿 判定)
    int    fired[2];               // 灭火动作已做(拨回停车位 = 解除)
    unsigned long long tOut;
} g_sw;

// 六个系统开关是否全开(起动联锁)
static int j35SysReady(void)
{
    return g_sw.pw && g_sw.hyd && g_sw.fuel && g_sw.brk && g_sw.gen && g_sw.bat;
}

// 生效刹车: 刹车阀(put_599)关 -> 0; 开关文件还没出现过(热启动没点过开关) -> 保持原值
static double j35BrakeEff(void)
{
    if (g_sw.inSeq > 0 && !g_sw.brk) return 0.0;
    return g_wheelBrake;
}

// 应用发动机开关请求(每次读到新行时)
static void j35SwApply(int i, double r)
{
    double prev = g_sw.engPrev[i];          // 上一次的开关位置(-1 = 还没读到过)
    g_sw.engPrev[i] = r;

    if (r > 0.75)                          // 1 = 灭火
    {
        if (!g_sw.fired[i])
        {
            g_sw.fired[i] = 1;
            if (g_engState[i] != ENG_OFF && g_engState[i] != ENG_SHUTDOWN)
            {
                engStopRequest(i);
                g_engLit[i]       = false;        // 立即断油断火
                g_engShutFrom[i] *= 0.5;          // 惰转起点压半 -> 灭火比正常停车快一倍
            }
            j35Dbg("ENG%d FIRE(灭火): 停车+断火; 拨回停车位前禁止再起动", i+1);
        }
    }
    else if (r > 0.25)                     // 0.5 = 起动
    {
        if (g_sw.fired[i])
            j35Dbg("ENG%d START blocked: 灭火后未复位(先拨回停车位)", i+1);
        else if (!j35SysReady())
            j35Dbg("ENG%d START blocked: 系统联锁未通(总电源/液压/燃油/刹车/发电机/蓄电池)", i+1);
        else
            engStartRequest(i);
    }
    else                                   // 0 = 停车
    {
        g_sw.fired[i] = 0;                 // 拨回停车位 = 解除灭火锁定
        // ★ 2026-09-17 修复("雷达光标不能动"根因): 停车必须【边沿触发】——
        //   只有"真的从起动位(0.5)/灭火位(1) 拨回停车位(0)"才停下发动机。
        //   旧写法只要读到 0 就停车, 而出生时座舱写的第一份 J35_SW_cmd.txt 里
        //   engL/engR 本来就是 0(模型上的 591/592 默认停在停车位, 当出生状态记账),
        //   于是热启动 / 空中出生一进座舱, 双发立刻被当"停车请求"掐掉:
        //     J35_EFM_dbg.log 实证: "ENG1 STOP: SHUTDOWN from n1=0.750" 紧跟
        //     "SW: seq=2 ... engL=0.00 engR=0.00"。
        //   双发一停 -> avSimpleElectricSystem 没电 -> 雷达关机(RADAR_MODE=0)
        //   -> 雷达页只画方框扫描线, 光标根本画不出来(表现为"光标不能动")。
        if (prev > 0.25 &&
            g_engState[i] != ENG_OFF && g_engState[i] != ENG_SHUTDOWN)
            engStopRequest(i);
        else if (prev < 0.0)
            j35Dbg("ENG%d 出生基线: 开关停在停车位(0), 只记账不停车 —— 保护热启动/空中出生的双发", i+1);
        else if (prev > 0.0 && g_engState[i] == ENG_OFF)
            j35Dbg("ENG%d STOP 忽略: 发动机已经是停车状态", i+1);
    }
}

// 出生/任务重开: 状态清零 + 删通道文件(与 j35AhReset 同款)
static void j35SwReset(void)
{
    memset(&g_sw, 0, sizeof(g_sw));
    // 发动机开关位置"还没读到过"(-1): 出生后第一份文件只当基线记账,
    // 不产生停车动作(否则热启动/空中出生会被自己的出生状态掐掉双发)。
    g_sw.engPrev[0] = g_sw.engPrev[1] = -1.0;
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_SW_IN_FILE); remove(p);
}

static void j35SwReadIn(void)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_SW_IN_FILE);
    FILE *fp = fopen(p, "r");
    if (!fp) return;                        // 文件还没出现(没进座舱/还没点过) -> 保持现状
    char line[160];
    line[0] = 0;
    if (fgets(line, sizeof(line), fp))
    {
        char tag[16] = {0};
        long seq = 0; int pw=0, hyd=0, fuel=0, brk=0, gen=0, bat=0; double el=0.0, er=0.0;
        int nf = sscanf(line, "%15s %ld %d %d %d %d %d %d %lf %lf",
                        tag, &seq, &pw, &hyd, &fuel, &brk, &gen, &bat, &el, &er);
        if (nf == 10 && strcmp(tag, "J35SW1") == 0 && seq > 0 && seq != g_sw.inSeq)
        {
            g_sw.inSeq = seq;
            g_sw.pw   = (pw   > 0);
            g_sw.hyd  = (hyd  > 0);
            g_sw.fuel = (fuel > 0);
            g_sw.brk  = (brk  > 0);
            g_sw.gen  = (gen  > 0);
            g_sw.bat  = (bat  > 0);
            g_sw.engReq[0] = clamp(el, 0.0, 1.0);
            g_sw.engReq[1] = clamp(er, 0.0, 1.0);
            // 燃油阀 开->关 边沿: 双发熄火(只在确认过"开"之后才认, 热启动不受牵连)
            if (g_sw.fuelPrev == 1 && !g_sw.fuel)
            {
                engStopRequest(0); engStopRequest(1);
                j35Dbg("燃油阀关闭: 双发熄火");
            }
            g_sw.fuelPrev = g_sw.fuel;
            j35SwApply(0, g_sw.engReq[0]);
            j35SwApply(1, g_sw.engReq[1]);
            j35Dbg("SW: seq=%ld pw=%d hyd=%d fuel=%d brk=%d gen=%d bat=%d engL=%.2f engR=%.2f ready=%d",
                   seq, g_sw.pw, g_sw.hyd, g_sw.fuel, g_sw.brk, g_sw.gen, g_sw.bat,
                   g_sw.engReq[0], g_sw.engReq[1], j35SysReady());
        }
    }
    fclose(fp);
}

static void j35SwTick(void)
{
    unsigned long long now = j35Ms();
    if (now - g_sw.tOut < (unsigned long long)J35_SW_MS) return;
    g_sw.tOut = now;
    j35SwReadIn();
}

// ================ 座舱内座舱盖动画 arg181 (2026-09-17 新增) ================
// 座舱里那条座舱盖动画 = 座舱模型的 arg 181(用户口径 2026-09-17)。
//
// 为什么必须由本 DLL 写: 座舱 Lua 【写不了】座舱参数 —— dcs.log 实证
//   set_cockpit_draw_argument_value = nil(只有 get_cockpit_draw_argument_value 能读),
//   而唯一能写座舱参数的通道就是飞行模型 DLL:
//   wHumanCustomPhysicsAPI.h 里的 ed_fm_set_fc3_cockpit_draw_args_v2
//   (注释原文: "direct control over cockpit arguments" —— FC3 式座舱 + 自制 EFM 时,
//    座舱参数由本 DLL 每帧填)。
//
// 通道: 座舱侧 SYSTEM/cockpit_click.lua 把座舱盖当前位置写
//   <SavedGames>\J35DATA\J35_CP_cmd.txt :  "J35CP1 <seq> <v>"
//     v = 0.000 = 座舱盖关, 0.900 = 全开(取外部 arg38 的读回值, 与模型同一条时间线)
//   本 DLL 20Hz 读走, 回调里写 array[181]。
// ★ 只写 181 这一个下标 —— 其余座舱参数(各开关由引擎按点击元素自己驱动)一律不碰。
// ★ 座舱 arg181 与外部 arg38 量程不一致时, 改 J35_CP_SCALE(如座舱要 0~1.0 就填 1.1111)。
#define J35_CP_IN_FILE  "J35_CP_cmd.txt"   // 座舱 Lua 写, 本 DLL 读
#define J35_CP_MS       50                 // 读限频(20Hz)
#define J35_CP_ARG      181                // ★ 座舱内座舱盖动画参数号
#define J35_CP_SCALE    1.0                // 量程系数(写出去的值 = 通道值 * 本系数)
// ★ 2026-09-21 襟翼三档手柄的两个座舱拨钮(DLL 权威回写, 同 181 一条回调):
//   键盘 F / 着舰辅助 / 文件通道改了 g_flapTarget, 拨钮动画必须同步。
#define J35_FL_ARG_TO   579                // put_579 起飞位拨钮(0/1; 2026-09-22 由 578 更正)
#define J35_FL_ARG_LAND 580                // put_580 降落位拨钮(0/1)

static struct J35CpState
{
    long   inSeq;                  // 座舱行号(只认新行)
    double v;                      // 座舱盖动画值(0 = 关 ... 0.9 = 开)
    unsigned long long tOut;       // 读限频
    unsigned long long tLog;       // 日志限频
    unsigned nTick;                // 本函数被调用次数(用来判"引擎到底给不给这个回调")
    unsigned nCall;                // 回调被引擎调用次数(0 = 引擎没调 -> arg181 只能靠模型自己动)
} g_cp = {0};

// 出生/任务重开: 归零 + 删通道文件(与 j35SwReset 同款), 免得上一局的座舱盖位置残留
static void j35CpReset(void)
{
    memset(&g_cp, 0, sizeof(g_cp));
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_CP_IN_FILE); remove(p);
}

static void j35CpReadIn(void)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_CP_IN_FILE);
    FILE *fp = fopen(p, "r");
    if (!fp) return;                        // 文件还没出现(还没进座舱) -> 保持现状
    char line[160];
    line[0] = 0;
    if (fgets(line, sizeof(line), fp))
    {
        char tag[16] = {0};
        long seq = 0; double v = 0.0;
        if (sscanf(line, "%15s %ld %lf", tag, &seq, &v) == 3 &&
            strcmp(tag, "J35CP1") == 0 && seq > 0 && seq != g_cp.inSeq)
        {
            g_cp.inSeq = seq;
            double nv = clamp(v, 0.0, 1.0);
            if (fabs(nv - g_cp.v) >= 0.002)
            {
                unsigned long long nw = j35Ms();
                if (nw - g_cp.tLog >= 500)
                {
                    g_cp.tLog = nw;
                    j35Dbg("CP: 座舱盖动画 -> %.3f (seq=%ld)", nv, seq);
                }
                g_cp.v = nv;
            }
        }
    }
    fclose(fp);
}

static void j35CpTick(void)
{
    unsigned long long now = j35Ms();
    if (now - g_cp.tOut < (unsigned long long)J35_CP_MS) return;
    g_cp.tOut = now;
    g_cp.nTick++;
    j35CpReadIn();
    // 进了座舱 5 秒(100 帧)还没等到引擎调用回调 -> 说明这个 DCS 版本/座舱不给这个回调
    if (g_cp.nCall == 0 && g_cp.nTick == 100)
        j35Dbg("CP: ★ 引擎一直没调用 ed_fm_set_fc3_cockpit_draw_args_v2 -> 座舱 arg%d"
               " 只能靠模型自己动(本 DLL 这条路走不通)", J35_CP_ARG);
}

// ================ 尾勾联动: DCS 拦阻索"脱钩"事件 (2026-09-18 新增) ================
// DCS 拦阻索要求 EFM 在【钩由放->收】的瞬间主动推 ED_FM_EVENT_CARRIER_HOOKED, [0]=0,
//   否则 DCS 不知道钩已收 -> rope 物理永远挂着 -> "阻拦索粘在钩上下不来"。
//   (DCS API 头 wHumanCustomPhysicsAPI.h 第 1158-1177 行明确要求 EFM 实现 pop/push)
//
// 通道: 座舱 SYSTEM/cockpit_click.lua 把当前开关(0=收/1=放)写到 <SavedGames>\J35DATA\J35_HK_cmd.txt:
//     "J35HK1 <seq> <down>"   (down = 0/1, seq 单调递增)
//   本 DLL 20Hz 读取 -> 边沿检测 1->0 -> 通过 ed_fm_pop_simulation_event 推 event CARRIER_HOOKED [0]=0
//   DCS 收到 [0]=0 立即解除 rope 物理连接(API 头原话: "rope's phys will disappear immediately")
// ⚠ 之所以走文件通道: PlaneHook 命令 (cockpit_click.lua 发的 Keys.PlaneHook=69) 是给 DCS
//   内置用的, 不会路由到 ed_fm_set_command(69); 本 DLL 没法直接拿到"钩开关"信号,
//   只能 Lua 写文件。
#define J35_HK_IN_FILE  "J35_HK_cmd.txt"   // 座舱 Lua 写, 本 DLL 读
#define J35_HK_MS       50                 // 读限频(20Hz)

// ---- 尾勾全局状态(钩位置 / rope 状态 / 边沿记忆) ----
static bool  g_hookDown    = false;   // Lua 端当前写的"钩位置": true=放, false=收
static bool  g_wasHookDown = false;   // 上一帧 DLL 读到的 g_hookDown(用于 1->0 边沿检测)
static bool  g_ropeActive  = false;   // DCS 通过 push_simulation_event 通知: rope 是否还挂着
                                       //   (保护: 只有 DCS 真的知道挂上了, 我们才在收钩时弹脱开)

static struct J35HkState
{
    long             inSeq;            // 文件行号(只认新行, 避免 Lua 写一半被读到)
    unsigned long long tOut;           // 读限频
    unsigned long long tLog;           // 日志限频
} g_hk = {0};

// 出生/任务重开: 归零 + 删通道文件(免得上一局残留)
static void j35HookReset(void)
{
    memset(&g_hk, 0, sizeof(g_hk));
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_HK_IN_FILE); remove(p);
    g_hookDown    = false;   // 出生时钩在收位
    g_wasHookDown = false;
    g_ropeActive  = false;   // 还没接触缆绳
}

// 20Hz 读文件: 拿到 Lua 端的最新钩位置
static void j35HookReadIn(void)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_HK_IN_FILE);
    FILE *fp = fopen(p, "r");
    if (!fp) return;                       // 文件还没出现(还没进座舱) -> 保持现状
    char line[160];
    line[0] = 0;
    if (fgets(line, sizeof(line), fp))
    {
        char tag[16] = {0};
        long seq = 0; double down = 0.0;
        if (sscanf(line, "%15s %ld %lf", tag, &seq, &down) == 3 &&
            strcmp(tag, "J35HK1") == 0 && seq > 0 && seq != g_hk.inSeq)
        {
            g_hk.inSeq = seq;
            int nd = (down >= 0.5) ? 1 : 0;
            if (nd != g_hookDown)
            {
                unsigned long long nw = j35Ms();
                if (nw - g_hk.tLog >= 500)
                {
                    g_hk.tLog = nw;
                    j35Dbg("HK: 尾勾 -> %s (seq=%ld)", nd ? "放" : "收", seq);
                }
                g_hookDown = nd;
            }
        }
    }
    fclose(fp);
}

static void j35HookTick(void)
{
    unsigned long long now = j35Ms();
    if (now - g_hk.tOut < (unsigned long long)J35_HK_MS) return;
    g_hk.tOut = now;
    j35HookReadIn();
}

// ================ 襟翼三档手柄: J35_FL_cmd.txt (2026-09-21 新增) ================
//   座舱 put_579=起飞位拨钮(0/1), put_580=降落位拨钮(0/1), 两钮互斥(同一根三档
//   手柄: 收起 / 起飞 / 降落)。座舱 SYSTEM/cockpit_click.lua 轮询两个拨钮实际位置,
//   互斥裁决后把【档位】写到 <SavedGames>\J35DATA\J35_FL_cmd.txt:
//       "J35FL1 <seq> <pos>"   pos: 0=收起 1=起飞位 2=降落位 (seq 单调递增)
//   本 DLL 20Hz 读取 -> g_flapTarget = 0 / J35_FLAP_TO_FRAC / 1, 襟翼面仍走原来
//   的 3 秒匀速运动(外部模型 arg9/10 = g_flapState, 起飞位停在半放)。
//   键盘 F(72) 同步改成 收 -> 起飞 -> 降落 -> 收 三档循环; 145=收 146=全放不变。
//   手柄位置由 ed_fm_set_fc3_cockpit_draw_args_v2 回写 arg579/580 —— 键盘/ACL
//   改变襟翼时座舱拨钮也跟着走(与座舱盖 arg181 同一套回写)。
// ★★ 2026-09-21 晚 修"鼠标点下去没反应": 回写绝不能每帧强制执行 —— 玩家点拨钮后
//   引擎把 arg 置 1, 但 Lua 文件通道(20Hz)还没把新档位送到, 这几十毫秒里回调若
//   按旧 target 把 arg 写回 0, 拨钮就被压回、Lua 下一拍读到 0 又写 pos=0,
//   物理也不动 —— 表现就是"点了没反应"。
//   规则: 只有档位来自键盘/ACL 等【非座舱点击】来源时才回写 arg(g_flapExtSync=1);
//   座舱 J35FL1 一报到(新 seq, 无论档位变没变)立即停写, arg 交还给点击元素维护。
#define J35_FL_IN_FILE  "J35_FL_cmd.txt"   // 座舱 Lua 写, 本 DLL 读
#define J35_FL_MS       50                 // 读限频(20Hz)

static struct J35FlState
{
    long              inSeq;             // 文件序号(只认新行)
    int               pos;               // 最近一次应用的档位 0/1/2
    unsigned long long tOut;             // 读限频
} g_fl = { 0, 0, 0 };

static double j35FlapTargetOf(int pos)
{
    if (pos == 1) return J35_FLAP_TO_FRAC;
    if (pos == 2) return 1.0;
    return 0.0;
}

// 出生/任务重开: 档位归零 + 删通道文件(免上一局残留)
static void j35FlReset(void)
{
    memset(&g_fl, 0, sizeof(g_fl));
    const char *d = j35SavedGamesDir();
    if (d)
    {
        char p[600];
        snprintf(p, sizeof(p), "%s%s", d, J35_FL_IN_FILE); remove(p);
    }
    g_flapState  = 0.0;      // 任何出生方式襟翼都在收位(同 cold_start 原口径)
    g_flapTarget = 0.0;
    g_flapExtSync = 0;       // 出生不需要回写拨钮(Lua 自己核对 arg)
}

static void j35FlReadIn(void)
{
    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[600];
    snprintf(p, sizeof(p), "%s%s", d, J35_FL_IN_FILE);
    FILE *fp = fopen(p, "r");
    if (!fp) return;                        // 座舱还没写过 -> 保持收位
    char line[160];
    line[0] = 0;
    if (fgets(line, sizeof(line), fp))
    {
        char tag[16] = {0};
        long seq = 0; double pos = 0.0;
        if (sscanf(line, "%15s %ld %lf", tag, &seq, &pos) == 3 &&
            strcmp(tag, "J35FL1") == 0 && seq > 0 && seq != g_fl.inSeq)
        {
            g_fl.inSeq = seq;
            // 座舱已按 arg 实际位置(含键盘改档后 DLL 回写带动的变化)报到:
            //   不管档位变没变, 都停止 DLL 侧回写, 把 arg 主权交还给座舱。
            g_flapExtSync = 0;
            int np = (pos >= 1.5) ? 2 : (pos >= 0.5) ? 1 : 0;
            if (np != g_fl.pos)
            {
                g_fl.pos = np;
                g_flapTarget = j35FlapTargetOf(np);
                j35Dbg("FL: 襟翼手柄 -> %s (g_flapTarget=%.2f, seq=%ld)",
                       np == 2 ? "降落(全放)" : np == 1 ? "起飞位(半放)" : "收起",
                       g_flapTarget, seq);
            }
        }
    }
    fclose(fp);
}

static void j35FlTick(void)
{
    unsigned long long now = j35Ms();
    if (now - g_fl.tOut < (unsigned long long)J35_FL_MS) return;
    g_fl.tOut = now;
    j35FlReadIn();
}

// =============================================================================
//  战损失控 (2026-09-22)
// -----------------------------------------------------------------------------
//  DCS 自己的损伤系统(见 J-35.lua Damage 表 / Scripts\Aircrafts\_Common\Damage.lua)
//  负责"扣血/视觉效果/部件失效", 每次部件完整度变化时回调 ed_fm_on_damage:
//    Element = 损伤单元编号(NOSE_CENTER=0 COCKPIT=3 ENGINE_L=11 ENGINE_R=12
//              WHEEL_F=83 WHEEL_L=84 WHEEL_R=85, 自定义机身单元从 137 起)
//    factor  = 完整度 1.0=完好 -> 0.0=摧毁
//  本节只管【飞行动力学反应】: 重创 -> 不可逆失控(双发停车 + 舵面失效 + 姿态发散)。
// =============================================================================

// 起落架轮(83 前轮 / 84 左主 / 85 右主): 轮胎被打不该导致空中解体式失控
static bool j35DmgIsWheel(int e) { return e==83 || e==84 || e==85; }

static void j35DamageReset(void)
{
    for (int i=0;i<256;i++) { g_dmgCell[i] = 1.0; g_dmgCellSet[i] = false; }
    g_dmgOn = false;
    g_dmgT  = 0.0;
    g_dmgK  = 0.0;
    g_dmgPh = 0.0;
}

// 进入不可逆失控: 只执行一次的边沿动作
static void j35DamageTrigger(int element, double integrity)
{
    if (g_dmgOn) return;
    g_dmgOn = true;
    g_dmgT  = 0.0;
    g_dmgK  = 0.0;
    // 用部件号 + 当前时刻播种发散相位, 每次被击中的翻滚形态都不同
    g_dmgPh = fmod((double)element*0.731 + (double)(j35Ms()%100000ULL)*0.0013, 6.2831853);
    j35Dbg("DMG ★★ 命中部件 %d 完整度=%.2f -> 进入失控: 双发停车/舵面失效/姿态发散",
           element, integrity);
    // 双发停车: 走正常停车状态机(N1 惰转 4s -> 0, 推力/油耗/EGT/音效全部同步)
    engStopRequest(0);
    engStopRequest(1);
    // 解除自驾: 定速巡航保持/锁定直接掐掉
    g_ah.hold=0; g_ah.holdReq=0; g_ah.lastHoldReq=0;
    g_ah.lock=0; g_ah.lockReq=0;
    // 全自动着舰断开(keepThr=1: 不要在停车后反而把油门抬起来)
    j35AclDisengage("战损失控", 1);
}

// 每物理步推进失控程度(computeForces 读 g_dmgK)
static void j35DamageUpdate(double dt)
{
    if (!g_dmgOn) return;
    g_dmgT += dt;
    g_dmgK  = clamp(g_dmgT/J35_DMG_RAMP, 0.0, 1.0);
}

// ---- DCS -> EFM: 机体部件受损回调 ----
J35EFM_API void ed_fm_on_damage(int Element, double element_integrity_factor)
{
    if (g_immortal) return;                       // 无敌选项: 完全不反应
    double f = clamp(element_integrity_factor, 0.0, 1.0);
    bool wheel = j35DmgIsWheel(Element);

    // 记账(仅 0..255 标准/自定义单元; 越界单元不参与累积统计)
    if (Element >= 0 && Element < 256)
    {
        if (!g_dmgCellSet[Element] || f < g_dmgCell[Element])
        {
            g_dmgCell[Element]    = f;
            g_dmgCellSet[Element] = true;
            j35Dbg("DMG 部件 %d 完整度 %.3f%s", Element, f, wheel?"(起落架轮, 不触发失控)":"");
        }
    }

    // ★ 2026-10-05 引擎火警: 发动机单元 11/12 重创即起火
    j35FireOnDamage(Element, f);

    if (g_dmgOn) return;                          // 已失控: 后续命中只记账
    if (wheel) return;                            // 轮胎损伤不触发起飞失控

    // 规则 1: 单个机身部件重创(完整度 < 0.50)
    if (f < J35_DMG_TRIGGER) { j35DamageTrigger(Element, f); return; }

    // 规则 2: 3 个及以上机身部件中度损伤(完整度 < 0.85) -> 累积成致命伤
    int n = 0;
    for (int i=0;i<256;i++)
        if (g_dmgCellSet[i] && !j35DmgIsWheel(i) && g_dmgCell[i] < 0.85) n++;
    if (n >= 3) j35DamageTrigger(Element, f);
}

// ---- 地勤修复后复位 ----
J35EFM_API void ed_fm_repair(void)
{
    j35DamageReset();
    j35FireReset();
    j35Dbg("DMG repair: 战损失控状态已复位");
}

// ---- 有内伤时通知 DCS 需要走修复流程 ----
J35EFM_API bool ed_fm_need_to_be_repaired(void)
{
    return g_dmgOn;
}

// ---- 无敌选项变化(游戏设置) ----
J35EFM_API void ed_fm_set_immortal(bool value)
{
    g_immortal = value;
}

// ---- 任务编辑器预设故障(只记日志, 不与战损混用) ----
J35EFM_API void ed_fm_on_planned_failure(const char * failure_id)
{
    j35Dbg("DMG planned failure: %s", failure_id ? failure_id : "(null)");
}

// ---- DCS -> EFM: 通知 hook 挂上 / 已脱开 (2026-09-18 新增) ----
//   in.event_params[0] = 1 -> DCS 刚把钩挂上, 更新 g_ropeActive
//   in.event_params[0] = 2 -> DCS 通知已脱开(物理上断), 更新 g_ropeActive
//   其他值 -> 忽略
J35EFM_API void ed_fm_push_simulation_event(const ed_fm_simulation_event& in)
{
    if (in.event_type == ED_FM_EVENT_CARRIER_HOOKED)
    {
        if (in.event_params[0] == 1.0)
        {
            if (!g_ropeActive)
                j35Dbg("HK: DCS 通知 hook 已挂上");
            g_ropeActive = true;
            if (g_acl.on) g_acl.trapped = 1;   // ★ v4: 全自动着舰 -> 下一 tick 进"上舰减速"
        }
        else if (in.event_params[0] == 2.0)
        {
            if (g_ropeActive)
                j35Dbg("HK: DCS 通知 hook 已脱开 (物理断)");
            g_ropeActive = false;
        }
    }
}

// ---- EFM -> DCS: 边沿钩由放->收时弹"释放 rope"事件 (2026-09-18 新增) ----
//   关键: 上一帧 down=1 (放) + 本帧 down=0 (收) 且 rope 确实在挂着 -> 一次 [0]=0 事件
//   DCS 收到 [0]=0 立即解除 rope (rope's phys disappear immediately),
//   之后 DCS 会推回 [0]=2 通知我物理已断 -> 上面的 push_simulation_event 把 g_ropeActive 清零
//   -> 下一个边沿就不会再误弹。
J35EFM_API bool ed_fm_pop_simulation_event(ed_fm_simulation_event& out)
{
    bool wasDown = g_wasHookDown;
    bool nowDown  = g_hookDown;
    g_wasHookDown = nowDown;

    if (wasDown && !nowDown && g_ropeActive)
    {
        out.event_type = ED_FM_EVENT_CARRIER_HOOKED;
        out.event_params[0] = 0.0;     // 0 = 释放 rope
        g_ropeActive = false;          // 已弹: 不再弹(等 DCS [0]=2 回来复原)
        j35Dbg("HK: 弹 ED_FM_EVENT_CARRIER_HOOKED [0]=0 (钩由放->收, 请 DCS 松 rope)");
        return true;
    }
    return false;
}

// ---- 座舱参数回调: DCS 每帧把它要画的座舱参数数组交给本 DLL 填 -----------------
J35EFM_API void ed_fm_set_fc3_cockpit_draw_args_v2(float * array, size_t size)
{
    if (!array) return;
    if (g_cp.nCall == 0)
        j35Dbg("CP: ★ 座舱参数回调首次被调用 size=%zu -> arg%d 由本 DLL 写",
               size, J35_CP_ARG);
    g_cp.nCall++;
    if (size > (size_t)J35_CP_ARG)
        array[J35_CP_ARG] = (float)(g_cp.v * J35_CP_SCALE);
    // ★ 2026-10-05 告警参数走座舱通道(外部 arg 9020~9023 是 TVC 喷口, 曾被撞坏):
    //   1900=左发火 1901=右发火(J35_FIRE.cpp) 1902=内油kg 1903=起落架(1放) 1904=液压阀(1开)
    //   座舱 Bridge 用 get_cockpit_draw_argument_value 读取。
    j35FireWriteCockpit(array, size);
    if (size > (size_t)1904)
    {
        array[1902] = (float)g_fuel;
        array[1903] = (float)g_gearState;
        array[1904] = g_sw.hyd ? 1.0f : 0.0f;
    }
    // ★ 2026-09-28 爆索 arg183: 由 J35_MDC.cpp 代写
    j35MdcWrite(array, size);
    // ★ 2026-09-21 襟翼手柄拨钮: 仅在键盘/ACL 改档后的同步窗口内回写 arg579/580。
    //   平时(座舱鼠标/键位点击驱动)【绝不能碰】这两个 arg, 否则会把玩家刚点开的
    //   拨钮每帧压回旧位(2026-09-21 晚"点下去没反应"的根因)。座舱文件通道报到后
    //   g_flapExtSync 清 0, 主权交还点击元素。
    if (g_flapExtSync && size > (size_t)J35_FL_ARG_LAND)
    {
        int toIs   = (g_flapTarget > 0.25 && g_flapTarget < 0.75) ? 1 : 0;
        int landIs = (g_flapTarget >= 0.75) ? 1 : 0;
        array[J35_FL_ARG_TO]   = (float)toIs;
        array[J35_FL_ARG_LAND] = (float)landIs;
    }
}

// =============================================================================
//  j35CobraTick  —  钟摆/眼镜蛇机动触发 (2026-09-29)
// -----------------------------------------------------------------------------
//  座舱 put_560 写 J35_COBRA_cmd.txt, 本函数 20Hz 读。
//  满足条件时把 g_cobra.phase 置 1(PULL), computeForces 里的状态机接管俯仰。
//  条件: 空中 + 速度>180m/s + 襟翼收 + 起落架收 + 无战损 + 飞控不在 OVERRIDE
// =============================================================================
static void j35CobraTick(void)
{
    static unsigned long long s_tLast = 0;
    static long s_lastSeq = 0;
    unsigned long long now = j35Ms();
    if (now - s_tLast < 50) return;   // 20Hz
    s_tLast = now;

    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[700];
    snprintf(p, sizeof(p), "%sJ35_COBRA_cmd.txt", d);
    FILE *fp = fopen(p, "r");
    if (!fp) return;
    char line[160] = {0};
    if (fgets(line, sizeof(line), fp))
    {
        char tag[16] = {0}; long seq = 0; int cmd = 0;
        if (sscanf(line, "%15s %ld %d", tag, &seq, &cmd) == 3 &&
            strcmp(tag, "J35COB1") == 0 && seq > 0 && seq != s_lastSeq)
        {
            s_lastSeq = seq;
            if (cmd == 1 && g_cobra.phase == 0)
            {
                // 触发条件检查
                double V = sqrt(g_velBody.x*g_velBody.x + g_velBody.y*g_velBody.y + g_velBody.z*g_velBody.z);
                if (V > 180.0 && g_gearState < 0.5 && g_flapState < 0.1 && !g_dmgOn)
                {
                    g_cobra.phase = 1;
                    g_cobra.timer = 0;
                    j35Dbg("COBRA: 触发! V=%.0f m/s", V);
                }
                else
                {
                    j35Dbg("COBRA: 条件不满足 V=%.0f gear=%.1f flap=%.1f dmg=%d",
                           V, g_gearState, g_flapState, g_dmgOn?1:0);
                }
            }
        }
    }
    fclose(fp);
}

// =============================================================================
// ★ 2026-10-06 反推开关: 座舱写 J35_REV_cmd.txt("J35REV1 <seq> <0|1>"),
//   本函数 20Hz 读。1=开反推(倒车), 0=关。安全条件在 computeForces 里守。
// =============================================================================
static void j35RevTick(void)
{
    static unsigned long long s_tLast = 0;
    static long s_lastSeq = 0;
    unsigned long long now = j35Ms();
    if (now - s_tLast < 50) return;   // 20Hz
    s_tLast = now;

    const char *d = j35SavedGamesDir();
    if (!d) return;
    char p[700];
    snprintf(p, sizeof(p), "%sJ35_REV_cmd.txt", d);
    FILE *fp = fopen(p, "r");
    if (!fp) return;
    char line[160] = {0};
    if (fgets(line, sizeof(line), fp))
    {
        char tag[16] = {0}; long seq = 0; int cmd = 0;
        if (sscanf(line, "%15s %ld %d", tag, &seq, &cmd) == 3 &&
            strcmp(tag, "J35REV1") == 0 && seq > 0 && seq != s_lastSeq)
        {
            s_lastSeq = seq;
            bool want = (cmd == 1);
            if (want != g_revOn)
            {
                g_revOn = want;
                j35Dbg("REV: %s", want ? "开(倒车)" : "关");
            }
        }
    }
    fclose(fp);
}

J35EFM_API void ed_fm_simulate(double dt)
{
    if (dt<=0.0) dt=0.01;
    if (dt>0.5)  dt=0.5;

    j35DamageUpdate(dt); // ★ 2026-09-22 战损: 推进失控程度 g_dmgK (0..1)

    // ★ 2026-10-05 引擎火警: 灭火手柄动作 2s 后解除
    j35FireTick(dt, g_sw.fired[0], g_sw.fired[1]);

    j35AhTick();         // 定速巡航: 读 J35_AH_cmd.txt -> 接合/断开/锁定 (20Hz)
    j35AhControl(dt);    //   速度保持 PI 控制律

    j35AclTick();        // 全自动着舰 v4: 读 J35_ACL_cmd.txt + J35_AL_link.txt -> 接合/模式 (40Hz)
    j35AclControl(dt);   //   油门 PI(进近速度) + 俯仰 PD(航迹角) + 滚转坡度保持

    j35SwTick();         // 系统开关: 读 J35_SW_cmd.txt -> 起动联锁/熄火/灭火 (20Hz, 2026-09-17)

    j35CpTick();         // 座舱内座舱盖 arg181: 读 J35_CP_cmd.txt (20Hz, 2026-09-17)
    j35MdcTick();        // 座舱内爆索 arg183: 读 J35_MDC_cmd.txt (20Hz, 2026-09-28)
    j35FcsTick();        // 飞控模式: 读 J35_FCS_cmd.txt (20Hz, 2026-09-29)
    j35CruiseTick();     // 巡航导弹: 读 J35_TGT_cmd.txt -> 堆扫描 add_pos 注入 (2Hz, 2026-09-30)

    // ★ 2026-09-29 钟摆机动: 读座舱命令触发 cobra 状态机
    j35CobraTick();

    j35FlTick();         // 襟翼三档手柄: 读 J35_FL_cmd.txt -> 收/起飞/降落 (20Hz, 2026-09-21)
    j35RevTick();        // 反推开关: 读 J35_REV_cmd.txt -> 倒车 (20Hz, 2026-10-06)

    updateCtlSlew(dt);   // 杆位平滑: 消除键控瞬间满舵

    j35LosUpdate();      // 真视线: 读数据链给的角度 -> 用本步姿态换算到机体系 -> 写给座舱
    j35SttUpdate();      // HUD STT 锁定框: J35HM 世界系闭环换算(钉死目标, 不随姿态漂移) 2026-09-28
    j35TgpTick(dt);      // 对地吊舱核心: 读 J35_TGP_cmd.txt -> 云台/增稳/跟踪/激光 -> 灌 g_pod (2026-09-27)
    j35PodUpdate();      // 对地吊舱: 同一套换算, 服务吊舱锁定的地面目标

    // ---- 诊断：首次调用 + 每 ~250ms 记一行当前状态 ----
    static bool firstSim = true;
    static unsigned long long tLast = 0;
    static unsigned long simCnt = 0;
    if (firstSim)
    {
        firstSim = false;
        tLast = j35Ms();
        j35Dbg("SIM first dt=%.4f", dt);
    }
    simCnt++;
    unsigned long long now = j35Ms();
    if (now - tLast >= 250)
    {
        tLast = now;
        double V = sqrt(g_velBody.x*g_velBody.x + g_velBody.y*g_velBody.y + g_velBody.z*g_velBody.z);
        // 顺带把两台发动机的状态打出来, 方便核对起动过程:
        //   st: 0=OFF 1=CRANK 2=LIGHTOFF 3=RUNNING 4=SHUTDOWN, lit: 是否已点火
        // ★ 2026-09-18 诊断加列: aoa=攻角(度) nz=法向过载 tr=配平基准(度, 迎角域) gs=起落架
        //   pit=俯仰姿态(度) qz=俯仰角速度(度/秒)
        j35Dbg("SIM #%u V=%.1f sp=%.2f sr=%.2f sy=%.2f thr=%.2f fuel=%.0f n1=%.2f/%.2f st=%d,%d lit=%d,%d aoa=%+.1f nz=%+.2f tr=%+.1f gs=%.0f pit=%+.1f qz=%+.1f acl=%d dmg=%d/%.2f",
               (unsigned)simCnt, V, g_stickPitch, g_stickRoll, g_stickYaw,
               g_throttleAxis, g_fuel, g_n1[0], g_n1[1],
               g_engState[0], g_engState[1], g_engLit[0]?1:0, g_engLit[1]?1:0,
               g_dbgAoa*57.29578, g_dbgNz, g_trimA*57.29578, g_gearState,
               g_los.pit*57.29578, g_omega[2]*57.29578, g_acl.on?1:0,
               g_dmgOn?1:0, g_dmgK);
        // ★ v4: 着舰辅助状态(mode/link/hook)单列一行, 只在状态变化时记, 免得刷屏
        static int sAclDbg = -1;
        int aclNow = g_acl.on * 100 + g_acl.mode * 10 + (g_acl.linkOk ? 1 : 0);
        if (aclNow != sAclDbg)
        {
            sAclDbg = aclNow;
            j35Dbg("ACL 状态: on=%d mode=%d link=%d (0=进近/1=上舰/2=逃逸)", 
                   g_acl.on, g_acl.mode, g_acl.linkOk);
        }
        // 诊断: DCS 问过的引擎参数(偏移=调用次数:最后一次返回的值)。用来确认座舱读数
        //   和发动机音效到底取自哪条参数: 0=RPM 1=RELATED 2=CORE_RPM 3=CORE_RELATED
        //   4/6=推力 5/7=相对推力 12=EGT 14=油耗 15=燃烧 17=起动机 26=可操作性
        {
            char q[640]; int p = 0; q[0] = 0;
            for (int e = 0; e < 2; e++)
                for (int L = 0; L < 40; L++)
                    if (g_pqCnt[e][L] && p < (int)sizeof(q) - 48)
                        p += snprintf(q+p, sizeof(q)-p, " e%d[%d]=%u:%.3f",
                                      e, L, g_pqCnt[e][L], g_pqVal[e][L]);
            if (p > 0) j35Dbg("PARAM%s", q);
        }
    }

    g_cF = Vec3();
    g_cM = Vec3();
    g_burnNow = 0;

    updateEngines(dt);
    computeForces(dt);
    updateConfigAnims(dt);
    updateNWS(dt);       // ★ 2026-09-19 前轮转向: 脚蹬 -> g_nwsYaw -> WHEEL_YAW
}

J35EFM_API void ed_fm_set_atmosphere(double h,double t,double a,double ro,double p,
                                     double wind_vx,double wind_vy,double wind_vz)
{
    (void)t;(void)p;
    g_alt = h;
    g_rho = ro;
    g_sos = (a>100)?a:340.0;
    g_windWorld = Vec3(wind_vx,wind_vy,wind_vz);
}

J35EFM_API void ed_fm_set_clouds_density(const atmo_clouds_and_precipation &info)
{
    (void)info;
}

// ★ 2026-09-18: 正下方地表高度(h=纯地形/海面, h_obj=含物体 —— 在航母上空就是甲板高)。
//   着舰辅助 v2 的"防触海平飞保护"用 g_surfH(甲板~20m, 所以正常下滑道不受影响)。
J35EFM_API void ed_fm_set_surface(double h,double h_obj,unsigned surface_type,
                                   double normal_x,double normal_y,double normal_z)
{
    (void)surface_type;(void)normal_x;(void)normal_y;(void)normal_z;
    g_surfH    = h;
    g_surfHObj = h_obj;
}

J35EFM_API void ed_fm_set_current_mass_state(double mass,
                                             double center_of_mass_x,
                                             double center_of_mass_y,
                                             double center_of_mass_z,
                                             double moment_of_inertia_x,
                                             double moment_of_inertia_y,
                                             double moment_of_inertia_z)
{
    g_mass = (mass>1000)?mass:g_mass;
    g_cg   = Vec3(center_of_mass_x,center_of_mass_y,center_of_mass_z);
    if (moment_of_inertia_x>1000)  g_Ixx=moment_of_inertia_x;
    if (moment_of_inertia_y>1000)  g_Iyy=moment_of_inertia_y;
    if (moment_of_inertia_z>1000)  g_Izz=moment_of_inertia_z;
}

J35EFM_API void ed_fm_set_current_state(double ax,double ay,double az,
                                        double vx,double vy,double vz,
                                        double px,double py,double pz,
                                        double omegadotx,double omegadoty,double omegadotz,
                                        double omegax,double omegay,double omegaz,
                                        double quaternion_x,double quaternion_y,
                                        double quaternion_z,double quaternion_w)
{
    (void)ax;(void)ay;(void)az;(void)px;(void)py;(void)pz;
    (void)omegadotx;(void)omegadoty;(void)omegadotz;
    (void)omegax;(void)omegay;(void)omegaz;
    (void)quaternion_x;(void)quaternion_y;(void)quaternion_z;(void)quaternion_w;
}

J35EFM_API void ed_fm_set_current_state_body_axis(double ax,double ay,double az,
                                                  double vx,double vy,double vz,
                                                  double wind_vx,double wind_vy,double wind_vz,
                                                  double omegadotx,double omegadoty,double omegadotz,
                                                  double omegax,double omegay,double omegaz,
                                                  double yaw,double pitch,double roll,
                                                  double common_angle_of_attack,
                                                  double common_angle_of_slide)
{
    (void)ax;(void)ay;(void)az;(void)omegadotx;(void)omegadoty;(void)omegadotz;
    // ★ 2026-09-14: 真视线换算要用本机姿态 —— 以前 yaw/pitch/roll 是直接丢掉的,
    //   现在存下来(弧度, 见 wHumanCustomPhysicsAPI.h 注释)。yaw 只是顺手留档: 进来的
    //   真视线方位角本来就是"相对机头"的, 换算只用得上 pit / rol。
    g_los.yaw = yaw;  g_los.pit = pitch;  g_los.rol = roll;  g_los.attOk = true;
    g_rollAtt = roll;                       // 镜像给气动段(FBW 自动配平算转弯目标过载)
    g_velBody = Vec3(vx,vy,vz);
    g_windBody= Vec3(wind_vx,wind_vy,wind_vz);
    g_omega[0]= omegax; g_omega[1]=omegay; g_omega[2]=omegaz;
    g_aoaDCS  = common_angle_of_attack;
    g_aosDCS  = common_angle_of_slide;
}

// =============================================================================
// 输入命令处理 (CMD 重写)
// -----------------------------------------------------------------------------
//  FC3 壳子下的实测结论:
//   * 离散/按键类命令以"事件"到达, value 基本无意义(多为 0),按下/释放成对下发;
//     旧实现按 |v|>0.5 判断 -> 几乎所有按键/开关都不生效, 这是"操纵无响应"主因之一。
//   * 模拟轴命令(2001..2006)若 DCS 路由给 EFM 会持续带量值; 现改为轴/离散双轨,
//     谁最近到达谁生效(与社区 EFM 模板一致)。
//   * 配平(93~97)作为杆位偏置叠加到控制输入, 供平飞改平使用(98/99 方向舵配平已删)。
// =============================================================================
J35EFM_API void ed_fm_set_command(int command, float value)
{
    // ---- 诊断: 每条命令限流 1 条/秒, 用于核对 DCS 到底路由了哪些输入 ----
    {
        static unsigned long long lastLog[8192];
        static unsigned long long lastElse = 0;
        unsigned long long now = j35Ms();
        if (command >= 0 && command < 8192)
        {
            if (now - lastLog[command] >= 1000)
            {
                lastLog[command] = now;
                j35Dbg("CMD c=%d v=%+.3f", command, value);
            }
        }
        else if (now - lastElse >= 250)
        {
            lastElse = now;
            j35Dbg("CMD c=%d v=%+.3f", command, value);
        }
    }

    switch (command)
    {
    // ---- 模拟轴 (仅无按键保持时接管目标, 避免轴=0 覆盖正在按的键) ----
    case 2001:   // 俯仰轴
        g_pitchAx = clamp((double)value,-1.0,1.0);
        g_pitchAnalog = true;
        // ★ 2026-09-18 着舰辅助: 轴真变了(防引擎同值重发)才算玩家动杆 -> 断开
        if (g_acl.on)
        {
            if (g_acl.paxValid && fabs(g_pitchAx - g_acl.paxLast) < 0.05) break;
            j35AclDisengage("玩家动杆(俯仰轴)");
        }
        g_acl.paxLast = g_pitchAx; g_acl.paxValid = 1;
        if (g_pitchKey == 0) g_reqPitch = g_pitchAx;
        break;
    case 2002:   // 滚转轴
        g_rollAx = clamp((double)value,-1.0,1.0);
        g_rollAnalog = true;
        if (g_acl.on)
        {
            if (g_acl.raxValid && fabs(g_rollAx - g_acl.raxLast) < 0.05) break;
            j35AclDisengage("玩家动杆(滚转轴)");
        }
        g_acl.raxLast = g_rollAx; g_acl.raxValid = 1;
        if (g_rollKey == 0) g_reqRoll = g_rollAx;
        break;
    case 2003:   // 方向舵轴
        g_yawAx = clamp((double)value,-1.0,1.0);
        g_yawAnalog = true;
        if (g_yawKey == 0) g_reqYaw = g_yawAx;
        break;
    case 2004:   // 油门总轴 FC3: -1..1(与社区一致取反相, 满载推进)
    case 2005:   // 左油门(分侧时)
    case 2006:   // 右油门(分侧时)
    {
        // ★ 2026-09-17 定速巡航: 锁定时手动油门轴【整个忽略】;
        //   未锁定时, 轴真的动了(防引擎重复派发同一值)才算"手动控制" -> 断开定速巡航。
        double axv = clamp(0.5*(-(double)value+1.0), 0.0, 1.0);
        // ★ 2026-09-18 着舰辅助: 手动油门轴(真变了才算) -> 断开
        if (g_acl.on)
        {
            if (g_acl.axValid && fabs(axv - g_acl.axLast) < 0.01) break;
            j35AclDisengage("玩家手动油门(轴)");
        }
        g_acl.axLast  = axv;
        g_acl.axValid = 1;
        if (g_ah.lock) break;
        if (g_ah.hold)
        {
            if (g_ah.axValid && fabs(axv - g_ah.axLast) < 0.01) break;  // 同值重发, 不是人手
            g_ah.hold = 0;
            j35Dbg("AH 手动油门轴(%.3f -> %.3f) -> 定速巡航断开", g_ah.axLast, axv);
        }
        g_ah.axLast  = axv;
        g_ah.axValid = 1;
        g_throttleAxis = axv;
        break;
    }

    // ★ 2026-09-17 修"玩家驾驶无加力": 补上固定翼油门轴命令 3001/3002/3003/3004
    //   原 EFM 只接直升机 2004/2005/2006 和键盘 161/163, HOTAS 油门轴满推也不进 AB
    //   DCS 固定翼 throttle axis value 范围 0..1 (0=IDLE, 1=MAX/MAX AB), 直接写
    case 3001:  // Throttle (Left)
    case 3002:  // Throttle (Right)
    case 3003:  // Collective (Left)
    case 3004:  // Collective (Right)
        // ★ 2026-09-18 着舰辅助: HOTAS 油门轴真变了 -> 断开
        if (g_acl.on)
        {
            double av = clamp((double)value, 0.0, 1.0);
            if (g_acl.axValid && fabs(av - g_acl.axLast) < 0.01) break;
            j35AclDisengage("玩家手动油门(HOTAS轴)");
        }
        if (g_ah.lock) break;
        if (g_ah.hold) { g_ah.hold = 0; j35Dbg("AH HOTAS 油门轴 -> 定速巡航断开"); }
        g_throttleAxis = clamp((double)value, 0.0, 1.0);
        g_ah.axLast  = g_throttleAxis;
        g_ah.axValid = 1;
        g_acl.axLast = g_throttleAxis;
        g_acl.axValid = 1;
        break;

    // ---- 离散俯仰杆 (195 拉杆=抬头; 193 推杆=低头; 轴符号由 SIGN_PITCH 消化) ----
    case 195:
        if (g_acl.on) j35AclDisengage("玩家动杆(拉杆)");
        g_pitchKey = +1; g_pitchAnalog = false; g_reqPitch = +1.0;  break;
    case 196:
        if (g_pitchKey == +1){ g_pitchKey = 0; g_reqPitch = g_pitchAnalog?g_pitchAx:0.0; }
        break;
    case 193:
        if (g_acl.on) j35AclDisengage("玩家动杆(推杆)");
        g_pitchKey = -1; g_pitchAnalog = false; g_reqPitch = -1.0;  break;
    case 194:
        if (g_pitchKey == -1){ g_pitchKey = 0; g_reqPitch = g_pitchAnalog?g_pitchAx:0.0; }
        break;

    // ---- 离散滚转杆 (199 右压=右滚; 197 左压=左滚) ----
    case 199:
        if (g_acl.on) j35AclDisengage("玩家动杆(右压杆)");
        g_rollKey = +1; g_rollAnalog = false; g_reqRoll = +1.0;      break;
    case 200:
        if (g_rollKey == +1){ g_rollKey = 0; g_reqRoll = g_rollAnalog?g_rollAx:0.0; }
        break;
    case 197:
        if (g_acl.on) j35AclDisengage("玩家动杆(左压杆)");
        g_rollKey = -1; g_rollAnalog = false; g_reqRoll = -1.0;      break;
    case 198:
        if (g_rollKey == -1){ g_rollKey = 0; g_reqRoll = g_rollAnalog?g_rollAx:0.0; }
        break;

    // ---- 离散方向舵 (203 右舵; 201 左舵) ----
    case 203:
        g_yawKey = +1; g_yawAnalog = false; g_reqYaw = +1.0;         break;
    case 204:
        if (g_yawKey == +1){ g_yawKey = 0; g_reqYaw = g_yawAnalog?g_yawAx:0.0; }
        break;
    case 201:
        g_yawKey = -1; g_yawAnalog = false; g_reqYaw = -1.0;         break;
    case 202:
        if (g_yawKey == -1){ g_yawKey = 0; g_reqYaw = g_yawAnalog?g_yawAx:0.0; }
        break;

    // ---- 配平 (等效杆位偏置) ----
    // ★ 方向舵配平(98/99)已删(2026-09-17 应用户要求); 97 键盘回中也只剩俯仰/滚转。
    case 95:  g_pitchTrim = clamp(g_pitchTrim + 0.02,-0.7,0.7); break;  // 抬头配平
    case 96:  g_pitchTrim = clamp(g_pitchTrim - 0.02,-0.7,0.7); break;  // 低头配平
    case 93:  g_rollTrim  = clamp(g_rollTrim  - 0.015,-0.5,0.5); break; // 左坡度配平
    case 94:  g_rollTrim  = clamp(g_rollTrim  + 0.015,-0.5,0.5); break; // 右坡度配平
    case 97:  g_pitchTrim = g_rollTrim = 0.0;               // 配平回中(键盘: 俯仰+滚转回中)
        break;

    // ---- 油门离散 (键/增量) ----
    // ★ 2026-09-17 定速巡航: 锁定时这些命令全忽略(不能控制油门);
    //   未锁定时手动油门 -> 断开定速巡航, 油门还给飞行员。
    case 161: case 163: case 1032:                       // 加油门
        if (g_acl.on) j35AclDisengage("玩家手动加油门");
        if (g_ah.lock) break;
        if (g_ah.hold) { g_ah.hold = 0; j35Dbg("AH 手动加油门 -> 定速巡航断开"); }
        g_throttleAxis = clamp(g_throttleAxis+0.025,0.0,1.0);
        break;
    case 162: case 164: case 1033:                       // 收油门
        if (g_acl.on) j35AclDisengage("玩家手动收油门");
        if (g_ah.lock) break;
        if (g_ah.hold) { g_ah.hold = 0; j35Dbg("AH 手动收油门 -> 定速巡航断开"); }
        g_throttleAxis = clamp(g_throttleAxis-0.025,0.0,1.0);
        break;

    // ---- 发动机开关/起动 ----
    // ★ 2026-09-13: 不再直接开"油门->转速"的开关, 而是触发下面的起动/停车状态机。
    //   冷舱出生 N1 保持 0, 只有收到这里的命令才起转(座舱里点 arg107/108)。
    // ★ 2026-09-17 起动联锁: 六个系统开关(总电源/液压/燃油/刹车/发电机/蓄电池)
    //   没全开时, 键盘起动命令一律拦下(座舱开关 put_591/592 拨起动位同样受限)。
    case 309:  if (j35SysReady()) { engStartRequest(0); engStartRequest(1); }
               else j35Dbg("ENG START blocked: 系统联锁未通(总电源/液压/燃油/刹车/发电机/蓄电池)");
               break;                                          // 双发起动
    case 310:  engStopRequest(0);  engStopRequest(1);  break;   // 双发停车
    case 311:  if (j35SysReady()) engStartRequest(0);
               else j35Dbg("ENG1 START blocked: 系统联锁未通");
               break;                                          // 左发起动
    case 313:  engStopRequest(0);  break;                       // 左发停车
    case 312:  if (j35SysReady()) engStartRequest(1);
               else j35Dbg("ENG2 START blocked: 系统联锁未通");
               break;                                          // 右发起动
    case 314:  engStopRequest(1);  break;                       // 右发停车

    // ---- 起落架 ----
    case 68:   if (pulseOK(68)) g_gearTarget = (g_gearTarget>0.5)?0.0:1.0; break;
    case 430:  g_gearTarget = 0.0; break;                 // 收起
    case 431:  g_gearTarget = 1.0; break;                 // 放下

    // ★ 2026-09-22 前轮转向锁(iCmd437; 座舱 put_574 开关, 0=锁定 1=转向)
    //   value>0.5 -> g_nwsLock=false(可转向); value<=0.5 -> g_nwsLock=true(锁定)
    case 437:  g_nwsLock = (value <= 0.5); break;

    // ---- 襟翼 (2026-09-21: 三档 收起/起飞/降落) ----
    //   F 键(72): 收 -> 起飞(半放) -> 降落(全放) -> 收; 145 收起; 146 全放。
    //   座舱拨钮 put_579/580 的权威通道是 J35_FL_cmd.txt(见 j35FlReadIn),
    //   键盘改了目标后, 手柄 arg579/580 由座舱参数回调回写。
    case 72:   if (pulseOK(72)) {
                   if (g_flapTarget < 0.25)      g_flapTarget = J35_FLAP_TO_FRAC;
                   else if (g_flapTarget < 0.75) g_flapTarget = 1.0;
                   else                           g_flapTarget = 0.0;
                   g_flapExtSync = 1;             // 键盘改档 -> 回写座舱拨钮
               } break;
    case 145:  g_flapTarget = 0.0; g_flapExtSync = 1; break;   // 收
    case 146:  g_flapTarget = 1.0; g_flapExtSync = 1; break;   // 放(降落位)

    // ---- 减速板 ----
    case 73:   if (pulseOK(73)) g_brakeTarget = (g_brakeTarget>0.5)?0.0:1.0; break;
    case 147:  g_brakeTarget = 0.0; break;                // 关
    case 148:  g_brakeTarget = 1.0; break;                // 开

    // ---- 机轮刹车 (踩/松, 重复事件只置状态, 无需去抖) ----
    case 74:   g_wheelBrake = 1.0; break;
    case 75:   g_wheelBrake = 0.0; break;

    default:
        break;
    }
}

J35EFM_API bool ed_fm_change_mass(double &delta_mass,
                                  double &delta_mass_pos_x,
                                  double &delta_mass_pos_y,
                                  double &delta_mass_pos_z,
                                  double &delta_mass_moment_of_inertia_x,
                                  double &delta_mass_moment_of_inertia_y,
                                  double &delta_mass_moment_of_inertia_z)
{
    if (g_burnNow > 1e-6)
    {
        delta_mass = g_burnNow;
        delta_mass_pos_x = 1.5;   // 燃料主要在前机身/主翼整体油箱
        delta_mass_pos_y = -0.6;
        delta_mass_pos_z = 0.0;
        delta_mass_moment_of_inertia_x = 0;
        delta_mass_moment_of_inertia_y = 0;
        delta_mass_moment_of_inertia_z = 0;
        g_burnNow = 0;
        return true;
    }
    return false;
}

J35EFM_API void ed_fm_set_internal_fuel(double fuel)
{
    g_fuel = clamp(fuel, 0.0, kFuelMax);
}

J35EFM_API double ed_fm_get_internal_fuel(void)
{
    return g_fuel;
}

J35EFM_API void ed_fm_set_external_fuel(int station,double fuel,double x,double y,double z)
{
    (void)station;(void)fuel;(void)x;(void)y;(void)z;
}

J35EFM_API double ed_fm_get_external_fuel(void)
{
    return 0.0;
}

J35EFM_API void ed_fm_refueling_add_fuel(double fuel)
{
    g_fuel = clamp(g_fuel+fuel, 0.0, kFuelMax);
}

J35EFM_API void ed_fm_set_draw_args(EdDrawArgument *drawargs, size_t size)
{
    if (!drawargs) return;
    // ---- 诊断：每 ~500ms 记录一次引擎传入的舵面参数 ----
    {
        static unsigned long long tLog = 0;
        unsigned long long nw = j35Ms();
        if (nw - tLog >= 500)
        {
            tLog = nw;
            j35Dbg("DRAW n=%zu a9=%.2f a10=%.2f a11=%.2f a12=%.2f a15=%.2f a16=%.2f a17=%.2f a28=%.2f a29=%.2f a89=%.2f a90=%.2f",
                   size,
                   (size>9)?drawargs[9].f:0.f, (size>10)?drawargs[10].f:0.f,
                   (size>11)?drawargs[11].f:0.f, (size>12)?drawargs[12].f:0.f,
                   (size>15)?drawargs[15].f:0.f, (size>16)?drawargs[16].f:0.f,
                   (size>17)?drawargs[17].f:0.f,
                   (size>28)?drawargs[28].f:0.f, (size>29)?drawargs[29].f:0.f,
                   (size>89)?drawargs[89].f:0.f, (size>90)?drawargs[90].f:0.f);
        }
    }
    // 舵面可视化（SFM 不再驱动，EFM 负责）。参数可能因 EDM 不同需调整：
    // 9/10 副翼？ J-35 EDM: 11右副翼 12左副翼 15右平尾 16左平尾 17方向舵
    if (size > 17)
    {
        drawargs[9].f  = (float)g_flapState;    // 右襟翼
        drawargs[10].f = (float)g_flapState;    // 左襟翼
        drawargs[11].f = (float)( 0.55*clamp(SIGN_ROLL*g_stickRoll,-1.0,1.0)); // 右副翼
        drawargs[12].f = (float)(-0.55*clamp(SIGN_ROLL*g_stickRoll,-1.0,1.0)); // 左副翼
        drawargs[15].f = (float)( 0.60*clamp(SIGN_PITCH*g_stickPitch,-1.0,1.0)); // 右平尾
        drawargs[16].f = (float)( 0.60*clamp(SIGN_PITCH*g_stickPitch,-1.0,1.0)); // 左平尾
        drawargs[17].f = (float)(-0.75*clamp(SIGN_YAW*g_stickYaw,-1.0,1.0));   // 方向舵
        // ★ 2026-09-19: 模型方向舵分 arg17/18 两个舵面, 旧版只写 17 —— 18 一并驱动
        if (size > 18)
            drawargs[18].f = drawargs[17].f;                                   // 方向舵(第二舵面, 同值)
    }
    // ★ 2026-09-28 座舱操控杆(真实杆量, 非姿态):
    //   外部模型 arg700=滚转杆 / arg701=俯仰杆(EDM 实测绑定, 与口头编号相反);
    //   mainpanel_init.lua 的 CreateSimpleConnectedGauge(700,700)/(701,701)
    //   每帧镜像进座舱驱动杆动画。
    //   g_stickPitch/Roll 是限速平滑后的实际杆位(含配平/自动驾驶), 范围 -1~+1。
    //   方向反了在 EDM 里翻动画, 或在这里对 f 取负。
    if (size > 701)
    {
        drawargs[700].f = (float)clamp(g_stickRoll,  -1.0, 1.0);
        drawargs[701].f = (float)clamp(g_stickPitch, -1.0, 1.0);
    }
    // ★ 2026-09-30 座舱油门杆动画(arg546): 分段映射(慢车0/半程0.4/军推0.75/
    //   过加力卡位0.78/全加力1.0), mainpanel_init.lua 的
    //   CreateConnectedGauge(546,546,{0,1},{0,1}) 每帧镜像进座舱。
    //   取代旧 Lua performClickableAction 同步(BTN 不吃小数值, 点不准)。
    if (size > 546)
        drawargs[546].f = (float)throttleLeverAnim(g_throttleAxis);
    // ---- ★ 2026-09-29 TVC 矢量喷口动画 (arg9020~9023) ----
    //   用户 EDM 约定: 9020=右发偏航(-1右/+1左) 9021=右发俯仰(-1下/+1上)
    //                 9023=左发偏航(-1右/+1左) 9022=左发俯仰(-1下/+1上)
    //   数据源 = 限速平滑后的实际杆位(含配平/自动驾驶/眼镜蛇接管), 与舵面同口径。
    //   俯仰: 拉杆抬头 -> 喷口上偏; 差动滚转: 右滚时右喷下偏左喷上偏(力矩绕纵轴)。
    //   偏航: 蹬右舵 -> 双喷口右偏。
    //   眼镜蛇机动期间: 俯仰通道强制满偏(随状态机杆位), 差动置零防干扰。
    if (size > 9023)
    {
        double tvcP = SIGN_TVC_PITCH * clamp(g_stickPitch, -1.0, 1.0);  // 抬头为正(上偏)
        double tvcY = SIGN_TVC_YAW   * clamp(g_stickYaw,   -1.0, 1.0);  // 右偏为负(-1)
        double tvcD = (g_cobra.phase != 0) ? 0.0
                    : SIGN_TVC_PITCH * clamp(g_stickRoll,  -1.0, 1.0) * TVC_DIFF_ROLL;
        drawargs[9020].f = (float)clamp(tvcY,         -1.0, 1.0);  // 右发左右
        drawargs[9021].f = (float)clamp(tvcP - tvcD,  -1.0, 1.0);  // 右发上下
        drawargs[9022].f = (float)clamp(tvcP + tvcD,  -1.0, 1.0);  // 左发上下
        drawargs[9023].f = (float)clamp(tvcY,         -1.0, 1.0);  // 左发左右
    }
    // ---- ★ 2026-09-17 修“加力生效但无尾焰特效”: 标准保留参数 28/29 = 左/右加力尾焰 ----
    // EFM 机型 DCS 不代管发动机外部动画: 模型里绑定在 arg 28(左发)/29(右发)上的
    // 加力火焰特效控制器, 必须由本 DLL 每帧写值才会点燃 —— 旧版只写了喷口 89/90,
    // 尾焰这条链路从头到尾没人写, 所以“推力有加力、外面看不到火”。
    //   0.0      = 军推及以下: 无焰
    //   0.0..1.0 = 加力段(N1 1.0..2.0): 从刚点着到全长全亮(N1 带 ~1.2s 惯性, 自然渐变)
    //   未点火(停车/熄火/灭火/风车)一律 0, 防止误点亮。
    if (size > 29)
    {
        for (int i = 0; i < 2; i++)              // i=0 -> arg28(左发), i=1 -> arg29(右发)
        {
            double v = 0.0;
            if (g_engLit[i] && g_n1[i] > 1.0)
                v = clamp(g_n1[i] - 1.0, 0.0, 1.0);
            drawargs[28 + i].f = (float)v;
        }
    }
    // ---- 尾喷口收敛片动画 (EDM arg 89=?, 90=? 用户提供号) ----
    // 由各发 N1 驱动, 平滑无跳变(g_n1 本身已带 ~1.2~2.2s 惯性):
    //   0.0      = 停车: 收敛片完全合拢
    //   0.28     = 慢车(N1≈0.2): 略张开
    //   0.12     = 军推(N1=1.0): 喉道最窄(收敛片近合拢)
    //   1.0      = 最大加力(N1=2.0): 喷管全开
    // 若游戏里方向相反(加油门反而合拢), 调换下方三个锚点值即可, 不用改结构。
    if (size > 89)
    {
        for (int i = 0; i < 2; i++)              // i=0 -> arg89(左发), i=1 -> arg90(右发)
        {
            double n1 = g_n1[i];
            double v;
            if (n1 < 0.08) v = 0.0;              // 停车: 合拢
            else if (n1 < 1.0)                   // 慢车->军推: 收敛片收窄
            {
                double t = clamp((n1 - 0.2) / 0.8, 0.0, 1.0);
                v = 0.28 - 0.16 * t;             // 0.28(慢车) -> 0.12(军推)
                if (n1 < 0.2) v *= (n1 - 0.08) / 0.12;   // 冷启动从全关平滑张开
            }
            else                                 // 加力: 喷管张开
            {
                double t = clamp((n1 - 1.0) / 1.0, 0.0, 1.0);
                v = 0.12 + 0.88 * t;             // 0.12(切入加力) -> 1.0(全开)
            }
            if (89 + i < (int)size) drawargs[89 + i].f = (float)v;
        }
    }

    // (2026-10-04 告警参数已迁座舱通道 1900~1904, 此处不再写外部 arg,
    //  9020~9023 全权归还上方 TVC 矢量喷口动画——曾在此覆写导致喷口卡死)
}

J35EFM_API void ed_fm_configure(const char *cfg_path)
{
    (void)cfg_path;
    j35Dbg("CONFIGURE cfg=%s", cfg_path ? cfg_path : "(null)");
}

J35EFM_API double ed_fm_get_param(unsigned index)
{
    // ---- APU 块 [0..99] ----
    if (index < 100) return 0.0;

    // ---- 引擎1块 [100..199] 引擎2块 [200..299]，各偏移 100 ----
    if (index >= 100 && index < 300)
    {
        int eng   = (index<200)?0:1;
        int local = (int)index - (eng==0?100:200);
        double n1 = g_n1[eng];
        double v;
        switch (local)
        {
        case 0:  v = n1*8500.0;  break;                 // RPM(风扇)
        case 1:  v = n1;         break;                 // RELATED_RPM (冷舱未起动 = 0)
        // ★★ 2026-09-13 修"点击起动没反应": 以前 CORE(核心机) 那几组参数一律返回 0,
        //    而 DCS 座舱的 base data(getEngineLeftRPM) 与发动机音效是拿核心机参数驱动的
        //    (对照 A-4E-C / acEFM: 同一条转速同时喂给 RPM 与 CORE_RPM)。
        //    结果: DLL 里 n1 明明已经 0.20, 座舱读回来还是 0 -> 按钮不跳、启动没声音。
        case 2:  v = n1*8500.0;  break;                 // CORE_RPM
        case 3:  v = n1;         break;                 // CORE_RELATED_RPM
        // ★ 未点火(起动机带转/停车)时推力、EGT、燃烧室都按"没着火"给
        case 4:  v = g_engLit[eng] ? engineThrustTotal()*(n1/(g_n1[0]+g_n1[1]+1e-9)) : 0.0;  break;  // THRUST N
        case 5:  v = g_engLit[eng] ? relThrustDry(n1) : 0.0;  break;   // RELATED_THRUST (军推=1, 全加力≈1.75)
        case 6:  v = g_engLit[eng] ? engineThrustTotal()*(n1/(g_n1[0]+g_n1[1]+1e-9)) : 0.0;  break;  // CORE_THRUST N
        case 7:  v = g_engLit[eng] ? relThrustDry(n1) : 0.0;  break;   // CORE_RELATED_THRUST (同上, 马赫环判据)
        case 12: v = g_engLit[eng] ? (25.0+900.0*n1) : (25.0+80.0*n1);  break;  // TEMPERATURE C
        case 13: v = 3.5e6;      break;                 // OIL_PRESSURE Pa
        case 14: v = engineFuelFlowTotal()*0.5; break;  // FUEL_FLOW kg/s(单发, 未点火=0)
        case 15: v = g_engLit[eng] ? ((n1>1.0)?0.999:0.6) : 0.0;        break;   // COMBUSTION
        // 起动机带转期间给带转进度(0..1), 其余时刻 0
        case 17: v = (g_engState[eng]==ENG_CRANK) ? clamp(g_n1[eng]/kN1Crank, 0.0, 1.0) : 0.0; break;  // STARTER_RELATED_RPM
        case 25: v = 1.0;        break;                 // OPERABILITY_FACTOR(新版头文件编号口径)
        case 26: v = 1.0;        break;                 // OPERABILITY_FACTOR(旧编号口径)/FAN_PHASE(常数相位, 无害)
        default: v = 0.0;        break;
        }
        // 诊断: 记录 DCS 问了哪些参数(每 250ms 汇总打到日志, 见 ed_fm_simulate 的 PARAM 行)
        if (local >= 0 && local < 40)
        {
            g_pqCnt[eng][local]++;
            g_pqVal[eng][local] = v;
        }
        return v;
    }

    // ---- 起落架参数块（3条腿，块长10）----
    const unsigned SUS0 = ED_FM_SUSPENSION_0_RELATIVE_BRAKE_MOMENT; // = 2000(ED头)
    if (index >= SUS0 && index < SUS0+3*10)
    {
        int leg = (int)(index - SUS0)/10;      // 0 前 1 左主 2 右主
        int sub = (int)(index - SUS0)%10;
        switch (sub)
        {
        case 0:  return (leg==0)?0.0:j35BrakeEff();      // RELATIVE_BRAKE_MOMENT(刹车阀关=0)
        case 1:  return g_gearState;                     // GEAR_POST_STATE
        case 2:  return (g_gearState<0.1)?1.0:0.0;       // UP_LOCK
        case 3:  return (g_gearState>0.9)?1.0:0.0;       // DOWN_LOCK
        case 4:  return (leg==0) ? g_nwsYaw : 0.0;       // WHEEL_YAW(★2026-09-19 前轮转向, 见 updateNWS)
        case 5:  // WHEEL_SELF_ATTITUDE(★2026-09-19 二次拟真, 见 updateNWS"混合模式")
                 //   前轮: NWS 接通=0(受脚蹬控制), 断开=1(自由定位随运动方向);
                 //   主轮: 恒 0(真实主轮固定于轮轴, 不随动)。
                 if (leg == 0) return g_nwsHold ? 0.0 : 1.0;
                 return 0.0;
        default: return 0.0;
        }
    }

    // ---- FC3 老座舱仪表对接块 [10000..] ----
    if (index >= ED_FM_FC3_RESERVED_SPACE && index < ED_FM_FC3_RESERVED_SPACE_END)
    {
        switch ((int)(index - ED_FM_FC3_RESERVED_SPACE))
        {
        case 1:  return g_gearTarget;                    // GEAR_HANDLE_POS
        case 2:  return g_flapTarget;                    // FLAPS_HANDLE_POS
        case 3:  return g_brakeTarget;                   // SPEED_BRAKE_HANDLE_POS
        case 4:  return g_stickPitch;                    // STICK_PITCH
        case 5:  return g_stickRoll;                     // STICK_ROLL
        case 6:  return g_stickYaw;                      // RUDDER_PEDALS
        // ★ 2026-09-17 修"油门满仍无加力"(当日二修):
        //   尾焰主通道是 ed_fm_set_draw_args 的 arg 28/29(本次已补上, 见那边注释);
        //   FC3 THROTTLE 这里保持 0..2 口径(1.0 = MIL, 2.0 = 最大加力), 供 DCS 侧
        //   AI/音效等内置系统按"油门>1 = 加力"判断。
        case 7:  return g_throttleAxis * 2.0;            // THROTTLE_LEFT
        case 8:  return g_throttleAxis * 2.0;            // THROTTLE_RIGHT
        case 14: return 1.0;                             // STICK_PITCH_LIMITER
        case 15: return 1.0;                             // STICK_ROLL_L_LIMITER
        case 16: return 1.0;                             // PEDAL_L_LIMITER
        case 17: return 1.0;                             // PEDAL_R_LIMITER
        case 19: return j35BrakeEff();                    // WHEEL_BRAKE_LEFT(刹车阀关=0)
        case 20: return j35BrakeEff();                    // WHEEL_BRAKE_RIGHT(刹车阀关=0)
        default: return 0.0;
        }
    }

    // ---- 通用系统 ----
    switch (index)
    {
    case ED_FM_OXYGEN_SUPPLY:                    return 78000.0;
    case ED_FM_ANTI_SKID_ENABLE:                 return 1.0;
    case ED_FM_FUEL_FUEL_TANK_GROUP_0_LEFT:
    case ED_FM_FUEL_FUEL_TANK_GROUP_0_RIGHT:     return g_fuel*0.5;
    case ED_FM_FUEL_INTERNAL_FUEL:               return g_fuel;
    case ED_FM_FUEL_TOTAL_FUEL:                  return g_fuel;
    case ED_FM_FUEL_LOW_SIGNAL:                  return (g_fuel<800.0)?1.0:0.0;
    case ED_FM_CAN_ACCEPT_FUEL_FROM_TANKER:      return (g_fuel<(kFuelMax-20.0))?1.0:0.0;
    default: break;
    }
    return 0.0;
}

J35EFM_API void ed_fm_cold_start(void)
{
    // 复位全部输入/操纵状态
    g_stickPitch=g_stickRoll=g_stickYaw=0.0;
    g_reqPitch=g_reqRoll=g_reqYaw=0.0;
    g_pitchAx=g_rollAx=g_yawAx=0.0;
    g_pitchAnalog=g_rollAnalog=g_yawAnalog=false;
    g_pitchKey=g_rollKey=g_yawKey=0;
    g_pitchTrim=g_rollTrim=0.0;    // 方向舵配平已删(2026-09-17)
    g_trimA=0.01;    g_rollAtt=0.0;                 // 俯仰配平基准归零(2026-09-18 重写)
    memset(g_lastTgl,0,sizeof(g_lastTgl));
    g_fuel = kFuelMax;
    // ★ 2026-09-13: 冷舱出生 = 两台都没起动 —— N1 必须真的是 0(不燃烧/不产推力/不耗油),
    //   等 311/312/309 起动命令才走状态机。
    //   (旧版这里把 g_engOn 置 true, 出生 1 秒后 N1 自己爬到慢车 0.21, 座舱侧
    //    "未起动"状态被吃掉 —— 这就是要修的地方。)
    for (int i=0;i<2;i++)
    {
        g_engState[i]    = ENG_OFF;
        g_engPhase[i]    = 0.0;
        g_engShutFrom[i] = 0.0;
        g_engLit[i]      = false;
        g_engOn[i]       = false;
    }
    g_n1[0]=g_n1[1]=0.0; g_n1cmd[0]=g_n1cmd[1]=0.0;
    g_gearState=g_gearTarget=1.0;
    g_flapState=g_flapTarget=0.0;   // j35FlReset() 也会置 0(任何出生方式襟翼收起)
    g_brakeOpen=g_brakeTarget=0.0;
    g_wheelBrake=0.0;
    g_nwsYaw=0.0;    // ★ 2026-09-19 前轮转向角清零
    g_nwsLock=false; // ★ 2026-09-22 前轮转向锁默认关(可转向)
    g_throttleAxis=0.0;
    j35AhReset();   // ★ 2026-09-17 定速巡航/锁定状态清零 + 删通道文件(防任务重开残留)
    j35SwReset();   // ★ 2026-09-17 系统开关联锁清零 + 删通道文件(同上)
    j35CpReset();   // ★ 2026-09-17 座舱盖动画通道清零 + 删文件(arg181 由本 DLL 写)
    j35MdcReset();  // ★ 2026-09-28 爆索通道清零 + 删文件(arg183 由本 DLL 写)
    j35FcsReset();  // ★ 2026-09-29 飞控模式重置 -> AUTO + 删文件
    g_cobra.phase = 0; g_cobra.timer = 0;  // ★ 2026-09-29 钟摆状态机复位
    j35HookReset(); // ★ 2026-09-18 尾勾状态清零 + 删 J35_HK_cmd.txt(防任务重开残留)
    j35AclReset();  // ★ 2026-09-18 着舰辅助状态清零 + 删 J35_ACL_cmd/out.txt(同上)
    // ★ 2026-09-22 NWS 锁文件清零 + 删 J35_NWS_cmd.txt(防任务重开残留)
    g_nwsLock = false; g_nwsInSeq = 0; g_nwsReadT = 0;
    { const char *dd = j35SavedGamesDir(); if (dd) { char pp[600]; snprintf(pp,sizeof(pp),"%sJ35_NWS_cmd.txt",dd); remove(pp); } }
    j35FlReset();   // ★ 2026-09-21 襟翼三档归零(收) + 删 J35_FL_cmd.txt(同上)
    j35DamageReset(); // ★ 2026-09-22 战损失控状态复位(任何出生方式都是完好新机)
    j35FireReset();   // ★ 2026-10-06 火警锁存复位: g_fire 是进程级 static, 上次坠机起火后
                      //   重开任务/重新出生不会自动清, 导致新机一出生 arg1900/1901=1 双发火警
}

J35EFM_API void ed_fm_hot_start(void)
{
    ed_fm_cold_start();
    // 地面热启动 = 两台已在慢车运转(座舱口径: 地面已起动 = 20%)
    for (int i=0;i<2;i++)
    {
        g_engState[i] = ENG_RUNNING;
        g_engPhase[i] = 0.0;
        g_engLit[i]   = true;
        g_engOn[i]    = true;
    }
    // 油门放慢车位(axis 0), 起飞推力由玩家自己推进 —— 预设在 0.45 会出生即冲出去。
    g_n1[0]=g_n1[1]=0.22;              // 慢车转速(指令 0.20)
    g_throttleAxis = 0.0;
    g_fuel = kFuelMax;
}

J35EFM_API void ed_fm_hot_start_in_air(void)
{
    ed_fm_cold_start();
    // 空中出生 = 两台已运转(口径: 空中已起动)
    for (int i=0;i<2;i++)
    {
        g_engState[i] = ENG_RUNNING;
        g_engPhase[i] = 0.0;
        g_engLit[i]   = true;
        g_engOn[i]    = true;
    }
    g_n1[0]=g_n1[1]=0.75;
    g_throttleAxis = 0.55;
    g_fuel = kFuelMax;
    g_gearState = g_gearTarget = 0.0;      // 空中出生: 起落架直接收起
}

// 逐部件式接口（不启用，返回 false 表示不提供）
J35EFM_API bool ed_fm_add_local_force_component(double &x,double &y,double &z,
                                                double &pos_x,double &pos_y,double &pos_z)
{
    x=0;y=0;z=0;pos_x=g_cg.x;pos_y=g_cg.y;pos_z=g_cg.z;
    return false;
}
J35EFM_API bool ed_fm_add_global_force_component(double &x,double &y,double &z,
                                                 double &pos_x,double &pos_y,double &pos_z)
{
    x=0;y=0;z=0;pos_x=g_cg.x;pos_y=g_cg.y;pos_z=g_cg.z;
    return false;
}
J35EFM_API bool ed_fm_add_local_moment_component(double &x,double &y,double &z)
{
    x=0;y=0;z=0;
    return false;
}
J35EFM_API bool ed_fm_add_global_moment_component(double &x,double &y,double &z)
{
    x=0;y=0;z=0;
    return false;
}

// =============================================================================
//  LERX 边条涡流云  (ed_fm_LERX_vortex_update)
// -----------------------------------------------------------------------------
//  本函数由 DCS 逐帧调用(每侧一个 idx)，实时给出"边条/机翼前缘涡流"样条，
//  驱动引擎的 LERX VORTEX 特效 —— 即高攻角时沿翼面向后拖的螺旋涡流云。
//
//  约定（与 wHumanCustomPhysicsAPI.h 注释一致）:
//    idx 0 = 左翼, idx 1 = 右翼; 本机只占这两个槽位，idx>=2 返回 false。
//    槽位内返回 null spline / 0 点 -> 通知 DCS detach 掉已存在的涡流。
//
//  坐标与 config.lua 起落架同一套"模型坐标系":
//    +X 前, +Y 上, +Z 右;  歼-35 机长约 21.9m，CG=(-1.55,-0.65,0)。
//  样条点沿 LEX/进气道边条向前缘外翼扫掠，涡核走在机翼上表面之上方
//  (苏-30 观感: 自进气道上沿起，贴着上翼面向后、略向内收，而不是在襟翼下缘)。
//
//  下面 kLerxAlpha* 与 kLerxSpline 均可按试飞观感微调。
#define VLERX_ALPHA_ON    10.0    // 攻角(deg)起现
#define VLERX_ALPHA_PEAK  22.0    // 到峰值亮度
#define VLERX_ALPHA_HOLD  34.0    // 保持全亮
#define VLERX_ALPHA_OFF   45.0    // 完全熄灭
// 右翼涡轴样条 {x, y, z, 半径m}; 左翼自动取 z 镜像。
// 直条: 沿机身两侧(X 向后)笔直延伸, 起点在 x≈0 附近。
// ★ 2026-09-22 用户口径: 涡轴整体上移 1.0m, y 0.30 -> 1.30(原位置偏低, 涡流云埋进翼面/机身)。
// ★ 2026-09-22 用户口径: 座舱盖左右各一条 —— z 0 -> ±0.60(座舱盖半宽),
//    原来左右两槽都在中轴线(z=0)上叠成一条; idx1 右翼 +0.60, idx0 左翼 -0.60。
// ★ 2026-09-22 三次调整(用户口径): 涡流云半径放大 2 倍; 整条涡轴前移 1.0m
//    (+X 为前, x 全部 +1.0); 左右再向外分开 |z| 0.60 -> 1.00(左翼更左/右翼更右)。
// ★ 2026-09-22 四次调整(用户口径): 再前移 1.5m(x 再 +1.5, 累计较初值 +2.5);
//    左右外分 |z| 1.00 -> 3.20(±3.2m, 到达翼面上方)。
// ★ 2026-09-22 五次调整(用户口径): 外分收回 1.2m, |z| 3.20 -> 2.00(±2.0m)。
// ★ 2026-09-22 六次调整(用户口径): 涡轴下移 1m, y 1.30 -> 0.30(回到初始高度)。
// ★ 2026-09-22 七次调整(用户口径): 向机身内收一点, |z| 2.00 -> 1.50(±1.5m)。
// ★ 2026-09-22 八次调整(用户口径): 涡流云加长 —— 尾端追加第 6 点 x=-1.95,
//    总长 4.2m -> 5.25m(前段形状不变; 缓冲上限 8 点)。
// ★ 2026-09-22 九次调整(用户口径): 往外一点 |z| 1.50 -> 1.80; 往上一点 y 0.30 -> 0.60;
//    再长一点 —— 尾端追加第 7 点 x=-3.00, 总长 5.25 -> 6.30m。
// ★ 2026-09-22 十次调整(用户口径): 总长加到 8m —— 起点 3.30, 尾点需 -4.70;
//    追加第 8/9 点 -3.85/-4.70(同步把样条缓冲 8 -> 16)。
static const float kLerxSpline[][4] = {
    {  3.20f,  0.55f,  0.90f, 0.35f },   // 边条根部(座舱侧)
    {  2.40f,  0.62f,  1.05f, 0.50f },
    {  1.60f,  0.68f,  1.15f, 0.65f },
    {  0.80f,  0.74f,  1.35f, 0.85f },
    {  0.00f,  0.78f,  1.60f, 1.05f },
    { -0.80f,  0.80f,  1.90f, 1.25f },
    { -1.60f,  0.80f,  2.20f, 1.45f },
    { -2.40f,  0.78f,  2.55f, 1.55f },   // 主翼上方最强
    { -3.20f,  0.72f,  2.80f, 1.55f },
    { -4.00f,  0.64f,  3.05f, 1.45f },
    { -4.80f,  0.56f,  3.25f, 1.30f },
    { -5.60f,  0.48f,  3.40f, 1.15f },
    { -6.40f,  0.42f,  3.50f, 1.00f },
    { -7.20f,  0.36f,  3.60f, 0.85f },
    { -8.00f,  0.32f,  3.65f, 0.68f },
    { -8.80f,  0.30f,  3.65f, 0.50f },   // 尾后拖曳渐散
};
static const int kLerxN = (int)(sizeof(kLerxSpline)/sizeof(kLerxSpline[0]));
// ★ 2026-09-25 十一次调整(用户口径, 参考 F/A-18 照片): 细直管 → 粗壮的双流升力涡,
//    左右各一条明显分开(±0.7m 边条根部起, 沿翼面展开到 ±2.8m), 主翼上方最粗 1.55m,
//    尾后拖曳渐散。不可太靠中线/太粗, 否则两条融成一团盖住机背(十二次调整教训)。
// 各点局部透过率：前弱、中段最强、尾部渐隐（version=1 时与总透明度相乘）
static const float kLerxFade[kLerxN] = {0.50f, 0.65f, 0.80f, 0.90f, 1.00f, 1.00f, 1.00f, 1.00f, 0.92f, 0.82f, 0.68f, 0.52f, 0.36f, 0.22f, 0.12f, 0.05f};

// 涡流样条点的持久缓冲（DCS 每帧拷贝，需保持稳定地址）: 0/1=左右翼面涡, 2/3/4=音爆云三环
static LERX_vortex_spline_point g_lerxPts[5][16];

J35EFM_API bool ed_fm_LERX_vortex_update(unsigned idx, LERX_vortex & out)
{
    if (idx > 4) return false;          // 0/1=左右翼面涡; 2/3/4=音爆云环(见下)

    // 当前空速 / 相对气流（与 computeForces 同口径；攻角为机体系俯仰面内）
    double V = sqrt(g_velBody.x*g_velBody.x+g_velBody.y*g_velBody.y+g_velBody.z*g_velBody.z);
    double ax = g_velBody.x - g_windBody.x;
    double ay = g_velBody.y - g_windBody.y;
    double az = g_velBody.z - g_windBody.z;
    double alphaDeg = fabs(atan2(-ay, smax(ax, 0.5))) * (180.0/kPi);
    double mach = V / smax(g_sos, 1.0);

    // ---- 槽 2/3: 音爆云(跨音速 Prandtl-Glauert 凝结云) ----
    // J-35.EDM 无内置云锥网格(二进制全文检索无 vapor/cone/boom), 改用 LERX 涡管 API
    // 画两条垂直机身纵轴的闭合圆环样条近似锥台云: 前环大、后环小。
    // 马赫窗口 0.94~1.06, M=1.0 最强; V<250 m/s 强制熄灭(环带上限~M0.74, 无意义)。
    if (idx >= 2)
    {
        // 三环重叠成连续云锥(管径>环间距, 消除环间空隙); 前大后小
        static const float kRingX[3] = { -2.2f, -3.4f, -4.6f };   // 环心 x(机体系, 负=向尾)
        static const float kRingR[3] = { 2.6f, 2.2f, 1.7f };      // 环半径 m
        static const float kRingT[3] = { 1.2f, 1.1f, 1.0f };      // 涡管半径 m
        static const float kRingO[3] = { 1.0f, 0.85f, 0.7f };     // 各环透明度系数
        const int ri = (int)idx - 2;
        double w  = 1.0 - fabs(mach - 1.00) / 0.06;
        double op = (w > 0.0 && V > 250.0) ? 0.85 * w * kRingO[ri] : 0.0;
        if (op < 0.02)
        {
            out.opacity = 0.0f;
            out.explosion_start = 1.0f;
            out.spline = nullptr;
            out.spline_points_count = 0;
            out.spline_point_size_in_bytes = LERX_vortex_spline_point_size;
            out.version = 1;
            return true;
        }
        out.opacity = (float)op;
        out.explosion_start = 1.0e9f;   // 云环无破裂点(值远大于环长=全程不爆发)
        out.spline = g_lerxPts[idx];
        out.spline_points_count = (unsigned)kLerxN;
        out.spline_point_size_in_bytes = LERX_vortex_spline_point_size;
        out.version = 1;
        for (int i = 0; i < kLerxN; i++)
        {
            double th = 2.0 * kPi * i / (kLerxN - 1);   // 末点=起点, 闭合圆环
            LERX_vortex_spline_point &p = g_lerxPts[idx][i];
            p.pos[0] = kRingX[ri];
            p.pos[1] = (float)(kRingR[ri] * sin(th));
            p.pos[2] = (float)(kRingR[ri] * cos(th));
            p.vel[0] = (float)ax;
            p.vel[1] = (float)ay;
            p.vel[2] = (float)az;
            p.radius  = kRingT[ri];
            p.opacity = 1.0f;
        }
        return true;
    }

    // 熄灭分支：低速 / 低攻角 / 进入跨超音速时关闭（null spline 让 DCS 移除旧涡流）
    bool on = (V > 40.0) && (alphaDeg > VLERX_ALPHA_ON) && (mach < 1.02);
    if (!on)
    {
        out.opacity = 0.0f;
        out.explosion_start = 1.0f;
        out.spline = nullptr;
        out.spline_points_count = 0;
        out.spline_point_size_in_bytes = LERX_vortex_spline_point_size;
        out.version = 1;
        return true;
    }

    // 攻角活动度：起现 -> 峰值 -> 保持 -> 熄灭
    double act;
    if (alphaDeg <= VLERX_ALPHA_HOLD)
        act = clamp((alphaDeg-VLERX_ALPHA_ON)/(VLERX_ALPHA_PEAK-VLERX_ALPHA_ON), 0.0, 1.0);
    else
        act = 1.0 - clamp((alphaDeg-VLERX_ALPHA_HOLD)/(VLERX_ALPHA_OFF-VLERX_ALPHA_HOLD), 0.0, 1.0);
    double spdF  = clamp((V-40.0)/90.0, 0.0, 1.0);            // 70~130 m/s 内渐亮
    double machF = (mach<0.80) ? 1.0 : clamp((1.02-mach)/0.22, 0.0, 1.0);
    double opacity = act*act*spdF*machF;
    if (opacity < 0.02)
    {
        out.opacity = 0.0f;
        out.explosion_start = 1.0f;
        out.spline = nullptr;
        out.spline_points_count = 0;
        out.spline_point_size_in_bytes = LERX_vortex_spline_point_size;
        out.version = 1;
        return true;
    }

// ★ 2026-09-25 十二次调整(用户口径): 关掉涡核破裂 —— 粗管在破裂点炸开像引爆云,
//    云毯/双流形态下禁用(设极大值=全程不爆发)。
    double burst = 1.0e9;

    out.opacity = (float)clamp(opacity, 0.0, 1.0);
    out.explosion_start = (float)burst;
    out.spline = g_lerxPts[idx];
    out.spline_points_count = (unsigned)kLerxN;
    out.spline_point_size_in_bytes = LERX_vortex_spline_point_size;
    out.version = 1;

    float zSign = (idx==1) ? 1.0f : -1.0f;   // 1=右翼(+Z), 0=左翼(-Z)
    for (int i = 0; i < kLerxN; i++)
    {
        LERX_vortex_spline_point &p = g_lerxPts[idx][i];
        p.pos[0] = kLerxSpline[i][0];
        p.pos[1] = kLerxSpline[i][1];
        p.pos[2] = kLerxSpline[i][2]*zSign;
        // 速度 = 该点附近的相对气流速度（机体系），供引擎做涡流尾迹推移
        p.vel[0] = (float)ax;
        p.vel[1] = (float)ay;
        p.vel[2] = (float)az;
        p.radius = kLerxSpline[i][3];
        p.opacity = kLerxFade[i];
    }
    return true;
}
