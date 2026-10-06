// =============================================================================
//  J35_HUDMath.cpp  —  实现见 J35_HUDMath.h 的坐标系约定
// -----------------------------------------------------------------------------
//  旋转链(右手系, 世界 -> 机体):
//      B = Rx(-roll) * Rz(-pitch) * Ry(heading) * W
//  其中世界轴序 (x北, y天, z东):
//      Ry(h)   = 绕世界 y(天) 轴, 把航向从"北"转到 heading(北0东90)
//      Rz(p)   = 绕机体 z(右翼) 轴, 抬头为正
//      Rx(r)   = 绕机体 x(机头) 轴, 右滚为正
//  机体 -> 世界为逆链:
//      W = Ry(-h) * Rz(p) * Rx(r) * B
// =============================================================================
#include "J35_HUDMath.h"
#include <cmath>

namespace J35HM {

static const double DEG2RAD = 0.017453292519943295;   // pi/180
static const double RAD2DEG = 57.29577951308232;      // 180/pi

static double vlen(Vec3 v) { return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z); }

static Vec3 vnorm(Vec3 v) {
    double l = vlen(v);
    if (l < 1e-12) { Vec3 z = {0,0,0}; return z; }
    Vec3 r = {v.x/l, v.y/l, v.z/l};
    return r;
}

// 世界方向 -> 机体方向
Vec3 WorldToBodyDir(Vec3 w, const Attitude& att) {
    w = vnorm(w);
    const double h  = att.heading_deg * DEG2RAD;
    const double p  = att.pitch_deg   * DEG2RAD;
    const double r  = att.roll_deg    * DEG2RAD;

    // 1) 去航向: 绕世界 y 轴 +h
    //    Ry(a): x' = x*cos a + z*sin a ; z' = -x*sin a + z*cos a
    double ch = std::cos(h), sh = std::sin(h);
    double x1 =  w.x*ch + w.z*sh;
    double y1 =  w.y;
    double z1 = -w.x*sh + w.z*ch;

    // 2) 去俯仰: 绕 z 轴 -p
    //    Rz(a): x' = x*cos a - y*sin a ; y' = x*sin a + y*cos a
    double cp = std::cos(p), sp = std::sin(p);
    double x2 =  x1*cp + y1*sp;   // Rz(-p): sin(-p)=-sp 代回得此行
    double y2 = -x1*sp + y1*cp;
    double z2 =  z1;

    // 3) 去滚转: 绕 x 轴 -r
    //    Rx(a): y' = y*cos a - z*sin a ; z' = y*sin a + z*cos a
    double cr = std::cos(r), sr = std::sin(r);
    double x3 =  x2;
    double y3 =  y2*cr + z2*sr;   // Rx(-r)
    double z3 = -y2*sr + z2*cr;

    Vec3 b = {x3, y3, z3};
    return b;
}

// 机体方向 -> 世界方向(逆链)
Vec3 BodyToWorldDir(Vec3 b, const Attitude& att) {
    b = vnorm(b);
    const double h  = att.heading_deg * DEG2RAD;
    const double p  = att.pitch_deg   * DEG2RAD;
    const double r  = att.roll_deg    * DEG2RAD;

    // 1) 加滚转: 绕 x 轴 +r
    double cr = std::cos(r), sr = std::sin(r);
    double x1 =  b.x;
    double y1 =  b.y*cr - b.z*sr;
    double z1 =  b.y*sr + b.z*cr;

    // 2) 加俯仰: 绕 z 轴 +p
    double cp = std::cos(p), sp = std::sin(p);
    double x2 =  x1*cp - y1*sp;
    double y2 =  x1*sp + y1*cp;
    double z2 =  z1;

    // 3) 加航向: 绕世界 y 轴 -h
    double ch = std::cos(h), sh = std::sin(h);
    double x3 =  x2*ch - z2*sh;
    double y3 =  y2;
    double z3 =  x2*sh + z2*ch;

    Vec3 w = {x3, y3, z3};
    return w;
}

void BodyDirToAzEl(Vec3 b, double& az_deg, double& el_deg) {
    b = vnorm(b);
    az_deg = std::atan2(b.z, b.x) * RAD2DEG;                       // 右正
    el_deg = std::atan2(b.y, std::sqrt(b.x*b.x + b.z*b.z)) * RAD2DEG; // 上正
}

HudPoint WorldDirToHud(Vec3 w, const Attitude& att, double glass_dist_m) {
    Vec3 b = WorldToBodyDir(w, att);
    HudPoint hp;
    BodyDirToAzEl(b, hp.az_deg, hp.el_deg);
    hp.in_front = (b.x > 1e-6);
    if (hp.in_front) {
        hp.x_m = glass_dist_m * (b.z / b.x);   // gnomonic: dist * tan(az) 精确值
        hp.y_m = glass_dist_m * (b.y / b.x);   // gnomonic: dist * tan(el视线斜率)
    } else {
        hp.x_m = 0.0;
        hp.y_m = 0.0;
    }
    return hp;
}

HudPoint WorldPointToHud(Vec3 target_pos, Vec3 own_pos, const Attitude& att, double glass_dist_m) {
    Vec3 los = { target_pos.x - own_pos.x,
                 target_pos.y - own_pos.y,
                 target_pos.z - own_pos.z };
    return WorldDirToHud(los, att, glass_dist_m);
}

Vec3 HudAzElToWorldDir(double az_deg, double el_deg, const Attitude& att) {
    // 先在机体系里由 (az, el) 还原单位向量, 再转世界
    const double az = az_deg * DEG2RAD;
    const double el = el_deg * DEG2RAD;
    Vec3 b;
    b.x = std::cos(el) * std::cos(az);   // 前
    b.y = std::sin(el);                  // 上
    b.z = std::cos(el) * std::sin(az);   // 右
    return BodyToWorldDir(b, att);
}

} // namespace J35HM

// =============================================================================
//  自测: g++ -O2 -DJ35_HUDMATH_TEST -o hudmath_test.exe J35_HUDMath.cpp
// =============================================================================
#ifdef J35_HUDMATH_TEST
#include <cstdio>
using namespace J35HM;

static int g_fail = 0;
static void chk(const char* name, double got, double want, double tol = 1e-6) {
    bool ok = std::fabs(got - want) < tol;
    std::printf("%-42s got=%12.6f want=%12.6f  %s\n", name, got, want, ok ? "OK" : "*** FAIL ***");
    if (!ok) ++g_fail;
}

int main() {
    Attitude lvl = {0, 0, 0};            // 平飞, 头朝北
    Vec3 own = {0, 3000, 0};

    // 1) 正前方同高目标 -> az=0 el=0
    { Vec3 t = {10000, 3000, 0};
      HudPoint hp = WorldPointToHud(t, own, lvl, 0.75);
      chk("ahead az", hp.az_deg, 0); chk("ahead el", hp.el_deg, 0);
      chk("ahead x_m", hp.x_m, 0);   chk("ahead y_m", hp.y_m, 0); }

    // 2) 正右方目标 -> az=+90, 前半球边界(bx=0 算后半球, 这里用 89.999 度方向)
    { Vec3 t = {10000, 3000, 1000};    // 北10km 东1km
      HudPoint hp = WorldPointToHud(t, own, lvl, 0.75);
      chk("right-ish az", hp.az_deg, std::atan2(1000.0, 10000.0) * RAD2DEG, 1e-4); }

    // 3) 正上方 -> el=+90, 后半球(bx=0)不画框
    { Vec3 t = {0, 13000, 0};
      HudPoint hp = WorldPointToHud(t, own, lvl, 0.75);
      chk("overhead el", hp.el_deg, 90, 1e-4);
      chk("overhead in_front", hp.in_front ? 1 : 0, 0); }

    // 4) 航向90(头朝东): 世界正东目标 -> az=0
    { Attitude a = {90, 0, 0};
      Vec3 t = {0, 3000, 10000};
      HudPoint hp = WorldPointToHud(t, own, a, 0.75);
      chk("hdg90 east az", hp.az_deg, 0, 1e-6); }

    // 5) 抬头10度: 世界水平前方目标 -> el=-10(目标在准星下方)
    { Attitude a = {0, 10, 0};
      Vec3 t = {10000, 3000, 0};
      HudPoint hp = WorldPointToHud(t, own, a, 0.75);
      chk("pitch10 horizon el", hp.el_deg, -10, 1e-6); }

    // 6) 右滚90度: 世界天顶方向 -> az=-90(滚转后天顶在机头左方)
    { Attitude a = {0, 0, 90};
      Vec3 w = {0, 1, 0};
      HudPoint hp = WorldDirToHud(w, a, 0.75);
      chk("roll90 zenith az", hp.az_deg, -90, 1e-6); }

    // 7) 逆变换往返: (az=25, el=-8) -> 世界 -> 机体 -> 角度 应还原
    { Attitude a = {123, -4, 60};
      Vec3 w = HudAzElToWorldDir(25, -8, a);
      Vec3 b = WorldToBodyDir(w, a);
      double az, el; BodyDirToAzEl(b, az, el);
      chk("roundtrip az", az, 25, 1e-9); chk("roundtrip el", el, -8, 1e-9); }

    // 8) 玻璃投影: az=30 度, dist=0.75 -> x = 0.75*tan(30)
    { Attitude a = {0, 0, 0};
      Vec3 w = HudAzElToWorldDir(30, 0, a);
      HudPoint hp = WorldDirToHud(w, a, 0.75);
      chk("glass x at az30", hp.x_m, 0.75 * std::tan(30 * DEG2RAD), 1e-9); }

    std::printf(g_fail ? "\n*** %d 项失败 ***\n" : "\n全部通过\n", g_fail);
    return g_fail ? 1 : 0;
}
#endif // J35_HUDMATH_TEST
