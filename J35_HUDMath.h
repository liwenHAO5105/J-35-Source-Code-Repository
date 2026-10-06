// =============================================================================
//  J35_HUDMath.h  —  HUD 锁定框角度换算: 世界坐标 -> 机体坐标系 XYZ -> HUD 玻璃坐标
// -----------------------------------------------------------------------------
//  用途: HUD 锁定框(TD框/TGP框)必须始终钉在【机体坐标系】里——
//        飞机滚转/俯仰/偏航时, 锁定框相对机头的位置才不变(准直 HUD 原理)。
//        本模块只做纯数学, 不依赖任何 DCS 头文件, EFM/PODCAM 任何位置都能用。
//
//  坐标系约定(改动前务必确认, 与 DCS 引擎一致):
//    世界系(DCS Position3 / LoGetWorldObjects 的 Position / 本机位置):
//        x = 北(经线方向), y = 高度(天), z = 东
//        航向 heading: 北=0, 东=90, 顺时针(度)
//    机体系(本模块定义, 右手系):
//        X = 机头(前), Y = 座舱盖方向(上), Z = 右机翼(右)
//    姿态角:
//        pitch: 抬头为正(度), roll: 右滚(右翼下沉)为正(度)
//
//  HUD 输出:
//    az_deg: 方位角, 目标在准星右侧为正(度)
//    el_deg: 高低角, 目标在准星上方为正(度)
//    x_m/y_m: 准直投影到玻璃上的米数 = 玻璃距 * tan(角度)(gnomonic 精确投影,
//             直接用 by/bx、bz/bx 比值, 不是小角度近似)
//    in_front: 目标是否在机头前半球(bx>0); 后半球的点 x/y 无意义不画框
//
//  逆变换(吊舱随动: HUD 框选点 -> 世界视线方向):
//    HudAzElToWorldDir()
//
//  构建: 与 J35_EFM.cpp 一起编进同一个 DLL:
//    $env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH          # 必须先加 PATH!
//    g++ -O2 -shared -static-libgcc -static-libstdc++ ^
//        -o J35Cam.dll J35_EFM.cpp J35_HUDMath.cpp ^
//        -I"D:\SteamLibrary\steamapps\common\DCSWorld\API\include"
//  独立自测(不进 DCS, 直接验证数学):
//    g++ -O2 -DJ35_HUDMATH_TEST -o hudmath_test.exe J35_HUDMath.cpp ; ./hudmath_test.exe
// =============================================================================
#pragma once
#ifndef J35_HUDMATH_H
#define J35_HUDMATH_H

namespace J35HM {

struct Vec3 { double x, y, z; };

// 本机姿态(度): heading 北0东90顺时针, pitch 抬头正, roll 右滚正
struct Attitude { double heading_deg, pitch_deg, roll_deg; };

// HUD 上一个点的完整描述
struct HudPoint {
    double az_deg;    // 方位角(右正)
    double el_deg;    // 高低角(上正)
    double x_m;       // 玻璃投影 x(右正, 米) = dist * bz/bx
    double y_m;       // 玻璃投影 y(上正, 米) = dist * by/bx
    bool   in_front;  // 是否前半球(后半球不画框)
};

// ---- 方向余弦级换算(输入向量长度任意, 内部归一) ----

// 机体系方向 -> 世界系方向
Vec3 BodyToWorldDir(Vec3 b, const Attitude& att);
// 世界系方向 -> 机体系方向(HUD 主用: 视线向量转进机体XYZ)
Vec3 WorldToBodyDir(Vec3 w, const Attitude& att);

// ---- HUD 投影 ----

// 机体方向 -> (az, el) 角度(度)
void BodyDirToAzEl(Vec3 b, double& az_deg, double& el_deg);

// 世界方向 -> HUD 点; glass_dist_m = 组合玻璃等效投影距离(米, J-35 取 0.75 上下微调)
HudPoint WorldDirToHud(Vec3 w, const Attitude& att, double glass_dist_m);

// 世界目标点 -> HUD 点(全流程: 目标位置 - 本机位置 -> 视线 -> 机体系 -> 投影)
// target_pos / own_pos 用 DCS 世界坐标(x北 y高 z东)
HudPoint WorldPointToHud(Vec3 target_pos, Vec3 own_pos, const Attitude& att, double glass_dist_m);

// ---- 逆变换(吊舱随动) ----

// HUD (az, el)(度) -> 世界系单位视线方向(准星正前方 az=el=0 -> 机头方向)
Vec3 HudAzElToWorldDir(double az_deg, double el_deg, const Attitude& att);

} // namespace J35HM

#endif // J35_HUDMATH_H
