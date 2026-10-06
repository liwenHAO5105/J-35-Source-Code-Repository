// =============================================================================
//  J35_CRUISE.h  —  巡航导弹目标坐标运行时注入 (WeaponBlocks.dll 逆向原型)
// -----------------------------------------------------------------------------
//  原理 (2026-09-30 逆向 WeaponBlocks.dll 结论):
//    DCS 巡航导弹飞行中由 wCruiseAutopilot 块制导, 它注册了一个
//    Vector3d 输入端口 "cruise_ap_target_pos", 引擎内部装订目标就走它。
//    运行时改目标 = 对在飞的 wCruiseAutopilot 实例调成员函数 add_pos(Vector3d)。
//
//  对象定位 = 虚表堆扫描 (所有偏移相对 WeaponBlocks.dll 基址, DCS 3.0.0 内部版本):
//      wCruiseAutopilot::vftable = base + 0x3a9930
//      add_pos(Vector3d)         = base + 0x162d20
//      simulate (vft[3], 校验用) = base + 0x168730
//    VirtualQuery 遍历已提交内存 -> 找 [qword == vftVA] 候选 ->
//    校验 vft[3] == simulateVA -> 命中即活的巡航自驾实例。
//
//  通道 (与 MDC/FCS 同款文件通道):
//    座舱/Hook 写 <SavedGames>\J35DATA\J35_TGT_cmd.txt:
//        J35TGT1 <seq> <x> <y> <z>      (DCS 世界坐标, 米)
//    seq 变化才执行一次注入。坐标换算(经纬度->世界)由 Lua 侧 coord.LLtoLO 完成。
//
//  构建: 与现有文件一起编进 J35Cam.dll:
//    g++ -O2 -shared -static-libgcc -static-libstdc++ ^
//        -o J35Cam.dll J35_EFM.cpp J35_HUDMath.cpp J35_MDC.cpp J35_FCS.cpp J35_CRUISE.cpp ^
//        -I"D:\SteamLibrary\steamapps\common\DCSWorld\API\include"
//
//  ★ 注意: 偏移只对当前 DCS 版本的 WeaponBlocks.dll 有效, DCS 大版本更新后需重扫。
// =============================================================================
#pragma once
#ifndef J35_CRUISE_H
#define J35_CRUISE_H

// 在 ed_fm_simulate 里调用 (内部限频 2Hz): 读命令文件 -> 堆扫描 -> add_pos 注入
extern void j35CruiseTick(void);

#endif // J35_CRUISE_H
