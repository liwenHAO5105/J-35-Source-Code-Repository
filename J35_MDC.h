// =============================================================================
//  J35_MDC.h  —  arg183 爆索(MDC / Miniature Detonating Cord)通道
// -----------------------------------------------------------------------------
//  座舱内座舱盖玻璃上的爆索裂纹动画 = 座舱模型的 arg 183。
//  座舱 Lua 写不了座舱参数(同 arg181), 由本 DLL 代写:
//    座舱侧 cockpit_click.lua 检测弹射(iCommand 83)后写
//    <SavedGames>\J35DATA\J35_MDC_cmd.txt :  "J35MDC1 <seq> <v>"
//      v = 0 = 无裂纹(默认), 1 = 爆索触发
//    本 DLL 20Hz 读走, 在 ed_fm_set_fc3_cockpit_draw_args_v2 回调里写 array[183]。
//
//  构建: 与 J35_EFM.cpp / J35_HUDMath.cpp 一起编进同一个 DLL:
//    g++ -O2 -shared -static-libgcc -static-libstdc++ \
//        -o J35Cam.dll J35_EFM.cpp J35_HUDMath.cpp J35_MDC.cpp \
//        -I"D:\SteamLibrary\steamapps\common\DCSWorld\API\include"
// =============================================================================
#pragma once
#ifndef J35_MDC_H
#define J35_MDC_H
#include <stddef.h>

// 限频读 J35_MDC_cmd.txt (在 ed_fm_simulate 里调用)
extern void j35MdcTick(void);

// 在 ed_fm_set_fc3_cockpit_draw_args_v2 回调里写 array[183]
extern void j35MdcWrite(float *array, size_t size);

// 出生/任务重开: 归零 + 删通道文件 (在 ed_fm_cold_start 里调用)
extern void j35MdcReset(void);

#endif // J35_MDC_H
