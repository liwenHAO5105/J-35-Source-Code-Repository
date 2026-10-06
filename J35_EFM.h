// =============================================================================
//  J35_EFM.h  —  歼-35 EFM(External Flight Model) DLL 导出宏
// -----------------------------------------------------------------------------
//  J35_EFM.cpp 里每个 ed_fm_* 接口都用 J35EFM_API 修饰。
//  DCS 是用 GetProcAddress 按【未修饰的 C 名字】去找这些函数的, 所以必须是:
//        extern "C"  +  __declspec(dllexport)
//  只写 __declspec(dllexport) 会被 C++ 名字修饰成 ?ed_fm_simulate@@YAXN@Z 这种,
//  DCS 找不到 -> 模组直接加载失败。这也是为什么这个头文件必须存在。
//
//  构建命令(MinGW-w64 / MSYS2 ucrt64, 本机已有 C:\msys64\ucrt64\bin\g++.exe):
//      g++ -O2 -shared -static-libgcc -static-libstdc++ ^
//          -o J35_EFM.dll J35_EFM.cpp ^
//          -I"D:\SteamLibrary\steamapps\common\DCSWorld\API\include"
//  产物复制到: Saved Games\DCS\mods\aircraft\J-35\bin\J35_EFM.dll
// =============================================================================
#pragma once
#ifndef J35_EFM_H
#define J35_EFM_H

#ifdef _WIN32
#  define J35EFM_API extern "C" __declspec(dllexport)
#else
#  define J35EFM_API extern "C"
#endif

#endif // J35_EFM_H
