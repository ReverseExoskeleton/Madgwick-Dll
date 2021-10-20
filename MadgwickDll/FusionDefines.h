#ifndef FUSION_DEFINES_H
#define FUSION_DEFINES_H

#ifdef MADGWICKDLL_EXPORTS
#define MADGWICK_API extern "C" __declspec(dllexport)
#else
#define MADGWICK_API extern "C" __declspec(dllimport)
#endif

#endif
