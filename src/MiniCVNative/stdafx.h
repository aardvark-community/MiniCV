// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files:
#include <windows.h>



// TODO: reference additional headers your program requires here

#ifdef __APPLE__
#define DllExport(t) extern "C" t
#elif __GNUC__
#define DllExport(t) extern "C" t
#else
#define DllExport(t) extern "C"  __declspec( dllexport ) t __cdecl
#endif