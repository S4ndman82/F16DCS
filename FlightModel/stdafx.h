// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
#endif

// Windows Header Files:
#include <windows.h>

// might be good idea to have but does not matter when not using windows api directly..
//#define _WIN32_WINNT 0x0600

// Visual Studio is rather dumb at times..
// -> moved to project settings
/*#define _USE_MATH_DEFINES*/ 
#include <cmath>

#include <malloc.h>
#include <memory.h>

