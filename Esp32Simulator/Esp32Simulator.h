#ifndef _FIRMWARE_H_
#define _FIRMWARE_H_

#ifndef _USE_OLD_OSTREAMS
using namespace std;
#endif

#ifdef _VISUAL_STUDIO
#else
#endif

#define HAVE_STRUCT_TIMESPEC

#pragma comment(lib, "winmm.lib")
#pragma comment( user, "Compiled on " __DATE__ " at " __TIME__ )

#ifndef _WINDLL
#include <iostream>
#endif

#include <pthread.h>

//#include "Serial.h"

#include <Windows.h>
#include <chrono>

#include <string>
#include <vector>

#include <time.h>

/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>

/* Visual studio intrinsics used so the __debugbreak() function is available
 * should an assert get hit. */
#include <intrin.h>

#include "main.h"

extern "C"
{

#ifdef _WINDLL
	#define dll __declspec(dllexport)
	dll int Firmware(void);
#else
	#define dll
	#define Firmware main
#endif

	dll void AbortSimulator(void);
}

#endif