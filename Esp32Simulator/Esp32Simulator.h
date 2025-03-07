#ifndef _FIRMWARE_H_
#define _FIRMWARE_H_

#ifndef _USE_OLD_OSTREAMS
using namespace std;
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

#include "main.h"

extern "C"
{
//#include "..\pilot.x\PILOT_main.h"

#ifdef _WINDLL
#define dll __declspec(dllexport)
	dll int Firmware(void);

	//when in dll mode, we print over the OutputDebugString
	void __cdecl myprintf(string str)
	{
		OutputDebugString(str.c_str());
	}
	void __cdecl myprintf(const char* format, ...)
	{
		char    buf[4096], * p = buf;
		va_list args;
		int     n;

		va_start(args, format);
		n = _vsnprintf(p, sizeof buf - 3, format, args); // buf-3 is room for CR/LF/NUL
		va_end(args);

		p += (n < 0) ? sizeof buf - 3 : n;

		while (p > buf && isspace(p[-1]))
			*--p = '\0';

		*p++ = '\r';
		*p++ = '\n';
		*p = '\0';

		OutputDebugString(buf);
	}
#else
#define dll
#define Firmware main

// When in Console mode, we print over it
#define myprintf cout<<
#endif

	dll void AbortSimulator(void);
}


	void timerSleep(double seconds) {
		using namespace std::chrono;

		static HANDLE timer = CreateWaitableTimer(NULL, FALSE, NULL);
		static double estimate = 5e-3;
		static double mean = 5e-3;
		static double m2 = 0;
		static int64_t count = 1;

		while (seconds - estimate > 1e-7) {
			double toWait = seconds - estimate;
			LARGE_INTEGER due;
			due.QuadPart = -int64_t(toWait * 1e7);
			steady_clock::time_point  start = high_resolution_clock::now();
			SetWaitableTimerEx(timer, &due, 0, NULL, NULL, NULL, 0);
			WaitForSingleObject(timer, INFINITE);
			steady_clock::time_point end = high_resolution_clock::now();

			double observed = (end - start).count() / 1e9;
			seconds -= observed;

			++count;
			double error = observed - toWait;
			double delta = error - mean;
			mean += delta / count;
			m2 += delta * (error - mean);
			double stddev = sqrt(m2 / (count - 1));
			estimate = mean + stddev;
		}

		// spin lock
		auto start = high_resolution_clock::now();
		while ((high_resolution_clock::now() - start).count() / 1e9 < seconds);
	}

#endif