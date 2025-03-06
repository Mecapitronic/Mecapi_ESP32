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

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* FreeRTOS+Trace includes. */
#include "trcRecorder.h"

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


#define FreeRTOS
#ifdef FreeRTOS

	/* This demo uses heap_5.c, and these constants define the sizes of the regions
	 * that make up the total heap.  heap_5 is only used for test and example purposes
	 * as this demo could easily create one large heap region instead of multiple
	 * smaller heap regions - in which case heap_4.c would be the more appropriate
	 * choice.  See http://www.freertos.org/a00111.html for an explanation. */
#define mainREGION_1_SIZE                     8201
#define mainREGION_2_SIZE                     23905
#define mainREGION_3_SIZE                     16807

	 /* This demo allows for users to perform actions with the keyboard. */
#define mainNO_KEY_PRESS_VALUE                -1
#define mainOUTPUT_TRACE_KEY                  't'
#define mainINTERRUPT_NUMBER_KEYBOARD         3

/* This demo allows to save a trace file. */
#define mainTRACE_FILE_NAME                   "Trace.dump"
/*
 * This demo uses heap_5.c, so start by defining some heap regions.  It is not
 * necessary for this demo to use heap_5, as it could define one large heap
 * region.  Heap_5 is only used for test and example purposes.  See
 * https://www.FreeRTOS.org/a00111.html for an explanation.
 */
	static void prvInitialiseHeap(void);

	/*
	 * Prototypes for the standard FreeRTOS application hook (callback) functions
	 * implemented within this file.  See http://www.freertos.org/a00016.html .
	 */
	void vApplicationMallocFailedHook(void);
	void vApplicationIdleHook(void);
	void vApplicationStackOverflowHook(TaskHandle_t pxTask,
		char* pcTaskName);
	void vApplicationTickHook(void);
	void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
		StackType_t** ppxIdleTaskStackBuffer,
		uint32_t* pulIdleTaskStackSize);
	void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
		StackType_t** ppxTimerTaskStackBuffer,
		uint32_t* pulTimerTaskStackSize);

	/*
	 * Writes trace data to a disk file when the trace recording is stopped.
	 * This function will simply overwrite any trace files that already exist.
	 */
	static void prvSaveTraceFile(void);

	/*
	 * Windows thread function to capture keyboard input from outside of the
	 * FreeRTOS simulator. This thread passes data safely into the FreeRTOS
	 * simulator using a stream buffer.
	 */
	static DWORD WINAPI prvWindowsKeyboardInputThread(void* pvParam);

	/*
	 * Interrupt handler for when keyboard input is received.
	 */
	static uint32_t prvKeyboardInterruptHandler(void);

	/*
	 * Keyboard interrupt handler for the blinky demo.
	 */
	extern void vBlinkyKeyboardInterruptHandler(int xKeyPressed);

	/*-----------------------------------------------------------*/

	/* When configSUPPORT_STATIC_ALLOCATION is set to 1 the application writer can
	 * use a callback function to optionally provide the memory required by the idle
	 * and timer tasks.  This is the stack that will be used by the timer task.  It is
	 * declared here, as a global, so it can be checked by a test that is implemented
	 * in a different file. */
	//StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];


	/* Thread handle for the keyboard input Windows thread. */
	static HANDLE xWindowsKeyboardInputThreadHandle = NULL;

	/* This stores the last key pressed that has not been handled.
	 * Keyboard input is retrieved by the prvWindowsKeyboardInputThread
	 * Windows thread and stored here. This is then read by the idle
	 * task and handled appropriately. */
	static int xKeyPressed = mainNO_KEY_PRESS_VALUE;

	/*-----------------------------------------------------------*/
#endif

#endif