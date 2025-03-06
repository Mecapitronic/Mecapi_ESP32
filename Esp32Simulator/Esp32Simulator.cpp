/********** INCLUDE *********/
#include "Esp32Simulator.h"

/********** VARIABLES *******/
vector<string> messageUartTX;
vector<string> messageUartTX2;
vector<string> messageUartRX;
vector<string> messageUartRX2;
int indexEcriture;
int indexEcriture2;
int indexLecture;
int indexLecture2;
//http://franckh.developpez.com/tutoriels/posix/pthreads/
pthread_t pthread_SETUP;
pthread_t pthread_LOOP;
pthread_t pthread_Interruption;
pthread_t pthread_uart;

bool arret = false;
bool uart_run = false;
bool interruption_run = false;

/********** THREAD INTERRUPTION dsPIC ******/
static void* thread_Interruption(void* p_data)
{
	//chrono::steady_clock::time_point  start, mid, end;

	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
	myprintf("Start thread Interruption\n");
	int i = 0;
	while (!arret)
	{
		if (interruption_run)
		{
			//start = chrono::high_resolution_clock::now();
			//TIMER_PRIMAIRE_INT();
			i += 5;

			//mid = chrono::high_resolution_clock::now();
			//myprintf("interruption 1: %0.3f ms, time calcul: %d ms\n", (mid - start).count() / 1e6, current_time);
			timerSleep(0.005);
			//end = chrono::high_resolution_clock::now();
			//myprintf("interruption 1: %0.3f ms, current_time: %d ms\n", (end - start).count() / 1e6, current_time);
		}
	}
	myprintf("End thread Interruption\n");
	return NULL;
}

static void* thread_SETUP(void* p_data)
{
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
	myprintf("Start thread SETUP\n");
	setup();
	myprintf("End thread SETUP\n");
	return NULL;
}

static void* thread_LOOP(void* p_data)
{
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
	myprintf("Start thread LOOP\n");
	while (!arret)
	{
		loop();
	}
	myprintf("End thread LOOP\n");
	return NULL;
}

static void* thread_uart(void* p_data)
{
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
	myprintf("Start thread Uart\n");

	indexEcriture = 0;
	indexEcriture2 = 0;
	indexLecture = 0;
	indexLecture2 = 0;
	int indexLectureTmp = 0;
	int indexLecture2Tmp = 0;
	string messageRX = "";
	string messageRX2 = "";
	string messageTX = "";
	string messageTX2 = "";
	while (!arret)
	{
		if (uart_run)
		{
			if (indexLecture != indexLectureTmp)
			{
				messageRX = messageUartRX[indexLectureTmp];
				indexLectureTmp++;
				if (indexLectureTmp >= 100)
					indexLectureTmp = 0;
				//RX1Interrupt(messageRX);
				messageRX = "";
			}

			if (indexLecture2 != indexLecture2Tmp)
			{
				messageRX2 = messageUartRX2[indexLecture2Tmp];
				indexLecture2Tmp++;
				if (indexLecture2Tmp >= 100)
					indexLecture2Tmp = 0;
				//RX2Interrupt(messageRX2);
				messageRX2 = "";
			}

			//if (U1STAbits.TRMT == 0)
			{
				//messageTX += U1TXREG;
				//if (U1TXREG == 10)
				{
					messageUartTX[indexEcriture] = messageTX;
					indexEcriture++;
					if (indexEcriture >= 100)
						indexEcriture = 0;
					messageTX = "";
				}
			}

			//if (U2STAbits.TRMT == 0)
			{
				//messageTX2 += U2TXREG;
				//if (U2TXREG == 10)
				{
					messageUartTX2[indexEcriture2] = messageTX2;
					indexEcriture2++;
					if (indexEcriture2 >= 100)
						indexEcriture2 = 0;
					messageTX2 = "";
				}
			}
		}
	}

	myprintf("End thread Uart\n");
	return NULL;
}

void AbortSimulator(void)
{
	arret = true;
	int ret = -1;
	myprintf("Abort des Thread ! \n");
	timerSleep(1);
	ret = pthread_cancel(pthread_SETUP);
	ret = pthread_join(pthread_SETUP, NULL);
	ret = pthread_cancel(pthread_LOOP);
	ret = pthread_join(pthread_LOOP, NULL);

	ret = pthread_cancel(pthread_Interruption);
	ret = pthread_join(pthread_Interruption, NULL);
	ret = pthread_cancel(pthread_uart);
	ret = pthread_join(pthread_uart, NULL);

	myprintf("Abort des Thread OK :) \n");


}


int Firmware(void)
{
	int ret = 0;
	
	messageUartTX.clear();
	for (int i = 0; i < 100; i++)
	{
		messageUartTX.push_back("");
	}

	messageUartTX2.clear();
	for (int i = 0; i < 100; i++)
	{
		messageUartTX2.push_back("");
	}

	messageUartRX.clear();
	for (int i = 0; i < 100; i++)
	{
		messageUartRX.push_back("");
	}

	messageUartRX2.clear();
	for (int i = 0; i < 100; i++)
	{
		messageUartRX2.push_back("");
	}
	
	myprintf("Starting Esp32Simulator !\n");

	arret = false;
	//Lancement des thread Interruption, Setup, Loop et UART reception/transmission

	ret = pthread_create(&pthread_Interruption, NULL, thread_Interruption, NULL);
	pthread_setname_np(pthread_Interruption, "Interruption");
	if (!ret)
	{
		ret = pthread_create(&pthread_uart, NULL, thread_uart, NULL);
		pthread_setname_np(pthread_uart, "UART");
		if (!ret)
		{
			ret = pthread_create(&pthread_SETUP, NULL, thread_SETUP, NULL);
			pthread_setname_np(pthread_SETUP, "SETUP");

			// Wait for Setup to finish then start Loop
			ret = pthread_join(pthread_SETUP, NULL);
			if (!ret)
			{
				ret = pthread_create(&pthread_LOOP, NULL, thread_LOOP, NULL);
				pthread_setname_np(pthread_LOOP, "LOOP");
				if (!ret)
				{
					myprintf("Lancement des thread OK !\n");
				}
			}
		}
	}

#ifndef _WINDLL
	myprintf("Type 'start' to start the PILOT Match, or 'test' to enter test mode : \n");
	string in;
	cin >> in;
	myprintf("STARTING");
	if (in == "TEST" || in == "Test" || in == "test")
	{
		myprintf(" WITH MODE TEST");
	}
	myprintf(" !\n");
	while (in != "exit")
	{
		cin >> in;
		//SendUART(in.c_str());
	}
	AbortSimulator();
#endif

	ret = pthread_join(pthread_LOOP, NULL);
	AbortSimulator();

	return ret;
}

#ifdef FreeRTOS

void vApplicationMallocFailedHook(void)
{
    /* vApplicationMallocFailedHook() will only be called if
     * configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
     * function that will get called if a call to pvPortMalloc() fails.
     * pvPortMalloc() is called internally by the kernel whenever a task, queue,
     * timer or semaphore is created.  It is also called by various parts of the
     * demo application.  If heap_1.c, heap_2.c or heap_4.c is being used, then the
     * size of the	heap available to pvPortMalloc() is defined by
     * configTOTAL_HEAP_SIZE in FreeRTOSConfig.h, and the xPortGetFreeHeapSize()
     * API function can be used to query the size of free heap space that remains
     * (although it does not provide information on how the remaining heap might be
     * fragmented).  See http://www.freertos.org/a00111.html for more
     * information. */
    vAssertCalled(__LINE__, __FILE__);
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
     * to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
     * task.  It is essential that code added to this hook function never attempts
     * to block in any way (for example, call xQueueReceive() with a block time
     * specified, or call vTaskDelay()).  If application tasks make use of the
     * vTaskDelete() API function to delete themselves then it is also important
     * that vApplicationIdleHook() is permitted to return to its calling function,
     * because it is the responsibility of the idle task to clean up memory
     * allocated by the kernel to any task that has since deleted itself. */

#if ( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY != 1 )
    {
        /* Call the idle task processing used by the full demo.  The simple
         * blinky demo does not use the idle task hook. */
        //vFullDemoIdleFunction();
    }
#endif
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask,
    char* pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;

    /* Run time stack overflow checking is performed if
     * configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
     * function is called if a stack overflow is detected.  This function is
     * provided as an example only as stack overflow checking does not function
     * when running the FreeRTOS Windows port. */
    vAssertCalled(__LINE__, __FILE__);
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
    /* This function will be called by each tick interrupt if
    * configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
    * added here, but the tick hook is called from an interrupt context, so
    * code must not attempt to block, and only the interrupt safe FreeRTOS API
    * functions can be used (those that end in FromISR()). */

#if ( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY != 1 )
    {
        //vFullDemoTickHookFunction();
    }
#endif /* mainCREATE_SIMPLE_BLINKY_DEMO_ONLY */
}
/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook(void)
{
    /* This function will be called once only, when the daemon task starts to
     * execute	(sometimes called the timer task).  This is useful if the
     * application includes initialisation code that would benefit from executing
     * after the scheduler has been started. */
}
/*-----------------------------------------------------------*/

void vAssertCalled(unsigned long ulLine,
    const char* const pcFileName)
{
    static BaseType_t xPrinted = pdFALSE;
    volatile uint32_t ulSetToNonZeroInDebuggerToContinue = 0;

    /* Called if an assertion passed to configASSERT() fails.  See
     * http://www.freertos.org/a00110.html#configASSERT for more information. */

     /* Parameters are not used. */
    (void)ulLine;
    (void)pcFileName;

    taskENTER_CRITICAL();
    {
        printf("ASSERT! Line %ld, file %s, GetLastError() %ld\r\n", ulLine, pcFileName, GetLastError());

        /* Stop the trace recording and save the trace. */
        (void)xTraceDisable();
        prvSaveTraceFile();

        /* Cause debugger break point if being debugged. */
        //__debugbreak();

        /* You can step out of this function to debug the assertion by using
         * the debugger to set ulSetToNonZeroInDebuggerToContinue to a non-zero
         * value. */
        while (ulSetToNonZeroInDebuggerToContinue == 1)
        {
            __asm {
                NOP
            };
            __asm {
                NOP
            };
        }

        /* Re-enable the trace recording. */
        (void)xTraceEnable(TRC_START);
    }
    taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

static void prvSaveTraceFile(void)
{
    FILE* pxOutputFile;

    fopen_s(&pxOutputFile, mainTRACE_FILE_NAME, "wb");

    if (pxOutputFile != NULL)
    {
        //fwrite(RecorderDataPtr, sizeof(RecorderDataType), 1, pxOutputFile);
        fclose(pxOutputFile);
        printf("\r\nTrace output saved to %s\r\n\r\n", mainTRACE_FILE_NAME);
    }
    else
    {
        printf("\r\nFailed to create trace dump file\r\n\r\n");
    }
}
/*-----------------------------------------------------------*/

static void prvInitialiseHeap(void)
{
    /* The Windows demo could create one large heap region, in which case it would
     * be appropriate to use heap_4.  However, purely for demonstration purposes,
     * heap_5 is used instead, so start by defining some heap regions.  No
     * initialisation is required when any other heap implementation is used.  See
     * http://www.freertos.org/a00111.html for more information.
     *
     * The xHeapRegions structure requires the regions to be defined in start address
     * order, so this just creates one big array, then populates the structure with
     * offsets into the array - with gaps in between and messy alignment just for test
     * purposes. */
    static uint8_t ucHeap[configTOTAL_HEAP_SIZE];
    volatile uint32_t ulAdditionalOffset = 19; /* Just to prevent 'condition is always true' warnings in configASSERT(). */
    const HeapRegion_t xHeapRegions[] =
    {
        /* Start address with dummy offsets						Size */
        { ucHeap + 1,                                          mainREGION_1_SIZE },
        { ucHeap + 15 + mainREGION_1_SIZE,                     mainREGION_2_SIZE },
        { ucHeap + 19 + mainREGION_1_SIZE + mainREGION_2_SIZE, mainREGION_3_SIZE },
        { NULL,                                                0                 }
    };

    /* Sanity check that the sizes and offsets defined actually fit into the
     * array. */
    configASSERT((ulAdditionalOffset + mainREGION_1_SIZE + mainREGION_2_SIZE + mainREGION_3_SIZE) < configTOTAL_HEAP_SIZE);

    /* Prevent compiler warnings when configASSERT() is not defined. */
    (void)ulAdditionalOffset;

    vPortDefineHeapRegions(xHeapRegions);
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 * used by the Idle task. */
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
    StackType_t** ppxIdleTaskStackBuffer,
    uint32_t* pulIdleTaskStackSize)
{
    /* If the buffers to be provided to the Idle task are declared inside this
     * function then they must be declared static - otherwise they will be allocated on
     * the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
     * state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

#if (configSUPPORT_STATIC_ALLOCATION == 0)


/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
 * application must provide an implementation of vApplicationGetTimerTaskMemory()
 * to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
    StackType_t** ppxTimerTaskStackBuffer,
    uint32_t* pulTimerTaskStackSize)
{
    /* If the buffers to be provided to the Timer task are declared inside this
     * function then they must be declared static - otherwise they will be allocated on
     * the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
     * task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif
/*-----------------------------------------------------------*/

/*
 * Interrupt handler for when keyboard input is received.
 */
static uint32_t prvKeyboardInterruptHandler(void)
{
    /* Handle keyboard input. */
    switch (xKeyPressed)
    {
    case mainNO_KEY_PRESS_VALUE:
        break;
    case mainOUTPUT_TRACE_KEY:
        /* Saving the trace file requires Windows system calls, so enter a critical
           section to prevent deadlock or errors resulting from calling a Windows
           system call from within the FreeRTOS simulator. */
        portENTER_CRITICAL();
        {
            (void)xTraceDisable();
            prvSaveTraceFile();
            (void)xTraceEnable(TRC_START);
        }
        portEXIT_CRITICAL();
        break;
    default:
#if ( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY == 1 )
    {
        /* Call the keyboard interrupt handler for the blinky demo. */
        vBlinkyKeyboardInterruptHandler(xKeyPressed);
    }
#endif
           break;
    }

    /* This interrupt does not require a context switch so return pdFALSE */
    return pdFALSE;
}

/*-----------------------------------------------------------*/
/*
 * Windows thread function to capture keyboard input from outside of the
 * FreeRTOS simulator. This thread passes data into the simulator using
 * an integer.
 */
static DWORD WINAPI prvWindowsKeyboardInputThread(void* pvParam)
{
    (void)pvParam;

    for (; ; )
    {
        /* Block on acquiring a key press. */
        xKeyPressed = _getch();

        /* Notify FreeRTOS simulator that there is a keyboard interrupt.
         * This will trigger prvKeyboardInterruptHandler.
         */
        vPortGenerateSimulatedInterrupt(mainINTERRUPT_NUMBER_KEYBOARD);
    }

    /* Should not get here so return negative exit status. */
    return (DWORD)(-1);
}

/*-----------------------------------------------------------*/

/* The below code is used by the trace recorder for timing. */
static uint32_t ulEntryTime = 0;

void vTraceTimerReset(void)
{
    ulEntryTime = xTaskGetTickCount();
}

uint32_t uiTraceTimerGetFrequency(void)
{
    return configTICK_RATE_HZ;
}

uint32_t uiTraceTimerGetValue(void)
{
    return(xTaskGetTickCount() - ulEntryTime);
}
#endif