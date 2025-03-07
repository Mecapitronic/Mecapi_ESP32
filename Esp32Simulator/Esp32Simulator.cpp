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
