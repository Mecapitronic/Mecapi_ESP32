/********** INCLUDE *********/
#include "Esp32Simulator.h"

/********** VARIABLES *******/
vector<string> messageUartTX;
vector<string> messageUartRX;
int indexEcriture;
int indexLecture;
//http://franckh.developpez.com/tutoriels/posix/pthreads/
pthread_t pthread_SETUP;
pthread_t pthread_LOOP;
pthread_t pthread_uart;

bool arret = false;

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
        indexLecture = 0;
        int indexLectureTmp = 0;
        string messageRX = "";
        string messageTX = "";
        while (!arret)
        {
            if (indexLecture != indexLectureTmp)
            {
                messageRX = messageUartRX[indexLectureTmp];
                indexLectureTmp++;
                if (indexLectureTmp >= 100)
                    indexLectureTmp = 0;
                // RX1Interrupt(messageRX);
                messageRX = "";
            }

            // if (U1STAbits.TRMT == 0)
            {
                // messageTX += U1TXREG;
                // if (U1TXREG == 10)
                {
                    messageUartTX[indexEcriture] = messageTX;
                    indexEcriture++;
                    if (indexEcriture >= 100)
                        indexEcriture = 0;
                    messageTX = "";
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
	vTaskDelay(1000);
	ret = pthread_cancel(pthread_SETUP);
	ret = pthread_join(pthread_SETUP, NULL);
        ret = pthread_cancel(pthread_uart);
        ret = pthread_join(pthread_uart, NULL);
        ret = pthread_cancel(pthread_LOOP);
        ret = pthread_join(pthread_LOOP, NULL);
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
    messageUartRX.clear();
    for (int i = 0; i < 100; i++)
    {
        messageUartRX.push_back("");
	}

        myprintf("Starting Esp32Simulator !\n");

        arret = false;
	//Lancement des thread Interruption, Setup, Loop et UART reception/transmission

        // ret = pthread_create(&pthread_uart, NULL, thread_uart, NULL);
        // pthread_setname_np(pthread_uart, "UART");
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

#ifndef _WINDLL
        string in = "";
        myprintf("STARTING");
        myprintf(" !\n");
        while (in != "exit" || in != "quit" || in != "close")
        {
            cin >> in;
            // SendUART(in.c_str());
            Serial.incoming = in + '\n';
            Serial.bytes = in.length() + 1;
        }
        AbortSimulator();
#endif

	ret = pthread_join(pthread_LOOP, NULL);
	AbortSimulator();

	return ret;
}
