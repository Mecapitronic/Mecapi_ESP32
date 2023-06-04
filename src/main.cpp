#include "main.h"

// put function declarations here:
int myFunction(int, int);

void setup()
{
    // put your setup code here, to run once:
    Debugger::init();

    queue = xQueueCreate(queueSize, sizeof(int));
    if (queue == NULL)
    {
        Debugger::log("Error creating the queue", ERROR);
    }

    a010 = A010();
    delay(500);

    // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(Task1code, /* Task function. */
                            "Task1",   /* name of task. */
                            10000,     /* Stack size of task */
                            NULL,      /* parameter of the task */
                            10,        /* priority of the task */
                            &Task1,    /* Task handle to keep track of created task */
                            0);        /* pin task to core 0 */

    // create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
    xTaskCreatePinnedToCore(Task2code, /* Task function. */
                            "Task2",   /* name of task. */
                            10000,     /* Stack size of task */
                            NULL,      /* parameter of the task */
                            5,         /* priority of the task */
                            &Task2,    /* Task handle to keep track of created task */
                            1);        /* pin task to core 1 */
}

void loop()
{
    // Do not put code in the loop() when using freeRTOS.
    // loop() is the only task that is guaranteed to not be ran per tasking iteration.
    delay(1000);
}

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void Task1code(void *pvParameters)
{
    a010_frame_t a010Packet;
    int send = 0;
    uint16_t distance_mm, distance_mm_old = 0;

    while (1)
    {
        if (a010.ReadSerial())
        {
            a010.Analyze();
            a010Packet = a010.GetData();
            Debugger::log("", a010Packet);

            distance_mm_old = distance_mm;
            distance_mm = a010Packet.payload[312] * QUANTIZATION_MM;
            if (distance_mm_old != distance_mm)
            {
                SERIAL_DEBUG.println(distance_mm);
            }

            // we send one int
            xQueueSend(queue, &send, 0);
            Debugger::log("Sending ", send);
            send++;
        }
        vTaskDelay(1);
    }
}

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void Task2code(void *pvParameters)
{
    int receive = 0;
    while (1)
    {
        if (uxQueueMessagesWaiting(queue) > 0)
        {
            if (xQueueReceive(queue, &receive, portTICK_PERIOD_MS * 0))
            {
                Debugger::log("Receiving ", receive);
            }
        }

        // Check if we get commands from operator via debug serial
        String cmd = Debugger::checkSerial();

        if (cmd != "")
        {
            if (cmd.startsWith("cmdAT:"))
            {
                try
                {
                    cmd.remove(0, 6);
                    // int i = atoi(cmd.c_str());
                    Debugger::log("cmdAT: ", cmd);
                    SERIAL_A010.write(cmd.c_str());
                }
                catch (std::exception const &e)
                {
                    Debugger::log("error : ", e.what());
                }
            }
        }

        vTaskDelay(1);
    }
}
