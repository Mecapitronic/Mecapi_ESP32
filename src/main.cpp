#include "main.h"

void setup()
{
    // put your setup code here, to run once:
    Debugger::init();

    queue = xQueueCreate(queueSize, sizeof(uint16_t));
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

    while (1)
    {
        if (a010.ReadSerial())
        {
            a010.FillStructure();
            a010.CheckContinuity();

            a010Packet = a010.GetData();

            // distance_mm = a010Packet.payload[312] * QUANTIZATION_MM;

            xQueueSend(queue, &a010Packet, 0);
        }
        vTaskDelay(1);
    }
}

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void Task2code(void *pvParameters)
{
    a010_frame_t a010Packet;
    uint16_t distance_mm = 0;
    Point3D p = {0, 0, 0};
    while (1)
    {
        if (uxQueueMessagesWaiting(queue) > 0)
        {
            if (xQueueReceive(queue, &a010Packet, portTICK_PERIOD_MS * 0))
            {
                /*
                for (int x = 1; x <= 25; x++)
                {
                    for (int y = 1; y <= 25; y++)
                    {
                        p.x = x;
                        p.y = y;
                        p.z = a010Packet.payload[x + (y - 1) * 25] * QUANTIZATION_MM;
                        // Debugger::plot3D(p, "p" + String(x) + "_" + String(y));
                        Debugger::plot3Dpy(p);
                    }
                }*/
                a010.GetPointCloudFromFrame(a010Packet);
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
