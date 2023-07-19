#include "main.h"

void setup()
{
    // put your setup code here, to run once:
    Debugger::init();

    // queue = xQueueCreate(queueSize, sizeof(a010_frame_t));
    // if (queue == NULL)
    {
        //      Debugger::log("Error creating the queue", ERROR);
    }

    SERIAL_DEBUG.println("A010 setup");
    a010.Initialisation();
    delay(500);

    SERIAL_DEBUG.println("Dbscan setup");
    dbscan.Config(60.0f, 3, TCHEBYCHEV);
    delay(500);

    SERIAL_DEBUG.println("Create Task1code");
    // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(Task1code, /* Task function. */
                            "Task1",   /* name of task. */
                            20000,     /* Stack size of task */
                            NULL,      /* parameter of the task */
                            10,        /* priority of the task */
                            &Task1,    /* Task handle to keep track of created task */
                            0);        /* pin task to core 0 */

    SERIAL_DEBUG.println("Create Task2code");
    // create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
    xTaskCreatePinnedToCore(Task2code, /* Task function. */
                            "Task2",   /* name of task. */
                            20000,     /* Stack size of task */
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
    SERIAL_DEBUG.println("Start Task1code");

    vector<vector<uint16_t>> _clusters;

    while (1)
    {
        try
        {
            // we enter once we have the complete frame
            if (a010.ReadSerial())
            {
                // a010.CheckContinuity();

                SERIAL_DEBUG.println("***");

                for (uint16_t i = 0; i < PICTURE_SIZE; i++)  // FIXME: n'affiche plus rien !
                {
                    // String data = "" + String(a010.cloudFrame[i].x) + " " + String(a010.cloudFrame[i].y) + " " + String(a010.cloudFrame[i].z) + " " + String("65520");
                    // SERIAL_DEBUG.println(data);
                }
                SERIAL_DEBUG.println("---");
                // SERIAL_DEBUG.println();

                // Erasing all previous _clusters
                for (size_t i = 0; i < _clusters.size(); i++)
                {
                    _clusters[i].clear();
                }
                _clusters.clear();
                SERIAL_DEBUG.println("Dbscan Process");
                //_clusters = dbscan.Process((Dbscan::Point3D *)&(a010.cloudFrame));
                _clusters = dbscan.Process((Point4D *)&(a010.cloudFrame));
                dbscan.displayStats();

                // xQueueSend(queue, &a010Packet, 0);
            }
        }
        catch (std::exception const &e)
        {
            Debugger::log("error : ", e.what());
        }
        vTaskDelay(1);
    }
}

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void Task2code(void *pvParameters)
{
    SERIAL_DEBUG.println("Start Task2code");

    while (1)
    {
        if (uxQueueMessagesWaiting(queue) > 0)
        {
            //   if (xQueueReceive(queue, &a010Packet, portTICK_PERIOD_MS * 0))
            {
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
