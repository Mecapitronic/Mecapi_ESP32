#include "main.h"

void setup()
{
    // put your setup code here, to run once:
    ESP32_Helper::ESP32_Helper(921600);

    myQueue = xQueueCreate(queueSize, sizeof(uint8_t));
    if (myQueue == NULL)
    {
        println("Error creating the queue !");
    }

    println("A010 setup");
    a010.Initialisation();
    delay(500);

    println("Dbscan setup");
    dbscan.Config(60.0f, 3, TCHEBYCHEV);
    delay(500);

    println("Create Task1code");
    // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(Task1code, /* Task function. */
                            "Task1",   /* name of task. */
                            20000,     /* Stack size of task */
                            NULL,      /* parameter of the task */
                            10,        /* priority of the task */
                            &Task1,    /* Task handle to keep track of created task */
                            0);        /* pin task to core 0 */

    println("Create Task2code");
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
    println("Start Task1code");

    vector<vector<uint16_t>> _clusters;

    while (1)
    {
        try
        {
            // we enter once we have the complete frame
            if (a010.ReadSerial())
            {
                // a010.CheckContinuity();

                println("***");

                // for (uint16_t i = 0; i < PICTURE_SIZE; i++)  // FIXME: n'affiche plus rien !
                //{
                //  String data = "" + String(a010.cloudFrame[i].x) + " " + String(a010.cloudFrame[i].y) + " " + String(a010.cloudFrame[i].z) + " " + String("65520");
                //  println(data);
                //}
                // println("---");
                // println();

                // Erasing all previous _clusters
                for (size_t i = 0; i < _clusters.size(); i++)
                {
                    _clusters[i].clear();
                }
                _clusters.clear();
                println("Dbscan Process");
                //_clusters = dbscan.Process((Dbscan::Point3D *)&(a010.cloudFrame));
                //_clusters = dbscan.Process((Point4D *)&(a010.cloudFrame));
                // dbscan.displayStats();

                // xQueueSend(myQueue, &a010Packet, 0);
            }
        }
        catch (std::exception const &e)
        {
            print("error : ");
            println(e.what());
        }
        vTaskDelay(1);
    }
}

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void Task2code(void *pvParameters)
{
    println("Start Task2code");
    uint8_t test = 0;
    while (1)
    {
        try
        {
            if (uxQueueMessagesWaiting(myQueue) > 0)
            {
                if (xQueueReceive(myQueue, &test, portTICK_PERIOD_MS * 0))
                {
                    println("xQueueReceive : ", test, "");
                }
            }

            // Check if we get commands from operator via debug serial
            ESP32_Helper::UpdateSerial();

            if (ESP32_Helper::HasWaitingCommand())
            {
                Command cmd = ESP32_Helper::GetCommand();

                if (cmd.cat == ("A010"))
                {
                    SERIAL_A010.write(cmd.cmd.c_str());
                }
                else if (cmd.cat == ("Debug"))
                {
                    if (cmd.cmd == "Steps")
                        Debugger::AddSteps(cmd.num);
                    else if (cmd.cmd == "Enable")
                        Debugger::EnableDebugger((Enable)cmd.num);
                }
                else if (cmd.cat == ("Print"))
                {
                    if (cmd.cmd == "Level")
                        Printer::PrintLevel((Level)cmd.num);
                    else if (cmd.cmd == "Enable")
                        Printer::PrintEnable((Enable)cmd.num);
                }
            }
        }
        catch (std::exception const &e)
        {
            print("error : ");
            println(e.what());
        }
        vTaskDelay(1);
    }
}
