#include "main.h"

LD06 ld06;
TaskHandle_t Task1;
TaskHandle_t Task2;

void setup()
{
    // put your setup code here, to run once:
    delay(500);
    SERIAL_PC.begin(230400);
    SERIAL_PC.print("Start PC ! ");
    ld06.Init();
    SERIAL_PC.print("Start Lidar ! ");

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
    while (1)
    {
        ld06.Read_lidar_data();
        ld06.Calc_lidar_data();
        ld06.Send_queue_data();
        vTaskDelay(1);
    }
}

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void Task2code(void *pvParameters)
{
    while (1)
    {
        LD06::PointLidar p;
        while (ld06.Get_queue_length() > 0)
        {
            p = ld06.Get_queue_data();
            // ld06.Print_lidar_data(p);
        }
        vTaskDelay(1);
    }
}