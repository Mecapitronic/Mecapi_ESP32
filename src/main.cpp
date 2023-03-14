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
                            1,         /* priority of the task */
                            &Task1,    /* Task handle to keep track of created task */
                            0);        /* pin task to core 0 */
    delay(500);

    // create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
    xTaskCreatePinnedToCore(Task2code, /* Task function. */
                            "Task2",   /* name of task. */
                            10000,     /* Stack size of task */
                            NULL,      /* parameter of the task */
                            1,         /* priority of the task */
                            &Task2,    /* Task handle to keep track of created task */
                            1);        /* pin task to core 1 */
    delay(500);
}

void loop()
{
    // put your main code here, to run repeatedly:
    ld06.Read_lidar_data();
}

// Note the 1ms delay, this is need so the watchdog doesn't get confused
void Task1code(void *pvParameters) {
    SERIAL_PC.print("Task1 running on core ");
    SERIAL_PC.println(xPortGetCoreID());
    delay(2000);
    while (1) {
        delay(1);
    }
}

void Task2code(void *pvParameters) {
    SERIAL_PC.print("Task2 running on core ");
    SERIAL_PC.println(xPortGetCoreID());
    delay(2000);
    while (1) {
        delay(1);
    }
}