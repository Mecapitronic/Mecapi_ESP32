#include "main.h"

TaskHandle_t Task1;
TaskHandle_t Task2;

QueueHandle_t queue;
int queueSize = 500;

const int MIN_DISTANCE = 0;
const int MAX_DISTANCE = 500;
const int MIN_QUALITY = 100;

void setup()
{
    // put your setup code here, to run once:
    Debugger::init(VERBOSE);

    queue = xQueueCreate(queueSize, sizeof(LD06::PointLidar));
    if (queue == NULL) SERIAL_PC.println("Error creating the queue");

    delay(500);
    // SERIAL_PC.begin(230400);
    // SERIAL_PC.print("Start PC ! ");
    LD06::Init();
    // SERIAL_PC.print("Start Lidar ! ");

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
        LD06::Read_lidar_data();
        LD06::PacketLidar data = LD06::Calc_lidar_data();
        for (int i = 0; i < PACKET_SIZE; i++)
        {
            // Filter point before sending to queue : increase speed for later calculus
            if (data.dataPoint[i].distance > MIN_DISTANCE && data.dataPoint[i].distance < MAX_DISTANCE &&
                data.dataPoint[i].confidence > MIN_QUALITY)
                xQueueSend(queue, &data.dataPoint[i], 0);
        }
        vTaskDelay(1);
    }
}

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void Task2code(void *pvParameters)
{
    int lastAngle = 0;
    int index = 0;
    LD06::PointLidar turn[queueSize];
    LD06::PointLidar p;
    while (1)
    {
        if (uxQueueMessagesWaiting(queue) > 0)
        {
            xQueueReceive(queue, &p, portTICK_PERIOD_MS * 0);

            // a new turn is starting
            if (p.angle < lastAngle)
            {
                // we process the previous data
                if (index > 0)
                {
                    // SERIAL_PC.println(index);
                    LD06::Filter_lidar_data(turn, index);
                }
                index = 0;
            }
            turn[index] = p;
            if (index < queueSize) index++;
            lastAngle = p.angle;
        }
        vTaskDelay(1);
    }
}