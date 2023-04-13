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

    Robot::Init();
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
    LD06::PacketLidar lidar;
    Robot_t robot;
    while (1)
    {
        Debugger::checkSerial();

        if (LD06::ReadSerial())
        {
            LD06::Analyze();
        }
        if (Robot::ReadSerial())
        {
            Robot::Analyze();
        }

        lidar = LD06::GetData();
        for (int i = 0; i < LD06::data_packet_size; i++)
        {
            // Filter point before sending to queue : increase speed for later calculus
            if (lidar.dataPoint[i].distance > MIN_DISTANCE && lidar.dataPoint[i].distance < MAX_DISTANCE &&
                lidar.dataPoint[i].confidence > MIN_QUALITY)
                xQueueSend(queue, &lidar.dataPoint[i], 0);
        }
        vTaskDelay(1);
    }
}

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void Task2code(void *pvParameters)
{
    LD06::PointLidar turn[queueSize];
    LD06::PointLidar point;
    while (1)
    {
        if (uxQueueMessagesWaiting(queue) > 0)
        {
            xQueueReceive(queue, &point, portTICK_PERIOD_MS * 0);
            LD06::AggregatePoint(point);
        }
        vTaskDelay(1);
    }
}

void Print(HardwareSerial s, PolarPoint p, bool debug)
{
    if (debug) s.print("Angle:");
    s.print((int)p.angle);
    if (debug)
        s.print(" Distance:");
    else
        s.print(";");
    s.print(p.distance);
    if (debug)
        s.print(" Confidence:");
    else
        s.print(";");
    s.println(p.confidence);
}

void Print(HardwareSerial s, Point p, bool debug)
{
    if (debug) s.print("X:");
    s.print((int)p.x);
    if (debug)
        s.print(" Y:");
    else
        s.print(";");
    s.print((int)p.y);
}