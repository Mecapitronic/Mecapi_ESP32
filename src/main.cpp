#include "main.h"

void setup()
{
    // put your setup code here, to run once:
    Debugger::init();

    queue = xQueueCreate(queueSize, sizeof(PointLidar));
    if (queue == NULL)
    {
        Debugger::println("Error creating the queue");
    }
    robot = Robot();
    delay(500);
    lidar06 = Lidar();
    delay(500);
    tracker = Tracker();

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
    PacketLidar lidarPacket;

    while (1)
    {
        if (robot.ReadSerial())
        {
            // set the new robot position
            robot.Analyze();
        }

        if (lidar06.ReadSerial())
        {
            lidar06.Analyze();
            lidar06.CheckContinuity();
        }

        lidarPacket = lidar06.GetData();
        for (int i = 0; i < LIDAR_DATA_PACKET_SIZE; i++)
        {
            // Filter point before sending to queue : increase speed for later calculus
            // TODO increase the confidence limit to avoid abberations
            if (lidarPacket.dataPoint[i].confidence != 0)
            {
                xQueueSend(queue, &lidarPacket.dataPoint[i], 0);
            }
        }
        vTaskDelay(1);
    }
}

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void Task2code(void *pvParameters)
{
    PointLidar turn[queueSize];
    PointLidar point;
    RobotPosition_t data;
    while (1)
    {
        if (uxQueueMessagesWaiting(queue) > 0)
        {
            xQueueReceive(queue, &point, portTICK_PERIOD_MS * 0);
            lidar06.SearchForObstacles(point, &tracker, robot);
        }
        tracker.sendObstaclesToRobot(robot);
        tracker.untrackOldObstacles(robot);

        // Check if we get commands from operator via debug serial
        String cmd = Debugger::checkSerial();

        if (cmd.startsWith("Lidar:"))
        {
            try
            {
                cmd.remove(0, 6);
                int i = atoi(cmd.c_str());
                Debugger::print("Lidar: ");
                Debugger::println(i);
                lidar06.Config(-1, i, -1, -1, -1);
                // TODO : make a function for reading commands
            }
            catch (std::exception const &e)
            {
                Debugger::print("error : ");
                Debugger::println(e.what());
            }
        }

        vTaskDelay(1);
    }
}
