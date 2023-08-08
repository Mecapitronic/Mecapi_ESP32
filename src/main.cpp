#include "main.h"

void setup()
{
    // put your setup code here, to run once:
    ESP32_Helper::ESP32_Helper(576000);

    myQueue = xQueueCreate(queueSize, sizeof(PolarPoint));
    if (myQueue == NULL)
    {
        println("Error creating the queue", LEVEL_ERROR);
    }
    robot = Robot();
    delay(500);
    lidar06 = Lidar();
    delay(500);
    tracker = Tracker();

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

    PacketLidar lidarPacket;

    while (1)
    {
        try
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

                lidarPacket = lidar06.GetData();
                int counter = 0;
                for (int i = 0; i < LIDAR_DATA_PACKET_SIZE; i++)
                {
                    // Filter point before sending to queue : increase speed for later calculus
                    // TODO increase the confidence limit to avoid aberrations

                    // if the point is out of bound, we will not use it
                    ConfigLidar configLidar = lidar06.GetConfig();
                    if (lidarPacket.dataPoint[i].distance < configLidar.minDistance || lidarPacket.dataPoint[i].distance > configLidar.maxDistance ||
                        lidarPacket.dataPoint[i].confidence < configLidar.minQuality)
                    {
                        counter++;
                    }
                    else
                    {
                        xQueueSend(myQueue, &lidarPacket.dataPoint[i], 0);
                    }
                }
                // TODO at least send 1 or 2 points to the queue (min max ?, middle ?) to end aggregation for obstacle
                if (counter == LIDAR_DATA_PACKET_SIZE)
                {
                    // we did not have any point to send, we send at least the last one.
                    xQueueSend(myQueue, &lidarPacket.dataPoint[LIDAR_DATA_PACKET_SIZE - 1], 0);
                    print("No point to send, Sending dull point", lidarPacket.dataPoint[LIDAR_DATA_PACKET_SIZE - 1]);
                }
                else
                {
                    println("Sending ", LIDAR_DATA_PACKET_SIZE - counter, " point");
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

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void Task2code(void *pvParameters)
{
    println("Start Task2code");

    PolarPoint turn[queueSize];
    PolarPoint point;
    RobotPosition data;
    while (1)
    {
        try
        {
            if (uxQueueMessagesWaiting(myQueue) > 0)
            {
                if (xQueueReceive(myQueue, &point, portTICK_PERIOD_MS * 0))
                {
                    lidar06.AggregatePoint(point, &tracker, robot);
                }
            }
            tracker.sendObstaclesToRobot(robot);
            tracker.untrackOldObstacles(robot);

            // Check if we get commands from operator via debug serial
            ESP32_Helper::UpdateSerial();

            if (ESP32_Helper::HasWaitingCommand())
            {
                Command cmd = ESP32_Helper::GetCommand();

                if (cmd.cat == ("LD06"))
                {
                    if (cmd.cmd == "PWM")
                    {
                        // LD06:PWM:25
                        lidar06.ChangePWM(cmd.num);
                    }
                    /*
                    cmd.remove(0, 6);
                        int i = atoi(cmd.c_str());
                        println("Lidar: ", i);
                        lidar06.Config(-1, i, -1, -1, -1);
                        // TODO : make a function for reading commands
                        */
                }
                else if (cmd.cat == ("RobotXYA"))
                {
                    /*
                        // RobotXYA:0000,0000,00000
                        int cmdLength = 9;
                        int x = atoi(cmd.substring(cmdLength, cmdLength + 4).c_str());
                        int y = atoi(cmd.substring(cmdLength + 5, cmdLength + 9).c_str());
                        int angle = atoi(cmd.substring(cmdLength + 10, cmdLength + 15).c_str());
                        robot.SetPosition(x, y, angle);
                        println("Robot Position : ", robot.GetPosition());
                        // TODO : make a function for reading commands
                    */
                }
                else if (cmd.cat == ("RobotState"))
                {
                    /*
                        // RobotState:0
                        int cmdLength = 11;
                        int state = atoi(cmd.substring(cmdLength, cmdLength + 1).c_str());
                        robot.dsPicSerial((State)state);

                        // TODO : make a function for reading commands
                    */
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
