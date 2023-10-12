#include "main.h"

void setup()
{
    // put your setup code here, to run once:
    ESP32_Helper::ESP32_Helper(921600);

    // myQueue = xQueueCreate(queueSize, sizeof(PolarPoint));
    myQueue = xQueueCreate(queueSize, sizeof(uint8_t));

    if (myQueue == NULL)
    {
        println("Error creating the queue", LEVEL_ERROR);
    }

    robot.Initialisation();
    delay(500);
    ld06.Initialisation();
    delay(500);
    tracker = Tracker();
    delay(500);

    a010.Initialisation();
    delay(500);

    println("Dbscan setup");
    dbscan.Config(60.0f, 3, TCHEBYCHEV);
    delay(500);

    /* Task function. */
    /* name of task. */
    /* Stack size of task */
    /* parameter of the task */
    /* priority of the task */
    /* Task handle to keep track of created task */
    /* pin task to core 0 */
    xTaskCreatePinnedToCore(Task1code, "Task1", 20000, NULL, 10, &Task1, 0);
    xTaskCreatePinnedToCore(Task2code, "Task2", 20000, NULL, 5, &Task2, 1);
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

            if (ld06.ReadSerial())
            {
                ld06.Analyze();
                ld06.CheckContinuity();

                lidarPacket = ld06.GetData();
                int counter = 0;
                for (int i = 0; i < LIDAR_DATA_PACKET_SIZE; i++)
                {
                    // Filter point before sending to queue : increase speed for later
                    // calculus
                    // TODO increase the confidence limit to avoid aberrations

                    // if the point is out of bound, we will not use it
                    ConfigLidar configLidar = ld06.GetConfig();
                    if (lidarPacket.dataPoint[i].distance < configLidar.minDistance ||
                        lidarPacket.dataPoint[i].distance > configLidar.maxDistance ||
                        lidarPacket.dataPoint[i].confidence < configLidar.minQuality)
                    {
                        counter++;
                    }
                    else
                    {
                        xQueueSend(myQueue, &lidarPacket.dataPoint[i], 0);
                    }
                }
                // TODO at least send 1 or 2 points to the queue (min max ?, middle ?) to
                // end aggregation for obstacle
                if (counter == LIDAR_DATA_PACKET_SIZE)
                {
                    // we did not have any point to send, we send at least the last one.
                    xQueueSend(myQueue,
                               &lidarPacket.dataPoint[LIDAR_DATA_PACKET_SIZE - 1], 0);
                    print("No point to send, Sending dull point",
                          lidarPacket.dataPoint[LIDAR_DATA_PACKET_SIZE - 1]);
                }
                else
                {
                    println("Sending ", LIDAR_DATA_PACKET_SIZE - counter, " point");
                }
            }

            // we enter once we have the complete frame
            if (a010.ReadSerial())
            {
                // a010.CheckContinuity();

                println("***");

                // for (uint16_t i = 0; i < PICTURE_SIZE; i++)  // FIXME: n'affiche plus
                // rien !
                //{
                //  String data = "" + String(a010.cloudFrame[i].x) + " " +
                //  String(a010.cloudFrame[i].y) + " " + String(a010.cloudFrame[i].z) + "
                //  " + String("65520"); println(data);
                //}
                // println("---");
                // println();

                // Erasing all previous _clusters

                /*for (size_t i = 0; i < _clusters.size(); i++)
                {
                    _clusters[i].clear();
                }
                _clusters.clear();
                println("Dbscan Process");*/

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
                    ld06.AggregatePoint(point, &tracker, robot);
                }
            }
            tracker.sendObstaclesToRobot(robot);
            tracker.untrackOldObstacles(robot);

            // Check if we get commands from operator via debug serial
            ESP32_Helper::UpdateSerial();

            if (ESP32_Helper::HasWaitingCommand())
            {
                Command cmd = ESP32_Helper::GetCommand();

                if (cmd.cmd == ("LD06PWM"))
                {
                    // LD06PWM:25
                    ld06.ChangePWM(cmd.data[0]);
                    println("LidarLD06 Change PWM : ", ld06.GetPWM(), "");
                }
                else if (cmd.cmd == ("RobotXYA"))
                {
                    // RobotXYA:0000;0000;00000
                    robot.SetPosition(cmd.data[0], cmd.data[1], cmd.data[2]);
                    print("Robot Position : ", robot.GetPosition());
                }
                else if (cmd.cmd == ("RobotState"))
                {
                    /*
                        // RobotState:0
                        int cmdLength = 11;
                        int state = atoi(cmd.substring(cmdLength, cmdLength + 1).c_str());
                        robot.dsPicSerial((State)state);

                        // TODO : make a function for reading commands
                    */
                }
                else if (cmd.cmd.startsWith("A010"))
                {
                    // a010.HandleCommand(cmd);
                    //  String s = cmd.cmd;
                    //  s.remove(0, 4);
                    //  SERIAL_A010.write(s.c_str());
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
