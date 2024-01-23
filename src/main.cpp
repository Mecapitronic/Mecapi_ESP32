#include "main.h"

void setup()
{
    // put your setup code here, to run once:
    ESP32_Helper::ESP32_Helper();

    // myQueue = xQueueCreate(queueSize, sizeof(PolarPoint));
    myQueue = xQueueCreate(queueSize, sizeof(uint8_t));

    if (myQueue == NULL)
    {
        println("Error creating the queue", LEVEL_ERROR);
    }

#ifdef LD06
    robot.Initialisation();
    delay(500);
    ld06.Initialisation();
    delay(500);
    tracker = Tracker();
    delay(500);
#endif

#ifdef A010
    a010.Initialisation();
    delay(500);
    dbscan.Initialisation();
    delay(500);
#endif

#ifdef VL53
    Serial.end();
    Serial.begin(115200);
    delay(1000);
    Serial.println("SparkFun VL53L5CX Imager Example");

    Wire.begin();            // This resets I2C bus to 100kHz
    Wire.setClock(1000000);  // Sensor has max I2C freq of 1MHz

    // myImager.setWireMaxPacketSize(128);  // Increase default from 32 bytes to 128 - not supported on all platforms

    Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
    // Time how long it takes to transfer firmware to sensor
    long startTime = millis();
    bool startup = myImager.begin();
    long stopTime = millis();

    if (startup == false)
    {
        Serial.println(F("Sensor not found - check your wiring. Freezing"));
        while (1)
            ;
    }

    Serial.print("Firmware transfer time: ");
    float timeTaken = (stopTime - startTime) / 1000.0;
    Serial.print(timeTaken, 3);
    Serial.println("s");

    myImager.setResolution(8 * 8);  // Enable all 64 pads

    imageResolution = myImager.getResolution();  // Query sensor for current resolution - either 4x4 or 8x8
    imageWidth = sqrt(imageResolution);          // Calculate printing width

    // Using 4x4, min frequency is 1Hz and max is 60Hz
    // Using 8x8, min frequency is 1Hz and max is 15Hz
    myImager.setRangingFrequency(2);

    myImager.startRanging();

    measurementStartTime = millis();
#endif

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

    while (1)
    {
        try
        {
#ifdef VL53
            // #define _CSV_FORMAT_
            //  Poll sensor for new data
            if (myImager.isDataReady() == true)
            {
                if (myImager.getRangingData(&measurementData))  // Read distance data into array
                {
                    // The ST library returns the data transposed from zone mapping shown in datasheet
                    // Pretty-print data with increasing y, decreasing x to reflect reality

#ifdef _CSV_FORMAT_
                    for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth)
                    {
                        for (int x = 0; x <= (imageWidth - 1); x++)
                        {
                            Serial.print(measurementData.distance_mm[x + y]);
                            Serial.print(",");
                        }
                    }
#else
                    for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth)
                    {
                        // for (int x = imageWidth - 1; x >= 0; x--)
                        for (int x = 0; x <= (imageWidth - 1); x++)
                        {
                            Serial.print("\t");
                            Serial.print(measurementData.distance_mm[x + y]);
                        }
                        Serial.println();
                    }
#endif
                    Serial.println();

                    // Uncomment to display actual measurement rate
                    // measurements++;
                    // float measurementTime = (millis() - measurementStartTime) / 1000.0;
                    // Serial.print("rate: ");
                    // Serial.print(measurements / measurementTime, 3);
                    // Serial.println("Hz");
                }
            }

            delay(5);  // Small delay between polling
#endif

#ifdef LD06
            if (robot.ReadSerial())
            {
                // set the new robot position
                robot.Analyze();
                plotRobot(robot.GetPosition());
            }
            if (ld06.ReadSerial())
            {
                ld06.CheckContinuity();

                int counter = 0;
                for (int i = 0; i < LIDAR_DATA_PACKET_SIZE; i++)
                {
                    ld06.AggregatePoint(ld06.GetData().dataPoint[i], &tracker, robot);
                }
                plotScanXY(ld06.scan);
                ld06.scan.clear();
                Debugger::WaitForAvailableSteps();

                tracker.sendObstaclesToRobot(robot);
                tracker.untrackOldObstacles(robot);
            }
#endif

#ifdef A010
            // we enter once we have the complete frame
            if (a010.ReadSerial())
            {
                // a010.CheckContinuity();

                println("***");

                // ! FIXME: n'affiche plus rien
                for (uint16_t i = 0; i < PICTURE_SIZE; i++)
                {
                    String data = "" + String(a010.cloudFrame[i].x) + " " +
                                  String(a010.cloudFrame[i].y) + " " +
                                  String(a010.cloudFrame[i].z) + " " + String("65520");
                    println(data);
                }
                println("---");
                println();

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
#endif
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
                    // ld06.AggregatePoint(point, &tracker, robot);
                }
            }
            // tracker.sendObstaclesToRobot(robot);
            // tracker.untrackOldObstacles(robot);

            // Check if we get commands from operator via debug serial
            ESP32_Helper::UpdateSerial();

            if (ESP32_Helper::HasWaitingCommand())
            {
                Command cmd = ESP32_Helper::GetCommand();

                if (cmd.cmd.startsWith("VL53"))
                {
                    println("VL53 : ", cmd.size, "");
                }
#ifdef LD06
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
#endif
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
