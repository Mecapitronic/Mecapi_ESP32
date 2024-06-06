#include "main.h"

#ifdef LD06
LidarLD06 ld06;
Robot robot;
Tracker tracker;
testModule test;
#endif

#ifdef A010
MetaSenseA010 a010;
// Dbscan dbscan;
#endif

PolarPoint MapBoundaries[] = {{0, 0}, {0, 2000}, {3000, 2000}, {3000, 0}};

void setup()
{
    Serial.end();
    Serial.begin(921600);
    delay(1000);
    Serial.println();
    Serial.println("ESP32 Firmware");
    // put your setup code here, to run once:
    ESP32_Helper::ESP32_Helper();

    Printer::PrintLevel(LEVEL_VERBOSE);

#ifdef LD06
    robot.Initialisation();
    delay(500);
    ld06.Initialisation();
    delay(500);
    tracker.Initialisation();
    delay(500);
#endif

#ifdef A010
    a010.Initialisation();
    delay(500);
    // dbscan.Initialisation();
    // delay(500);
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

    Serial.println("Creating Tasks");
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
    //  Do not put code in the loop() when using freeRTOS.
    //  loop() is the only task that is guaranteed to not be ran per tasking iteration.
    //  delay(1000);
#ifdef A010
    a010.Update();
#endif
}

void functionChrono(int nbrLoop)
{
    unsigned long startChrono = micros();
    for (int i = 0; i < nbrLoop; i++)
    {
        // function or code to loop
    }
    unsigned long endChrono = micros();
    unsigned long deltaChrono = endChrono - startChrono;

    unsigned long chrono = deltaChrono / nbrLoop;
    Serial.print("Chrono from ");
    Serial.print(nbrLoop);
    Serial.print(" loop is : ");
    Serial.print(deltaChrono);
    Serial.print(" µs total or ");
    Serial.print(deltaChrono / 1000);
    Serial.print(" ms total.    ");
    Serial.print(chrono);
    Serial.print(" µs/func or ");
    Serial.print(chrono / 1000);
    Serial.print(" ms/func.");
    Serial.println();
}

#ifdef LD06
int64_t lastSendRobotTime = millis();
PolarPoint lastPosition = {0, 0, 0, 0, 0};
PolarPoint lastTrackerSend[5] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};
#endif

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void Task1code(void *pvParameters)
{
    Serial.println("Start Task1code");

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
            robot.Update();

            ld06.SetRobotPosition(robot.GetPosition());
            ld06.Update();
            // if (ld06.scan.size() > 0)
            //     teleplot("scan", ld06.scan, LEVEL_WARN);
            ld06.scan.clear();

            tracker.Track(ld06.clusterCenterPoints);
            tracker.Update();

            if (millis() - lastSendRobotTime > 200)
            {
                lastSendRobotTime = millis();
                tracker.SendToRobot();
            }
#endif

#ifdef A010

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

#endif
        }
        catch (std::exception const &e)
        {
            print("error : ", LEVEL_ERROR);
            println(e.what(), LEVEL_ERROR);
        }
        vTaskDelay(1);
    }
}

#ifdef WITH_WIFI
unsigned long previousMillisWifi = 0;
unsigned long previousMillisServer = 0;
unsigned long intervalWifi = 5000;
unsigned long intervalServer = 5000;
unsigned long currentMillisWifi = 0;
unsigned long currentMillisServer = 0;
#endif

int64_t lastSendSerialTime = millis();

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void Task2code(void *pvParameters)
{
    println("Start Task2code");

    PolarPoint point;
    while (1)
    {
        try
        {
            // Check if we get commands from operator via debug serial
            ESP32_Helper::UpdateSerial();

#ifdef LD06
            if (millis() - lastSendSerialTime > 100)
            {
                lastSendSerialTime = millis();
                tracker.Teleplot(false);

                if ((int)lastPosition.x != (int)robot.GetPosition().x ||
                    (int)lastPosition.y != (int)robot.GetPosition().y ||
                    (int)(lastPosition.angle / 100) != (int)(robot.GetPosition().angle / 100))
                {
                    teleplot("robot", robot.GetPosition(), robot.GetPosition().angle, LEVEL_WARN);
                    lastPosition = robot.GetPosition();
                }
                // teleplot("mapBoundaries", MapBoundaries, 4, LEVEL_WARN);
                // teleplot("robot", robot.GetPosition(), LEVEL_WARN);
            }
#endif

#ifdef WITH_WIFI
            currentMillisWifi = millis();
            // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
            if ((WiFi.status() != WL_CONNECTED) && (currentMillisWifi - previousMillisWifi >= intervalWifi))
            {
                Serial.println("Reconnecting to WiFi...");
                // WiFi.disconnect();
                WiFi.reconnect();
                previousMillisWifi = currentMillisWifi;
            }

            currentMillisServer = millis();
            if (WiFi.status() == WL_CONNECTED)
            {
                if (!client.connected() && (currentMillisServer - previousMillisServer >= intervalServer))
                {
                    client.stop();
                    if (client.connect("192.168.137.1", 20240))
                    {
                        Serial.println("Connected to server !");
                    }
                    else
                    {
                        Serial.println("Connection to server failed");
                    }
                    previousMillisServer = currentMillisServer;
                }
            }
#endif

            if (ESP32_Helper::HasWaitingCommand())
            {
                Command cmd = ESP32_Helper::GetCommand();

#ifdef VL53
                if (cmd.cmd.startsWith("VL53"))
                {
                    println("VL53 : ", cmd.size, "");
                }
#endif

#ifdef LD06
                if (cmd.cmd == ("LD06PWM"))
                {
                    // LD06PWM:25
                    ld06.ChangePWM(cmd.data[0]);
                    println("LidarLD06 Change PWM : ", ld06.GetPWM(), "");
                }
                else if (cmd.cmd == ("RobotXYA"))
                {
                    // RobotXYA:1000;1500;00000
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
                else if (cmd.cmd == ("RobotPosition"))
                {
                    teleplot("robot", robot.GetPosition(), robot.GetPosition().angle, LEVEL_WARN);
                }
                else if (cmd.cmd == ("MapBoundaries"))
                {
                    teleplot("mapBoundaries", MapBoundaries, 4, LEVEL_WARN);
                }
                else if (cmd.cmd == ("Tracker"))
                {
                    tracker.Teleplot(true);
                }
#endif

#ifdef A010
                if (cmd.cmd.startsWith("A010"))
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
            print("error : ", LEVEL_ERROR);
            println(e.what(), LEVEL_ERROR);
        }
        vTaskDelay(1);
    }
}
