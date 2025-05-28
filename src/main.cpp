#include "main.h"

#ifdef LD06
LidarLD06 ld06;
Robot robot;
Tracker tracker;
// testModule test;

PoseF lastPosition = {0.0, 0.0, 0.0};
PolarPoint lastTrackerSend[5] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};
Point MapBoundaries[] = {{0, 0}, {0, 2000}, {3000, 2000}, {3000, 0}};
#endif

#ifdef A010
MetaSenseA010 a010;
// Dbscan dbscan;
#endif

#ifdef VL53
VL53L5CX vl53;
#endif

// we could make them not global and only in setup
TaskThread Task1;
TaskThread Task2;
TaskThread Task3;

void setup()
{
    ESP32_Helper::Initialisation();
    Wifi_Helper::SetLocalIP("192.168.137.111");

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
    vl53.Initialisation();
#endif

    println("Creating Tasks");
    Task1 = TaskThread(TaskSerial, "TaskSerial", 20000, 1, 1);
    Task2 = TaskThread(TaskTeleplot, "TaskTeleplot", 20000, 10, 0);
    Task3 = TaskThread(TaskCommand, "TaskCommand", 20000, 5, 0);
}

// task running on core 1
void loop() { TaskThread::DeleteTask(NULL); }

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void TaskSerial(void *pvParameters)
{
    Serial.println("Start TaskSerial1");
    Timeout toSendRobot;
    toSendRobot.Start(50);
    while (1)
    {
#ifdef LD06
        robot.Update();
        ld06.robotPosition = robot.position;

        ld06.clusterCenterPoints.clear();
        ld06.Update();

        tracker.Track(ld06.clusterCenterPoints);
        tracker.Update();

        if (toSendRobot.IsTimeOut())
        {
            tracker.SendToRobot();
        }
#endif

#ifdef A010
        a010.Update();

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

#ifdef VL53
        vl53.Update();
#endif
        vTaskDelay(1);  // smallest 1 Tick delay
    }
}

void TaskTeleplot(void *pvParameters)
{
    Serial.println("Start TaskTeleplot");
    // Variables pour stocker les dernières valeurs envoyées
    static PoseF lastSentPos = {0.0, 0.0, 0.0};
    while (1)
    {
#ifdef LD06
        // ld06.lidarPacket.Print();
        // println("Step : ", float(ld06.lidarPacket.dataPoint[0].angle - ld06.lidarPacket.dataPoint[1].angle) / 100);
        tracker.Teleplot(false);

        // N'envoyer que si la position a changé
        if (robot.position.x != lastSentPos.x || robot.position.y != lastSentPos.y || robot.position.h != lastSentPos.h)
        {
            teleplot("LD06Pos", robot.position);
            teleplot("LD06Orient", robot.position.h / 100);
            lastSentPos = robot.position;
        }
        // teleplot("LD06Obstacle", ld06.clusterCenterPoints.size());
#endif
        vTaskDelay(500);  // let other task to run
    }
}

void TaskCommand(void *pvParameters)
{
    Serial.println("Start TaskCommand");
    while (1)
    {
        if (ESP32_Helper::HasWaitingCommand())
        {
            Command cmd = ESP32_Helper::GetCommand();
            if (cmd.cmd == "Com" && cmd.size == 1)
            {
                if (cmd.data[0] == 1)
                {
                    Printer::EnablePrinter(Enable::ENABLE_TRUE);
                    Wifi_Helper::EnableWifi(Enable::ENABLE_TRUE);
                    Printer::teleplotUDPEnable = Enable::ENABLE_TRUE;
                    println("Enable Com");
                }
                else
                {
                    println("Disable Com");
                    Printer::EnablePrinter(Enable::ENABLE_FALSE);
                    Wifi_Helper::EnableWifi(Enable::ENABLE_FALSE);
                    Printer::teleplotUDPEnable = Enable::ENABLE_FALSE;
                }
                return;
            }
#ifdef LD06
            ld06.HandleCommand(cmd);
            robot.HandleCommand(cmd);
            tracker.HandleCommand(cmd);
            if (cmd.cmd == ("MapBoundaries"))
            {
                println("Teleplot MapBoundaries");
                teleplot("mapBoundaries", MapBoundaries[0]);
                teleplot("mapBoundaries", MapBoundaries[1]);
                teleplot("mapBoundaries", MapBoundaries[2]);
                teleplot("mapBoundaries", MapBoundaries[3]);
            }
#endif

#ifdef A010
            a010.HandleCommand(cmd);
#endif

#ifdef VL53
            vl53.HandleCommand(cmd);
#endif
        }
        vTaskDelay(10);  // let other task to run
    }
}

void functionChrono(int nbrLoop)
{
    unsigned long startChrono = micros();
    for (int i = 0; i < nbrLoop; i++)
    {
        // function or code to loop
        // loop();
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
