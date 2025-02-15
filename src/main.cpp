#include "main.h"

#ifdef LD06
LidarLD06 ld06;
Robot robot;
Tracker tracker;
testModule test;

int64_t lastSendRobotTime = millis();
PolarPoint lastPosition = {0, 0, 0, 0, 0};
PolarPoint lastTrackerSend[5] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};
int64_t lastSendSerialTime = millis();

#endif

#ifdef A010
MetaSenseA010 a010;
// Dbscan dbscan;
#endif

#ifdef VL53
VL53L5CX vl53;
#endif

#ifdef SPARKFUN_OTOS
OpticalTrackingOdometrySensor otos;
#endif

PolarPoint MapBoundaries[] = {{0, 0}, {0, 2000}, {3000, 2000}, {3000, 0}};

void setup()
{
    ESP32_Helper::Initialisation();

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

#ifdef SPARKFUN_OTOS
    otos.Initialisation();
#endif
    // functionChrono();
}

void loop()
{
    try
    {
#ifdef LD06
        robot.Update();

        ld06.SetRobotPosition(robot.position);
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

#ifdef SPARKFUN_OTOS
        otos.Update();
#endif

#ifdef LD06
        if (millis() - lastSendSerialTime > 100)
        {
            lastSendSerialTime = millis();
            tracker.Teleplot(false);

            if ((int)lastPosition.x != (int)robot.position.x || (int)lastPosition.y != (int)robot.position.y ||
                (int)(lastPosition.angle / 100) != (int)(robot.position.angle / 100))
            {
                teleplot("robot", robot.position, robot.position.angle, LEVEL_WARN);
                lastPosition = robot.position;
            }
            // teleplot("mapBoundaries", MapBoundaries, 4, LEVEL_WARN);
            // teleplot("robot", robot.position, LEVEL_WARN);
        }
#endif

        if (ESP32_Helper::HasWaitingCommand())
        {
            Command cmd = ESP32_Helper::GetCommand();

#ifdef VL53

            vl53.HandleCommand(cmd);
#endif

#ifdef LD06
            ld06.HandleCommand(cmd);
            robot.HandleCommand(cmd);
            tracker.HandleCommand(cmd);
            if (cmd.cmd == ("MapBoundaries"))
            {
                teleplot("mapBoundaries", MapBoundaries, 4, LEVEL_WARN);
            }
#endif

#ifdef A010
            a010.HandleCommand(cmd);
#endif

#ifdef SPARKFUN_OTOS
            otos.HandleCommand(cmd);
#endif
        }
    }
    catch (std::exception const &e)
    {
        print("error : ", LEVEL_ERROR);
        println(e.what(), LEVEL_ERROR);
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
