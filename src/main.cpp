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

#ifdef AX12
    int8_t index = 2;
    int8_t found_dynamixel = 0;
    int8_t protocol = 1;
    // for (int8_t protocol = 1; protocol < 3; protocol++)
    {
        // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
        dxl.setPortProtocolVersion((float)protocol);
        print("SCAN PROTOCOL ");
        println(protocol);

        // for (index = 0; index < MAX_BAUD; index++)
        {
            // Set Port baudrate.
            print("SCAN BAUDRATE ");
            println(baud[index]);
            dxl.begin(baud[index]);
            for (int id = 0; id < DXL_BROADCAST_ID; id++)
            {
                // iterate until all ID in each baudrate is scanned.
                if (dxl.ping(id))
                {
                    print("ID : ");
                    print(id);
                    print(", Model Number: ");
                    println(dxl.getModelNumber(id));
                    found_dynamixel++;
                }
            }
        }
    }

    print("Total ");
    print(found_dynamixel);
    println(" DYNAMIXEL(s) found!");

    // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
    dxl.begin(1000000);
    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
    dxl.setPortProtocolVersion(1.0);
    // Get DYNAMIXEL information
    dxl.ping(DXL_ID);

    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_POSITION);
    dxl.torqueOn(DXL_ID);

    // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, DXL_ID, 30);
    dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, DXL_ID, 50);
    dxl.ledOn(DXL_ID);
#endif

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
void loop()
{
    TaskThread::DeleteTask(NULL);
}

// Note the 1 Tick delay, this is need so the watchdog doesn't get confused
void TaskSerial(void *pvParameters)
{
    Serial.println("Start TaskSerial1");
    Timeout toSendRobot;
    toSendRobot.Start(200);
    while (1)
    {
#ifdef AX12
        // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/)
        dxl.setGoalPosition(DXL_ID, 0, UNIT_DEGREE);

        static int i_present_position = 0;
        static float f_present_position = 0.0;

        // Check if DYNAMIXEL is in motion
        while (abs(0 - f_present_position) > 2)
        {
            i_present_position = dxl.getPresentPosition(DXL_ID);
            println("Position(raw) : ", i_present_position);
            f_present_position = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
            println("Position(degree) : ", f_present_position);
            // println("Velocity : ", dxl.getPresentVelocity(DXL_ID, UNIT_RPM));
            // println("Current : ", dxl.getPresentCurrent(DXL_ID, UNIT_MILLI_AMPERE));
            // println("PWM : ", dxl.getPresentPWM(DXL_ID));
        }
        delay(1000);

        // Set Goal Position in DEGREE value
        dxl.setGoalPosition(DXL_ID, 180, UNIT_DEGREE);

        while (abs(180.0 - f_present_position) > 2.0)
        {
            i_present_position = dxl.getPresentPosition(DXL_ID);
            println("Position(raw) : ", i_present_position);
            f_present_position = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
            println("Position(degree) : ", f_present_position);
            // println("velocity : ", dxl.getPresentVelocity(DXL_ID, UNIT_RPM));
            // println("current : ", dxl.getPresentCurrent(DXL_ID, UNIT_MILLI_AMPERE));
            // println("PWM : ", dxl.getPresentPWM(DXL_ID));
        }
        delay(1000);
#endif

#ifdef LD06
        robot.Update();
        ld06.SetRobotPosition(robot.GetPosition());

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
    while (1)
    {
#ifdef LD06
        // ld06.lidarPacket.Print();
        // println("Step : ", float(ld06.lidarPacket.dataPoint[0].angle - ld06.lidarPacket.dataPoint[1].angle) / 100);
        tracker.Teleplot(false);
        PoseF p = robot.GetPosition();
        teleplot("LD06Pos", p);
        teleplot("LD06Orient", p.h);
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
#ifdef LD06
            ld06.HandleCommand(cmd);
            robot.HandleCommand(cmd);
            tracker.HandleCommand(cmd);
            if (cmd.cmd == ("MapBoundaries"))
            {
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
        vTaskDelay(100);  // let other task to run
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
