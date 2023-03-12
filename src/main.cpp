#include "main.h"

// LD06Driver Lidar;

LD06 ld06;

void setup()
{
    // put your setup code here, to run once:
    delay(500);
    SERIAL_PC.begin(230400);
    SERIAL_PC.print("Start PC ! ");
    ld06.Init();
    SERIAL_PC.print("Start Lidar ! ");

    // Lidar.Initialisation();
}

void loop()
{
    // put your main code here, to run repeatedly:
    // Serial.println("data start");
    ld06.Read_lidar_data();

    // Lidar.ReadLidarData();
}
