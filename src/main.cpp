#include "main.h"

LD06 ld06;

void setup()
{
    // put your setup code here, to run once:
    delay(500);
    SERIAL_PC.begin(230400);
    SERIAL_PC.print("Start PC ! ");
    ld06.Init();
    SERIAL_PC.print("Start Lidar ! ");
}

void loop()
{
    // put your main code here, to run repeatedly:
    ld06.Read_lidar_data();
    ld06.Calc_lidar_data();
    ld06.Filter_lidar_data();
}
