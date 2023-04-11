#ifndef ROBOT_H
#define ROBOT_H

#include "main.h"

namespace Robot
{

void Init();
boolean ReadSerial();
void Analyze();
Robot_t GetRobot();
void Print();

}  // namespace Robot

#endif