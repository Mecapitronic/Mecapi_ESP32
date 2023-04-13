#ifndef ROBOT_H
#define ROBOT_H

#include "main.h"

namespace Robot
{

uint8_t const serial_packet_size = 32;
// '!' + "2000,2000,36000" + 'n'
uint8_t const data_packet_size = 1 + 2 * 3 + 1;

void Init();
boolean ReadSerial();
void Analyze();
Robot_t GetData();
void Print();
void WriteSerial(int n, int x, int y);

}  // namespace Robot

#endif