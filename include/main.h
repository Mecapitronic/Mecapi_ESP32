#ifndef MAIN_H
#define MAIN_H

#include "ESP32_Helper.h"
using namespace Printer;

#ifdef LD06
#warning "! Compiling for LD06 !"
#include "LD06.h"
#include "Robot.h"
#include "Tracker.h"
#include "testModule.h"
extern Robot robot;
#endif

#ifdef A010
#warning "! Compiling for MetaSenseA010 !"
#include "A010.h"
// #include "DBSCAN.h"
#endif

#ifdef VL53
#warning "! Compiling for VL53L5CX !"
#include "VL53L5CX.h"
#endif

#ifdef SPARKFUN_OTOS
#warning "! Compiling for QwiicOTOS !"
#include "OTOS.h"
#endif

//void functionChrono(int nbrLoop = 1);
#endif
