#ifndef VL53L5CX_H
#define VL53L5CX_H

#ifdef VL53

#include <Arduino.h>
#include "ESP32_Helper.h"

// #include <Wire.h>

#include <SparkFun_VL53L5CX_Library.h>  //http://librarymanager/All#SparkFun_VL53L5CX

using namespace Printer;

class VL53L5CX
{
   public:
    void Initialisation();
    void Config();
    void Update();

   private:
    SparkFun_VL53L5CX myImager;
    VL53L5CX_ResultsData measurementData;  // Result data class structure, 1356 byes of RAM

    int imageResolution = 0;  // Used to pretty print output
    int imageWidth = 0;       // Used to pretty print output

    long measurements = 0;          // Used to calculate actual output rate
    long measurementStartTime = 0;  // Used to calculate actual output rate
};

#endif
#endif
