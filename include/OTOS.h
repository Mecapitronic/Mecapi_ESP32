#ifndef OTOS_H
#define OTOS_H
#ifdef SPARKFUN_OTOS

#include <Arduino.h>
#include "ESP32_Helper.h"

#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <Wire.h>

using namespace Printer;

class OpticalTrackingOdometrySensor
{
   public:
    void Initialisation();
    void Config();
    void Update();

   private:
    // SparkFun_
};

#endif
#endif
