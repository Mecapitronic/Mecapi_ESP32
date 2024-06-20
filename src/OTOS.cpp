#include "OTOS.h"
#ifdef SPARKFUN_OTOS

void OpticalTrackingOdometrySensor::Initialisation()
{
    println("Init QwiicOTOS");
    Wire.begin();
    Config();
}

void OpticalTrackingOdometrySensor::Config() {}

void OpticalTrackingOdometrySensor::Update() {}

#endif
