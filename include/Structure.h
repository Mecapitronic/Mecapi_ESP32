#ifndef STRUCTURE_H
#define STRUCTURE_H

#include "main.h"

struct PolarPoint
{
    float angle;
    uint16_t distance;
    uint16_t confidence;
    float x;
    float y;

    PolarPoint();
    PolarPoint(float _angle, u_int16_t _distance, uint16_t _confidence);
    void Print(HardwareSerial s, PolarPoint p, bool debug = false);
};

struct Point
{
    float x;
    float y;
    Point();
    Point(float _x, float _y);
    void Print(HardwareSerial s, Point p, bool debug = false);
};

#endif