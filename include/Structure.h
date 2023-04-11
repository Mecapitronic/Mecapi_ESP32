#ifndef STRUCTURE_H
#define STRUCTURE_H

#include "main.h"

struct PolarPoint
{
    double angle;
    int distance;
    uint16_t confidence;
    float x;
    float y;

    PolarPoint();
    PolarPoint(double _angle, int _distance, uint16_t _confidence, float _x, float _y);
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

struct Robot_t
{
    int x;
    int y;
    double angle;
};

#endif