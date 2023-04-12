#ifndef STRUCTURE_H
#define STRUCTURE_H

#include "main.h"

// 3 is the minimum number of points to detect a 7cm wide target @ 3m distance
#define OBS_MIN_POINT 3

// at 200mm distance, to detect an object of 70mm, it represent 20°, with 0.8° each point, we need 25 points
#define OBS_MAX_POINT 25

struct PolarPoint
{
    double angle;
    int distance;
    uint16_t confidence;
    float x;
    float y;
};

struct Point
{
    float x;
    float y;
};

struct Robot_t
{
    int x;
    int y;
    double angle;
};

struct Obstacle
{
    PolarPoint data[OBS_MAX_POINT];
    int size;
};

#endif