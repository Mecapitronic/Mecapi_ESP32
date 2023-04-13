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
    float angle;
};

struct Obstacle
{
    static constexpr size_t kMaxPoints = 20;
    PolarPoint data[kMaxPoints];
    int size;
};

#endif