/**
 * All data structures used across program
 */

#ifndef STRUCTURE_H
#define STRUCTURE_H

#include <Arduino.h>

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

struct RobotPosition_t
{
    int x;
    int y;
    float angle;
};

struct Obstacle
{
    static constexpr size_t kMaxPoints = 20;
    PolarPoint data[kMaxPoints] = {0, 0, 0, 0, 0};
    int size = 0;
};

#endif