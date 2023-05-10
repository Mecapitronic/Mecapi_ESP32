/**
 * @file Structure.h
 * @author Mecapitronic (mecapitronic@gmail.com)
 * @brief All data structures used across program
 *
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
struct PointTracker
{
    // point coordinates beeing tracked
    Point point;
    // timestamp of the last time the tracker has been updated
    int64_t lastUpdateTime;
};

struct RobotPosition_t
{
    int x;
    int y;
    float angle;
};

/**
 * Represent an obstacle, the topping cylinder on adversary robots
 * The maximum points needed to represent a 70mm wide cylinder is 20 (kMaxPoints)
 */
struct Obstacle
{
    static constexpr size_t kMaxPoints = 20;
    PolarPoint data[kMaxPoints] = {0, 0, 0, 0, 0};
    uint8_t size = 0;
};

#endif