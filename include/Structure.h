/**
 * @file Structure.h
 * @author Mecapitronic (mecapitronic@gmail.com)
 * @brief All data structures used across program
 *
 */

#ifndef STRUCTURE_H
#define STRUCTURE_H

#include <Arduino.h>

#define _kMaxPoints 25

struct PolarPoint
{
    double angle;
    int distance;
    uint16_t confidence;
    float x;
    float y;
};

/**
 * Represent a Cartesian point, with x and y as coordinates
 */
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
    // Polar points that define the mid point
    PolarPoint data[_kMaxPoints] = {0, 0, 0, 0, 0};
    // Size of the polar points array
    uint8_t size = 0;
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
    static constexpr size_t kMaxPoints = _kMaxPoints;
    PolarPoint data[kMaxPoints] = {0, 0, 0, 0, 0};
    uint8_t size = 0;
};

#endif