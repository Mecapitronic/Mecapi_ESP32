/**
 * All data structures used across program
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
 * Represent a cartesian point, with x and y as coordinates
 */
struct Point
{
    float x;
    float y;
};
struct TrackPoint
{
    Point point;
    static constexpr size_t kMaxPoints = _kMaxPoints;
    PolarPoint data[kMaxPoints] = {0, 0, 0, 0, 0};
    uint8_t size;
    bool isNew;
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