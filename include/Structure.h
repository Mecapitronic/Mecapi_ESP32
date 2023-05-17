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

enum State
{
    Stop = 0,
    Start = 1,
    Debug = 2
};

/**
 * @brief Represents a Cartesian point, with x and y as coordinates
 */
struct Point
{
    float x;
    float y;
};

/**
 * @brief Represents a Polar point, with angle and distance as coordinates
 * Has confidence and cartesian coordinates too
 *
 */
struct PolarPoint
{
    double angle;
    int distance;
    uint16_t confidence;
    float x;
    float y;
};

/**
 * @brief Represents a Polar point with confidence from Lidar, with angle and distance as coordinates
 *
 */
struct PointLidar
{
    int angle;
    int distance;
    uint16_t confidence;
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
    // avoid to send the same points twice
    bool hasBeenSent = false;
};

struct RobotPosition
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