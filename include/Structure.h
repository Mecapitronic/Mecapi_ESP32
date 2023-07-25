/**
 * @file Structure.h
 * @author Mecapitronic (mecapitronic@gmail.com)
 * @brief All data structures used across program
 *
 */

#ifndef STRUCTURE_H
#define STRUCTURE_H

#include <Arduino.h>

enum class Level
{
    LEVEL_VERBOSE = 0,
    LEVEL_INFO = 1,
    LEVEL_WARN = 2,
    LEVEL_ERROR = 3
};

/**
 * @brief Represents a Cartesian point, with x and y as coordinates
 */
struct Point
{
    int16_t x;
    int16_t y;
};

/**
 * @brief Represents a 3D Cartesian point, with x, y and z as coordinates
 */
struct Point3D
{
    int16_t x;
    int16_t y;
    int16_t z;
};

/**
 * @brief Represents a 4D point, with x, y and z as Cartesian coordinates, and d as 4th dimension
 */
struct Point4D
{
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t d;
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

#endif
