/**
 * @file Structure.h
 * @author Mecapitronic (mecapitronic@gmail.com)
 * @brief All data structures used across program
 *
 */

#ifndef STRUCTURE_H
#define STRUCTURE_H

#include <Arduino.h>

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

#endif