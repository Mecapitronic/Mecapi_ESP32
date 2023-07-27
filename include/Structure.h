/**
 * @file Structure.h
 * @author Mecapitronic (mecapitronic@gmail.com)
 * @brief All data structures used across program
 * @date 2023-07-25
 */

#ifndef STRUCTURE_H
#define STRUCTURE_H

#include <Arduino.h>

#define ENUM_STRING(p) #p

enum Level
{
    LEVEL_VERBOSE,
    LEVEL_INFO,
    LEVEL_WARN,
    LEVEL_ERROR,
    LEVEL_NONE
};

enum Enable
{
    ENABLE_FALSE,
    ENABLE_TRUE,
    ENABLE_NONE
};

/**
 * @brief Represents a Cartesian point with x and y as coordinates
 * @param x int16_t
 * @param y int16_t
 */
struct Point
{
    int16_t x;
    int16_t y;
    /**
     * @brief Construct a new Point object
     * @param _x int16_t
     * @param _y int16_t
     */
    Point(int16_t _x, int16_t _y)
    {
        x = _x;
        y = _y;
    }
    /**
     * @brief Construct a new Point object
     *
     */
    Point()
    {
        x = 0;
        y = 0;
    }
};

/**
 * @brief Represents a Cartesian point with x and y as coordinates
 * @param x float
 * @param y float
 */
struct PointF
{
    float x;
    float y;
    /**
     * @brief Construct a new Point F object
     *
     * @param _x float
     * @param _y float
     */
    PointF(float _x, float _y)
    {
        x = _x;
        y = _y;
    }
    /**
     * @brief Construct a new Point F object
     *
     */
    PointF()
    {
        x = 0;
        y = 0;
    }
};

/**
 * @brief Represents a 3D Cartesian point with x, y and z as coordinates
 * @param x int16_t
 * @param y int16_t
 * @param z int16_t
 */
struct Point3D
{
    int16_t x;
    int16_t y;
    int16_t z;
    /**
     * @brief Construct a new Point3D object
     *
     * @param _x int16_t
     * @param _y int16_t
     * @param _z int16_t
     */
    Point3D(int16_t _x, int16_t _y, int16_t _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }
    /**
     * @brief Construct a new Point3D object
     */
    Point3D()
    {
        x = 0;
        y = 0;
        z = 0;
    }
};

/**
 * @brief Represents a 3D Cartesian point with x, y and z as coordinates
 * @param x float
 * @param y float
 * @param z float
 */
struct PointF3D
{
    float x;
    float y;
    float z;
    /**
     * @brief Construct a new Point3D object
     *
     * @param _x float
     * @param _y float
     * @param _z float
     */
    PointF3D(float _x, float _y, float _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }
    /**
     * @brief Construct a new Point3D object
     */
    PointF3D()
    {
        x = 0;
        y = 0;
        z = 0;
    }
};

/**
 * @brief Represents a 4D point with x, y and z as Cartesian coordinates, and w as 4th dimension
 * @param x int16_t
 * @param y int16_t
 * @param z int16_t
 * @param w int16_t
 */
struct Point4D
{
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t w;
    /**
     * @brief Construct a new Point4D object
     *
     * @param _x int16_t
     * @param _y int16_t
     * @param _z int16_t
     * @param _w int16_t
     */
    Point4D(int16_t _x, int16_t _y, int16_t _z, int16_t _w)
    {
        x = _x;
        y = _y;
        z = _z;
        w = _w;
    } /**
       * @brief Construct a new Point4D object
       */
    Point4D()
    {
        x = 0;
        y = 0;
        z = 0;
        w = 0;
    }
};

/**
 * @brief Represents a 4D point with x, y and z as Cartesian coordinates, and w as 4th dimension
 * @param x float
 * @param y float
 * @param z float
 * @param w float
 */
struct PointF4D
{
    float x;
    float y;
    float z;
    float w;
    /**
     * @brief Construct a new Point4D object
     *
     * @param _x float
     * @param _y float
     * @param _z float
     * @param _w float
     */
    PointF4D(float _x, float _y, float _z, float _w)
    {
        x = _x;
        y = _y;
        z = _z;
        w = _w;
    }
    /**
     * @brief Construct a new Point4D object
     */
    PointF4D()
    {
        x = 0;
        y = 0;
        z = 0;
        w = 0;
    }
};

/**
 * @brief Represents a Polar point
 *  with angle, distance, Cartesian coordinates, and confidence
 * @param angle float Degree
 * @param distance int16_t mm
 * @param confidence uint16_t
 * @param x float
 * @param y float
 *
 */
struct PolarPoint
{
    float angle;
    int16_t distance;
    uint16_t confidence;
    float x;
    float y;
    /**
     *
     * @brief Construct a new Polar Point object
     *
     * @param _angle
     * @param _distance
     * @param _confidence
     */
    PolarPoint(float _angle, int16_t _distance, uint16_t _confidence)
    {
        angle = _angle;
        distance = _distance;
        confidence = _confidence;
        x = 0;
        y = 0;
    }
    /**
     * @brief Construct a new Polar Point object
     *
     * @param _x
     * @param _y
     */
    PolarPoint(float _x, float _y)
    {
        angle = 0;
        distance = 0;
        confidence = 0;
        x = _x;
        y = _y;
    }
    /**
     * @brief Construct a new Polar Point object
     * @param _angle
     * @param _distance
     * @param _confidence
     * @param _x
     * @param _y
     */
    PolarPoint(float _angle, int16_t _distance, uint16_t _confidence, float _x, float _y)
    {
        angle = _angle;
        distance = _distance;
        confidence = _confidence;
        x = _x;
        y = _y;
    }
    /**
     * @brief Construct a new Polar Point object
     */
    PolarPoint()
    {
        angle = 0;
        distance = 0;
        confidence = 0;
        x = 0;
        y = 0;
    }
};

#endif
