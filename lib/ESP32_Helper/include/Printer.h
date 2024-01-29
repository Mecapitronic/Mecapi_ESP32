/**
 * @file Printer.h
 * @author Mecapitronic (mecapitronic@gmail.com)
 * @brief Used to print data over predefined serial for debug purpose
 * @date 2023-07-25
 */
#ifndef PRINTER_H
#define PRINTER_H

#include "ESP32_Helper.h"

#define ENUM_PRINT(p)             \
    case (p):                     \
        SERIAL_DEBUG.println(#p); \
        break;

namespace Printer
{
    // static bool printEnable = false;
    // Level printLevel = LEVEL_NONE;

    void PrintLevel(Level level);
    Level PrintLevel();
    void PrintEnable(Enable enable);
    Enable PrintEnable();

    bool IsPrintable();

    void println(Level level = LEVEL_VERBOSE);
    void print(String prefix, String suffix = "", Level level = LEVEL_VERBOSE);
    void println(String prefix, String suffix = "", Level level = LEVEL_VERBOSE);
    void print(String prefix, int data, String suffix = "", Level level = LEVEL_VERBOSE);
    void println(String prefix, int data, String suffix = "", Level level = LEVEL_VERBOSE);
    void print(String prefix, uint data, String suffix = "", Level level = LEVEL_VERBOSE);
    void println(String prefix, uint data, String suffix = "", Level level = LEVEL_VERBOSE);
    void print(String prefix, char data, String suffix = "", Level level = LEVEL_VERBOSE);
    void println(String prefix, char data, String suffix = "", Level level = LEVEL_VERBOSE);
    void print(String prefix, float data, String suffix = "", Level level = LEVEL_VERBOSE);
    void println(String prefix, float data, String suffix = "", Level level = LEVEL_VERBOSE);
    void print(String prefix, String data, String suffix = "", Level level = LEVEL_VERBOSE);
    void println(String prefix, String data, String suffix = "", Level level = LEVEL_VERBOSE);

    void print(String prefix, Point data, String suffix = "", Level level = LEVEL_VERBOSE, boolean lineFeed = true);
    void print(String prefix, Point3D data, String suffix = "", Level level = LEVEL_VERBOSE, boolean lineFeed = true);
    void print(String prefix, Point4D data, String suffix = "", Level level = LEVEL_VERBOSE, boolean lineFeed = true);
    void print(String prefix, PolarPoint data, String suffix = "", Level level = LEVEL_VERBOSE,
               boolean lineFeed = true);
    void print(String prefix, RobotPosition data, String suffix = "", Level level = LEVEL_VERBOSE,
               boolean lineFeed = true);
    void print(String prefix, Command cmd, String suffix = "", Level level = LEVEL_VERBOSE, boolean lineFeed = true);

    // bool needs to be the last because it overrides all functions
    void print(String prefix, bool data, String suffix = "", Level level = LEVEL_VERBOSE, boolean lineFeed = true);

    void printArray(String prefix, int array[], size_t size, char separator = ',', String suffix = "",
                    Level level = LEVEL_VERBOSE);

    /**
     * send point data to teleplot to trace x and y in a graph
     */
    void plotPoint(Point p, String varName, Level level = LEVEL_VERBOSE);

    /**
     * send batch of point data to teleplot to trace x and y in a graph
     */
    void plotPolarPoints(PolarPoint polarPoints[], uint16_t size, String varName, Level level = LEVEL_VERBOSE);

    /**
     * send point data on serial for teleplot to trace x and y as two separate graphs
     */
    void plotPoint(Point p, Level level = LEVEL_VERBOSE);

    void plotScanAD(std::vector<PolarPoint> scan, Level level = LEVEL_VERBOSE);
    void plotScanXY(std::vector<PolarPoint> scan, Level level = LEVEL_VERBOSE);

    void plotRobot(RobotPosition pos, Level level = LEVEL_VERBOSE);

    void plotTrackerPoints(PointTracker p, int size, String varName, Level level = LEVEL_VERBOSE);

    /**
     * send cloud point data on serial for teleplot to trace 3D shape
     */
    void plot3D(Point3D p, String varName);
    void plot3Dpy(Point3D p);
};  // namespace Printer
#endif
