/**
 * @file Printer.h
 * @author Mecapitronic (mecapitronic@gmail.com)
 * @brief Used to print data over predefined serial for debug purpose
 * @date 2023-07-25
 */
#ifndef PRINTER_H
#define PRINTER_H

#include "ESP32_Helper.h"

namespace Printer
{
// static bool printEnable = false;
// Level printLevel = LEVEL_NONE;

Level PrintLevel(Level level = LEVEL_NONE);
Enable PrintEnable(Enable enable = Enable::ENABLE_NONE);
bool IsPrintable();

void println(Level level = LEVEL_VERBOSE);

void print(String data, Level level = LEVEL_VERBOSE);
void println(String data, Level level = LEVEL_VERBOSE);
void print(String prefix, int data, String suffix = "", Level level = LEVEL_VERBOSE);
void println(String prefix, int data, String suffix = "", Level level = LEVEL_VERBOSE);
void print(String prefix, char data, String suffix = "", Level level = LEVEL_VERBOSE);
void println(String prefix, char data, String suffix = "", Level level = LEVEL_VERBOSE);
void print(String prefix, float data, String suffix = "", Level level = LEVEL_VERBOSE);
void println(String prefix, float data, String suffix = "", Level level = LEVEL_VERBOSE);

void print(String prefix, Point data, String suffix = "", Level level = LEVEL_VERBOSE, boolean lineFeed = true);
void print(String prefix, Point3D data, String suffix = "", Level level = LEVEL_VERBOSE, boolean lineFeed = true);
void print(String prefix, Point4D data, String suffix = "", Level level = LEVEL_VERBOSE, boolean lineFeed = true);
void print(String prefix, PolarPoint data, String suffix = "", Level level = LEVEL_VERBOSE, boolean lineFeed = true);

// bool needs to be the last because it overrides all functions
void print(String prefix, bool data, String suffix = "", Level level = LEVEL_VERBOSE, boolean lineFeed = true);

void printArray(String prefix, int array[], size_t size, char separator = ',', String suffix = "", Level level = LEVEL_VERBOSE);

/**
 * send point data on serial for teleplot to trace x and y in a graph
 */
void plotPoint(Point p, String varName, Level level = LEVEL_VERBOSE);

/**
 * send point data on serial for teleplot to trace x and y as two separate graphs
 */
void plotPoint(Point p, Level level = LEVEL_VERBOSE);

/**
 * send cloud point data on serial for teleplot to trace 3D shape
 */
void plot3D(Point3D p, String varName);
void plot3Dpy(Point3D p);
};  // namespace Printer
#endif
