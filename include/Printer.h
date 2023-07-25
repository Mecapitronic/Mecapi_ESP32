/**
 * Helper functions to print via serial
 */

#ifndef PRINTER_H
#define PRINTER_H

#include "ESP32_Helper.h"

#define VERBOSE Level::LEVEL_VERBOSE
#define INFO Level::LEVEL_INFO
#define WARN Level::LEVEL_WARN
#define ERROR Level::LEVEL_ERROR

class Printer
{
   private:
    static const Level debugLevel = INFO;

   public:
    /**
     * Return the debugging level used: VERBOSE, INFO,WARN or ERROR
     */
    // static Level level();

    static void log(String data, Level level = VERBOSE, boolean lineFeed = true);
    static void log(String prefix, int data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);
    static void log(String prefix, char data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);
    static void log(String prefix, float data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);

    static void log(String prefix, Point data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);
    static void log(String prefix, PolarPoint data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);

    // bool needs to be the last because it overrides all functions
    static void log(String prefix, bool data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);

    static void logArray(String prefix, int array[], size_t size, char separator = ',', String suffix = ")", Level level = VERBOSE);
    static void logArrayN(String prefix, int element, String interFix, int array[], size_t size, char separator = ',', String suffix = ")", Level level = VERBOSE);

    /**
     * send point data on serial for teleplot to trace x and y in a graph
     */
    static void plotPoint(Point p, String varName, Level level = INFO);

    /**
     * send point data on serial for teleplot to trace x and y as two separate graphs
     */
    static void plotPoint(Point p, Level level = INFO);

    /**
     * send cloud point data on serial for teleplot to trace 3D shape
     */
    static void plot3D(Point3D p, String varName);
    static void plot3Dpy(Point3D p);
};
#endif
