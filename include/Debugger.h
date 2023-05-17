/**
 * Helper functions to print and debug via serial
 */

#ifndef DEBUGGER_H
#define DEBUGGER_H

#define VERBOSE Level::LEVEL_VERBOSE
#define INFO Level::LEVEL_INFO
#define WARN Level::LEVEL_WARN
#define ERROR Level::LEVEL_ERROR

#include <Arduino.h>
#include "Structure.h"

#define SERIAL_PC Serial
#define SERIAL_DEBUG SERIAL_PC

enum class Level
{
    LEVEL_VERBOSE = 0,
    LEVEL_INFO = 1,
    LEVEL_WARN = 2,
    LEVEL_ERROR = 3
};

class Debugger
{

public:
    /**
     * Print custom header on debugging serial
     */
    static void header();
    /**
     * Initialize debugging serial for PC communication
     */
    static void init();

    /**
     * Check for commands send on debugging serial plugged to a computer
     * List of commands is still empty now
     */
    static String checkSerial();

    /**
     * Return the debugging level used: VERBOSE, INFO,WARN or ERROR
     */
    static Level level();

    static void log(String data = "", Level level = VERBOSE, boolean lineFeed = true);
    static void log(String prefix, int data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);
    static void log(String prefix, char data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);
    static void log(String prefix, float data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);
    // static void log(String prefix, Obstacle data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);
    static void log(String prefix, Point data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);
    static void log(String prefix, PointLidar data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);
    static void log(String prefix, PointTracker data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);
    static void log(String prefix, PolarPoint data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);
    static void log(String prefix, RobotPosition data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);
    static void log(String prefix, PacketLidar data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);

    // bool needs to be the last because it overrides all functions
    static void log(String prefix, bool data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);

    static void
    logArray(String prefix, int array[], size_t size, char separator = ',', String suffix = ")", Level level = VERBOSE);
    static void logArrayN(String prefix, int element, String interFix, int array[], size_t size, char separator = ',', String suffix = ")",
                          Level level = VERBOSE);

    /**
     * send point data on serial for teleplot to trace x and y in a graph
     */
    static void plotTrackerPoints(PointTracker p, int size, String varName);

    /**
     * send point data on serial for teleplot to trace x and y in a graph
     */
    static void plotPoint(Point p, String varName);

    /**
     * send point data on serial for teleplot to trace x and y as two separate graphs
     */
    static void plotPoint(Point p);

private:
    static const boolean enabled = true;
    static const Level debugLevel = VERBOSE;
};
#endif