#ifndef DEBUGGER_H
#define DEBUGGER_H

#include "main.h"

#define VERBOSE Debugger::Level::LEVEL_VERBOSE
#define INFO Debugger::Level::LEVEL_INFO
#define WARN Debugger::Level::LEVEL_WARN
#define ERROR Debugger::Level::LEVEL_ERROR

#define SERIAL_DEBUG SERIAL_PC

namespace Debugger
{

    enum class Level
    {
        LEVEL_VERBOSE = 0,
        LEVEL_INFO = 1,
        LEVEL_WARN = 2,
        LEVEL_ERROR = 3
    };

    void header();
    void init(Level = INFO);
    void checkSerial();

    Level level();

    void println(char);
    void println(String = "");
    void println(bool);
    void println(int);
    void println(float);

    void print(char);
    void print(String);
    void print(bool);
    void print(int);
    void print(float);

    void log(String prefix, int data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);
    void log(String prefix, char data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);
    void log(String prefix, float data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);
    void log(String prefix, bool data, String suffix = "", Level level = VERBOSE, boolean lineFeed = true);

    void logArray(String prefix, int array[], size_t size, char separator = ',', String suffix = ")", Level level = VERBOSE);
    void logArrayN(String prefix, int element, String interFix, int array[], size_t size, char separator = ',', String suffix = ")",
                   Level level = VERBOSE);

}; // namespace Debugger

#endif