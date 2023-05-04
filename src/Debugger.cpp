#include "Debugger.h"

void Debugger::init()
{
    if (enabled)
    {
        SERIAL_DEBUG.begin(230400);

        if (SERIAL_DEBUG.available() <= 0)
        {
        }

        header();
        SERIAL_DEBUG.print("Preparing system...");
        delay(200);
        SERIAL_DEBUG.println("done.");
    }
}

void Debugger::header()
{
    SERIAL_DEBUG.println();
    SERIAL_DEBUG.println(".--------------.");
    SERIAL_DEBUG.println("  MECAPITRONIC  ");
    SERIAL_DEBUG.println("'--------------'");
    SERIAL_DEBUG.println();
    SERIAL_DEBUG.print(__DATE__);
    SERIAL_DEBUG.print(" at ");
    SERIAL_DEBUG.println(__TIME__);
    SERIAL_DEBUG.println();
}

Level Debugger::level() { return debugLevel; }

String Debugger::checkSerial()
{
    if (enabled && SERIAL_DEBUG.available() > 0)
    {
        String command = SERIAL_DEBUG.readStringUntil('\n');
        SERIAL_DEBUG.println("Received : " + command);
        return command;
    }
    return "";
}

void Debugger::println(String message) { SERIAL_DEBUG.println(message); }

void Debugger::println(char c) { SERIAL_DEBUG.println(c); }

void Debugger::println(float data) { SERIAL_DEBUG.println(data); }

void Debugger::println(int data) { SERIAL_DEBUG.println(data); }

void Debugger::print(String message) { SERIAL_DEBUG.print(message); }

void Debugger::print(char c) { SERIAL_DEBUG.print(c); }

void Debugger::print(float data) { SERIAL_DEBUG.print(data); }

void Debugger::print(int data) { SERIAL_DEBUG.print(data); }

void Debugger::log(String prefix, int data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel)
        return;
    SERIAL_DEBUG.print(prefix);
    SERIAL_DEBUG.print(data);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed)
        SERIAL_DEBUG.println();
}

void Debugger::log(String prefix, char data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel)
        return;
    SERIAL_DEBUG.print(prefix);
    SERIAL_DEBUG.print(data);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed)
        SERIAL_DEBUG.println();
}

void Debugger::log(String prefix, float data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel)
        return;
    SERIAL_DEBUG.print(prefix);
    SERIAL_DEBUG.print(data);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed)
        SERIAL_DEBUG.println();
}

void Debugger::log(String prefix, bool data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel)
        return;
    SERIAL_DEBUG.print(prefix);
    SERIAL_DEBUG.print(data);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed)
        SERIAL_DEBUG.println();
}

void Debugger::logArray(String prefix, int array[], size_t size, char separator, String suffix, Level level)
{
    if (level < debugLevel)
        return;
    if (size > 0)
    {
        print(prefix);
        for (size_t i = 0; i < size - 1; i++)
        {
            print(array[i]);
            print(separator);
        }
        print(array[size - 1]);
        println(suffix);
    }
}

void Debugger::logArrayN(String prefix, int element, String interFix, int array[], size_t size, char separator, String suffix, Level level)
{
    if (level < debugLevel)
        return;
    if (size > 0)
    {
        print(prefix);
        print(element);
        print(interFix);
        for (size_t i = 0; i < size - 1; i++)
        {
            print(array[i]);
            print(separator);
        }
        print(array[size - 1]);
        println(suffix);
    }
    else
        println("Invalid array printed !");
}

void Debugger::printPolarPoint(PolarPoint p, Level level)
{
    if (level < debugLevel)
    {
        SERIAL_DEBUG.print((int)p.angle);
        SERIAL_DEBUG.print(";");
        SERIAL_DEBUG.print(p.distance);
        SERIAL_DEBUG.print(";");
        SERIAL_DEBUG.println(p.confidence);
    }
    else
    {
        SERIAL_DEBUG.print("Angle:");
        SERIAL_DEBUG.print((int)p.angle);
        SERIAL_DEBUG.print(" Distance:");
        SERIAL_DEBUG.print(p.distance);
        SERIAL_DEBUG.print(" Confidence:");
        SERIAL_DEBUG.println(p.confidence);
    }
}

void Debugger::printPoint(Point p, Level level)
{
    if (level < debugLevel)
    {
        SERIAL_DEBUG.print((int)p.x);
        SERIAL_DEBUG.print(";");
        SERIAL_DEBUG.print((int)p.y);
    }
    else
    {
        SERIAL_DEBUG.print("X:");
        SERIAL_DEBUG.print((int)p.x);
        SERIAL_DEBUG.print(" Y:");
        SERIAL_DEBUG.print((int)p.y);
    }
}

void Debugger::plotPoint(Point p, String varName)
{
    String data = ">" + varName + ":" + (int)p.x + ":" + (int)p.y + "|xy";
    SERIAL_DEBUG.println(data);
}
