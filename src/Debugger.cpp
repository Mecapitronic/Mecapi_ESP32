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

void Debugger::log(String data, Level level, boolean lineFeed)
{
    if (level < debugLevel) return;
    SERIAL_DEBUG.print(data);
    if (lineFeed) SERIAL_DEBUG.println();
}

void Debugger::log(String prefix, int data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel) return;
    SERIAL_DEBUG.print(prefix);
    SERIAL_DEBUG.print(data);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed) SERIAL_DEBUG.println();
}

void Debugger::log(String prefix, char data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel) return;
    SERIAL_DEBUG.print(prefix);
    SERIAL_DEBUG.print(data);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed) SERIAL_DEBUG.println();
}

void Debugger::log(String prefix, float data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel) return;
    SERIAL_DEBUG.print(prefix);
    SERIAL_DEBUG.print(data);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed) SERIAL_DEBUG.println();
}

void Debugger::log(String prefix, Point data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel) return;
    SERIAL_DEBUG.print(prefix);
    SERIAL_DEBUG.print("x: ");
    SERIAL_DEBUG.print(data.x);
    SERIAL_DEBUG.print(" y: ");
    SERIAL_DEBUG.print(data.y);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed) SERIAL_DEBUG.println();
}

void Debugger::log(String prefix, PolarPoint data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel) return;
    SERIAL_DEBUG.print("A: ");
    SERIAL_DEBUG.print((int)(data.angle / 100));
    SERIAL_DEBUG.print(" D: ");
    SERIAL_DEBUG.print(data.distance);
    SERIAL_DEBUG.print(" C: ");
    SERIAL_DEBUG.println(data.confidence);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed) SERIAL_DEBUG.println();
}

void Debugger::log(String prefix, a010_frame_t data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel) return;
    log("frame_data_len", data.frame_head.frame_data_len, "", level, true);
    if (lineFeed) SERIAL_DEBUG.println();
}

// bool needs to be the last because it overrides all functions
void Debugger::log(String prefix, bool data, String suffix, Level level, boolean lineFeed)
{
    if (level < debugLevel) return;
    SERIAL_DEBUG.print(prefix);
    SERIAL_DEBUG.print(data);
    SERIAL_DEBUG.print(suffix);
    if (lineFeed) SERIAL_DEBUG.println();
}

void Debugger::logArray(String prefix, int array[], size_t size, char separator, String suffix, Level level)
{
    if (level < debugLevel) return;
    if (size > 0)
    {
        SERIAL_DEBUG.print(prefix);
        for (size_t i = 0; i < size - 1; i++)
        {
            SERIAL_DEBUG.print(array[i]);
            SERIAL_DEBUG.print(separator);
        }
        SERIAL_DEBUG.print(array[size - 1]);
        SERIAL_DEBUG.println(suffix);
    }
}

void Debugger::logArrayN(String prefix, int element, String interFix, int array[], size_t size, char separator, String suffix, Level level)
{
    if (level < debugLevel) return;
    if (size > 0)
    {
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.print(element);
        SERIAL_DEBUG.print(interFix);
        for (size_t i = 0; i < size - 1; i++)
        {
            SERIAL_DEBUG.print(array[i]);
            SERIAL_DEBUG.print(separator);
        }
        SERIAL_DEBUG.print(array[size - 1]);
        SERIAL_DEBUG.println(suffix);
    }
    else
        SERIAL_DEBUG.println("Invalid array printed !");
}

void Debugger::plotPoint(Point p, String varName, Level level)
{
    if (level < debugLevel) return;
    String data = ">" + varName + ":" + (int)p.x + ":" + (int)p.y + "|xy";
    SERIAL_DEBUG.println(data);
}

void Debugger::plotPoint(Point p, Level level)
{
    if (level < debugLevel) return;
    SERIAL_DEBUG.print(">x:");
    SERIAL_DEBUG.println((int)p.x);
    SERIAL_DEBUG.print(">y:");
    SERIAL_DEBUG.println((int)p.y);
}
