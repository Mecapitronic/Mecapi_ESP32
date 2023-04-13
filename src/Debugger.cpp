#include "Debugger.h"

namespace Debugger
{

    boolean enabled = true;
    Level debugLevel = WARN;

    void header()
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

    void init(Level level)
    {
        if (enabled)
        {
            SERIAL_DEBUG.begin(230400);
            debugLevel = level;

            if (SERIAL_DEBUG.available() <= 0)
            {
            }

            header();
            SERIAL_DEBUG.print("Preparing system...");

            delay(200);
            SERIAL_DEBUG.println("done.");
        }
    }

    Level level() { return debugLevel; }

    void checkSerial()
    {
        if (enabled)
        {
            if (SERIAL_DEBUG.available() > 0)
            {
                String command = SERIAL_DEBUG.readStringUntil('\n');
                SERIAL_DEBUG.println("Received :" + command);

                // if (command.startsWith("Config:"))
                //{
                LD06::Config(0, atoi(command.c_str()), 100);
                //}
            }
        }
    }

    void println(String message) { SERIAL_DEBUG.println(message); }

    void println(char c) { SERIAL_DEBUG.println(c); }

    void println(float data) { SERIAL_DEBUG.println(data); }

    void println(int data) { SERIAL_DEBUG.println(data); }

    void print(String message) { SERIAL_DEBUG.print(message); }

    void print(char c) { SERIAL_DEBUG.print(c); }

    void print(float data) { SERIAL_DEBUG.print(data); }

    void print(int data) { SERIAL_DEBUG.print(data); }

    void log(int data, Level level)
    {
        if (level < debugLevel)
            return;
        println(data);
    }

    void log(char data, Level level)
    {
        if (level < debugLevel)
            return;
        println(data);
    }

    void log(float data, Level level)
    {
        if (level < debugLevel)
            return;
        println(data);
    }

    void log(String data, Level level)
    {
        if (level < debugLevel)
            return;
        println(data);
    }

    void log(String prefix, int data, String suffix, Level level)
    {
        if (level < debugLevel)
            return;
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.print(data);
        SERIAL_DEBUG.println(suffix);
    }

    void log(String prefix, float data, String suffix, Level level)
    {
        if (level < debugLevel)
            return;
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.print(data);
        SERIAL_DEBUG.println(suffix);
    }

    void log(String prefix, bool data, String suffix, Level level)
    {
        if (level < debugLevel)
            return;
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.print(data);
        SERIAL_DEBUG.println(suffix);
    }

    void log(String prefix, int data, Level level)
    {
        if (level < debugLevel)
            return;
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.println(data);
    }

    void log(String prefix, float data, Level level)
    {
        if (level < debugLevel)
            return;
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.println(data);
    }

    void log(String prefix, bool data, Level level)
    {
        if (level < debugLevel)
            return;
        SERIAL_DEBUG.print(prefix);
        SERIAL_DEBUG.println(data);
    }

    void logArray(String prefix, int array[], size_t size, char separator, String suffix, Level level)
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

    void logArrayN(String prefix, int element, String interFix, int array[], size_t size, char separator, String suffix, Level level)
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

} // namespace Debugger