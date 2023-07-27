#include "ESP32_Helper.h"

using namespace Printer;

namespace ESP32_Helper
{

void ESP32_Helper(int baud_speed)
{
    SERIAL_DEBUG.begin(baud_speed);
    if (SERIAL_DEBUG.available() > 0)
    {
        SERIAL_DEBUG.flush();
    }
    SERIAL_DEBUG.println();

    Printer::PrintEnable(Enable::ENABLE_TRUE);
    Printer::PrintLevel(Level::LEVEL_VERBOSE);

    println();
    println("|--------------|");
    println("  MECAPITRONIC  ");
    println("|--------------|");
    println();
    print(__DATE__);
    print(" at ");
    println(__TIME__);
    println();

    Debugger::EnableDebugger(ENABLE_TRUE);
    Debugger::Initialisation();
}

char* checkSerial()
{
    // static declaration to initialise only once at startup
    static uint16_t indexBuffer = 0;
    static char readBuffer[Serial_Read_Buffer];

    if (SERIAL_DEBUG.available() > 0)
    {
        char tmpChar = SERIAL_DEBUG.read();
        if (indexBuffer < Serial_Read_Buffer)
        {
            readBuffer[indexBuffer++] = tmpChar;
            if (tmpChar == '\n')
            {
                SERIAL_DEBUG.print("Received : ");
                SERIAL_DEBUG.write(readBuffer, indexBuffer);
                indexBuffer = 0;
                return readBuffer;
            }
        }
        else
        {
            SERIAL_DEBUG.print("Read Buffer Overflow : ");
            SERIAL_DEBUG.println(indexBuffer);
            indexBuffer = 0;
        }
    }
    return nullptr;
}
}  // namespace ESP32_Helper
