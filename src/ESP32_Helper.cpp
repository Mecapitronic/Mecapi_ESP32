#include "ESP32_Helper.h"

namespace ESP32_Helper
{

void ESP32_Helper()
{
    SERIAL_DEBUG.begin(Serial_Debug_Speed);
    if (SERIAL_DEBUG.available() > 0)
    {
        SERIAL_DEBUG.flush();
    }

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
