#include "ESP32_Helper.h"

char readBuffer[READ_SERIAL_BUFFER_SIZE];
uint16_t indexBuffer;

void ESP32_Helper()
{
    SERIAL_DEBUG.begin(SERIAL_DEBUG_SPEED);
    if (SERIAL_DEBUG.available() > 0)
    {
        SERIAL_DEBUG.flush();
    }

    strcpy(readBuffer, "");
    indexBuffer = 0;

    SERIAL_DEBUG.println();
    SERIAL_DEBUG.println(".--------------.");
    SERIAL_DEBUG.println("  MECAPITRONIC  ");
    SERIAL_DEBUG.println("'--------------'");
    SERIAL_DEBUG.println();
    SERIAL_DEBUG.print(__DATE__);
    SERIAL_DEBUG.print(" at ");
    SERIAL_DEBUG.println(__TIME__);
    SERIAL_DEBUG.println();
    delay(200);
}

char* checkSerial()
{
    if (SERIAL_DEBUG.available() > 0)
    {
        char tmpChar = SERIAL_DEBUG.read();
        if (indexBuffer < READ_SERIAL_BUFFER_SIZE)
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
