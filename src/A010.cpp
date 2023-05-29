#include "A010.h"

A010::A010()
{
    Debugger::log("Init A010");

    // minDistance, maxDistance
    Config(100, 1500);
    SERIAL_A010.begin(230400);
}

void A010::Config(int min = -1, int max = -1)
{
    if (min != -1)
    {
        Debugger::log("A010 Config 'Distance Min' from ", a010Config.minDistance, "", INFO, false);
        Debugger::log(" to ", min, "", INFO);
        a010Config.minDistance = min;
    }
    if (max != -1)
    {
        Debugger::log("A010 Config 'Distance Max' from ", a010Config.maxDistance, "", INFO, false);
        Debugger::log(" to ", max, "", INFO);
        a010Config.maxDistance = max;
    }
}

boolean A010::ReadSerial()
{
    while (SERIAL_A010.available() > 0)
    {
        uint32_t tmpInt = SERIAL_A010.read();
        // SERIAL_3.write(tmpInt);
        if (tmpInt == A010_FIRST_PACKET_BYTE && cursorTmp == 0)  // First byte of packet
        {
            serialBuffer[cursorTmp++] = tmpInt;
        }
        else if (cursorTmp > 0)
        {
            serialBuffer[cursorTmp++] = tmpInt;
            if (cursorTmp == A010_SERIAL_PACKET_SIZE)
            {
                cursorTmp = 0;
                return true;
            }
        }
    }
    return false;
}