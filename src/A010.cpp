#include "A010.h"

A010::A010()
{
    Debugger::log("Init A010");

    // minDistance, maxDistance
    Config(100, 1500);
    SERIAL_A010.begin(115200);

    Debugger::log("Init A010 COPY");
    SERIAL_A010_COPY.begin(115200, SERIAL_8N1, RX1, TX1);

    serialBuffer.clear();
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
        SERIAL_A010_COPY.write(tmpInt);

        if (cursorTmp == 0 && tmpInt == A010_FIRST_PACKET_BYTE)  // First byte of packet
        {
            serialBuffer.push_back(tmpInt);
            cursorTmp++;
        }
        else if (cursorTmp == 1)
        {
            if (tmpInt == A010_SECOND_PACKET_BYTE)
            {
                serialBuffer.push_back(tmpInt);
                cursorTmp++;
            }
            else
            {
                serialBuffer.clear();
                cursorTmp = 0;
            }
        }
        else if (cursorTmp == 2)
        {
            serialBuffer.push_back(tmpInt);
            cursorTmp++;
        }
        else if (cursorTmp == 3)
        {
            serialBuffer.push_back(tmpInt);
            cursorTmp++;
            packetSize = serialBuffer[3] << 8 | serialBuffer[2];
        }
        else if (cursorTmp > 3)
        {
            serialBuffer.push_back(tmpInt);
            cursorTmp++;
            if (cursorTmp == packetSize)
            {
                cursorTmp = 0;
                return true;
            }
        }
    }
    return false;
}