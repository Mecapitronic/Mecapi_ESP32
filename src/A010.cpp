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
            serialBuffer.clear();
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
            Debugger::log("packetSize", packetSize);
        }
        else if (cursorTmp > 3)
        {
            serialBuffer.push_back(tmpInt);
            cursorTmp++;
            if (cursorTmp == packetSize + 4 + 2)  // length count from Byte4 to the Byte before Checksum
            {
                cursorTmp = 0;
                return true;
            }
        }
    }
    return false;
}

void A010::Analyze()
{
    a010Packet.frame_head.frame_begin_flag = serialBuffer[1] << 8 | serialBuffer[0];
    a010Packet.frame_head.frame_data_len = serialBuffer[3] << 8 | serialBuffer[2];
    a010Packet.frame_head.reserved1 = serialBuffer[4];
    a010Packet.frame_head.output_mode = serialBuffer[5];
    a010Packet.frame_head.senser_temp = serialBuffer[6];
    a010Packet.frame_head.driver_temp = serialBuffer[7];
    a010Packet.frame_head.exposure_time[0] = serialBuffer[8];
    a010Packet.frame_head.exposure_time[1] = serialBuffer[9];
    a010Packet.frame_head.exposure_time[2] = serialBuffer[10];
    a010Packet.frame_head.exposure_time[3] = serialBuffer[11];
    a010Packet.frame_head.error_code = serialBuffer[12];
    a010Packet.frame_head.reserved2 = serialBuffer[13];
    a010Packet.frame_head.resolution_rows = serialBuffer[14];
    a010Packet.frame_head.resolution_cols = serialBuffer[15];
    a010Packet.frame_head.frame_id = serialBuffer[17] << 8 | serialBuffer[16];
    a010Packet.frame_head.isp_version = serialBuffer[18];
    a010Packet.frame_head.reserved3 = serialBuffer[19];

    a010Packet.frame_tail.checksum = serialBuffer[serialBuffer.size() - 2];
    a010Packet.frame_tail.frame_end_flag = serialBuffer[serialBuffer.size() - 1];
}

a010_frame_t A010::GetData() { return a010Packet; }