#include "A010.h"

A010::A010()
{
    Debugger::log("Init A010");

    // minDistance, maxDistance
    Config(100, 1500, 2);
    SERIAL_A010.begin(115200);

    // wait A010 power up
    delay(1000);

    // AT commands configuration
    // https://wiki.sipeed.com/hardware/en/maixsense/maixsense-a010/at_command_en.html
    // https://wiki.sipeed.com/hardware/en/metasense/metasense-a010/metasense-a010.html
    SERIAL_A010.println("AT+BINN=" + String(BINNING_SIZE));     // pixel binning : 1=1x1 (100x100), 2=2x2 (50x50), 4=4x4 (25x25)
    SERIAL_A010.println("AT+DISP=5");                           // 2=usb, 3=usb+lcd, 4=uart, 5=uart+lcd, 6=usb+uart, 7=usb+uart+lcd
    SERIAL_A010.println("AT+UNIT=" + String(QUANTIZATION_MM));  // 1 to 9, Represents quantization in x mm. The smaller the value, the more details
                                                                // and the shorter the visual distance.
    SERIAL_A010.println("AT+FPS=20");                           // FPS from 1 to 20 (30?)
    SERIAL_A010.println("AT+ANTIMMI=-1");  // Automatic anti-multi-machine interference is turned on and off (susceptible to interference, the effect
                                           // of turning off is better)
    SERIAL_A010.println("AT+AE=0");        // Ev:Exposure gap control (leftmost represents AE, others are fixed exposure time)

    // TODO: receive command response to see if it's OK !

    // FIXME: fait planter le lcd et ne sauvegarde rien... SERIAL_A010.println("AT+SAVE"); // The current configuration of the TOF camera is cured,
    // and it needs to be reset afterwards
    SERIAL_A010.println("AT+BAUD=3");  // 6=1M, 7=2M, 8=3M
    SERIAL_A010.flush();
    SERIAL_A010.end();
    SERIAL_A010.begin(230400);

    // Debugger::log("Init A010 COPY");
    // SERIAL_A010_COPY.begin(115200, SERIAL_8N1, RX1, TX1);

    serialBuffer.clear();
}

void A010::Config(int min = -1, int max = -1, int discontinuity = -1)
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
    if (discontinuity != -1)
    {
        Debugger::log("A010 Config 'Discontinuity ' from ", a010Config.IDMaxDiscontinuity, "", INFO, false);
        Debugger::log(" to ", discontinuity, "", INFO);
        a010Config.IDMaxDiscontinuity = discontinuity;
    }
}

boolean A010::ReadSerial()
{
    while (SERIAL_A010.available() > 0)
    {
        uint8_t tmpInt = SERIAL_A010.read();

        if (cursorTmp == 0)  // First byte of packet
        {
            if (tmpInt == A010_FIRST_PACKET_BYTE)
            {
                serialBuffer.clear();
                serialBuffer.push_back(tmpInt);
                cursorTmp++;
            }
            else
            {
                serialBuffer.clear();
                cursorTmp = 0;
            }
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
            // Debugger::log("packetSize", packetSize);
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

void A010::FillStructure()
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

    memcpy(&a010Packet.payload[0], &serialBuffer[20], sizeof(a010Packet.payload));

    a010Packet.frame_tail.checksum = serialBuffer[serialBuffer.size() - 2];
    a010Packet.frame_tail.frame_end_flag = serialBuffer[serialBuffer.size() - 1];
}

a010_frame_t A010::GetData() { return a010Packet; }

boolean A010::CheckContinuity()
{
    // We compare the ID of this packet with the ID of the previous packet
    int delta = a010Packet.frame_head.frame_id - a010LastPacketHeader.frame_id;
    // previous packet overflowed
    if (a010LastPacketHeader.frame_id <= a010Packet.frame_head.frame_id)
    {
        // TODO get the overflow value
        // delta += 4096;
    }

    // save the last point to compare to the next packet's first point
    a010LastPacketHeader = a010Packet.frame_head;

    if (delta > a010Config.IDMaxDiscontinuity)
    {
        Debugger::log("a010Packet.frame_head.frame_id : ", a010Packet.frame_head.frame_id, "", WARN);
        Debugger::log("a010LastPacketHeader.frame_id : ", a010LastPacketHeader.frame_id, "", WARN);
        Debugger::log("Discontinuity : ", delta, "", WARN);
        return false;
    }
    else
    {
        return true;
    }
}

// A010 FOV 70°(H) * 60°(V) => 1.22173 rad * 1.0472
a010_point_cloud_t A010::GetPointCloudFromFrame(a010_frame_t frame)
{
    a010_point_cloud_t cloud;
    uint8_t row, col = 1;
    uint16_t i = 1;
    double dist, ang_h, ang_v;
    double x, y, z;
    // frame.frame_head.resolution_cols = 25;
    // frame.frame_head.resolution_rows = 25;
    double res_h = 1.22173 / (double)frame.frame_head.resolution_cols;  // 0.0488692
    double res_v = 1.0472 / (double)frame.frame_head.resolution_rows;   // 0.041888
    double zero_h = (double)frame.frame_head.resolution_cols / 2;       // 12.5
    double zero_v = (double)frame.frame_head.resolution_rows / 2;       // 12.5

    for (row = 1; row <= frame.frame_head.resolution_rows; row++)
    {
        for (col = 1; col <= frame.frame_head.resolution_cols; col++)
        {
            i = col + ((row - 1) * frame.frame_head.resolution_rows) - 1;  // from 0 to n-1
            dist = frame.payload[i];
            if (dist < 255)  // ignorer le fond
            {
                dist *= QUANTIZATION_MM;
                ang_h = col;
                ang_h -= zero_h;
                ang_h *= res_h;
                // ang_h = res_h * (col - zero_h);
                ang_v = zero_v;
                ang_v -= row;
                ang_v *= res_v;
                // ang_v = res_v * (zero_v - row);

                x = dist * sin(ang_h);
                y = dist * cos(ang_h);
                z = dist * sin(ang_v);
                cloud.point[i].x = (int16_t)x;
                cloud.point[i].y = (int16_t)y;
                cloud.point[i].z = (int16_t)z;
                cloud.cluster[i] = 0xff00;
                if (frame.frame_head.frame_id % 2)
                    cloud.cluster[i] = 0xff00;
                else
                    cloud.cluster[i] = 0x00ff;
                String data = "" + String(cloud.point[i].x) + " " + String(cloud.point[i].y) + " " + String(cloud.point[i].z);
                //+" " + String(cloud.cluster[i]);
                // SERIAL_DEBUG.println(String(dist));
                SERIAL_DEBUG.println(data);
            }
        }
    }
    return cloud;
}

void A010::logHeader()
{
    Debugger::log("frame_data_len ", a010Packet.frame_head.frame_data_len);
    Debugger::log("output_mode ", a010Packet.frame_head.output_mode);
    Debugger::log("senser_temp ", a010Packet.frame_head.senser_temp);
    Debugger::log("driver_temp ", a010Packet.frame_head.driver_temp);
    Debugger::log("exposure_time[0] ", a010Packet.frame_head.exposure_time[0]);
    Debugger::log("exposure_time[1] ", a010Packet.frame_head.exposure_time[1]);
    Debugger::log("exposure_time[2] ", a010Packet.frame_head.exposure_time[2]);
    Debugger::log("exposure_time[3] ", a010Packet.frame_head.exposure_time[3]);
    Debugger::log("error_code ", a010Packet.frame_head.error_code);
    Debugger::log("resolution_rows ", a010Packet.frame_head.resolution_rows);
    Debugger::log("resolution_cols ", a010Packet.frame_head.resolution_cols);
    Debugger::log("frame_id ", a010Packet.frame_head.frame_id);
    Debugger::log("isp_version ", a010Packet.frame_head.isp_version);
}
