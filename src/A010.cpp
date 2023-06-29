#include "A010.h"

A010::A010() {}

void A010::Initialisation()
{
    Debugger::log("Init A010");
    // minDistance, maxDistance
    Config(100, 1500, 2);
    SERIAL_A010.begin(115200);

    // wait A010 power up
    delay(1000);

    // pre-computing of coefficient to convert depth data to cartesian coordinate point
    ComputeCartesianCoefficient(PICTURE_RES, PICTURE_RES, FOV_HOR, FOV_VER, 0, 0);
    // logCartesianCoefficient();

    // AT commands configuration
    // https://wiki.sipeed.com/hardware/en/maixsense/maixsense-a010/at_command_en.html
    // https://wiki.sipeed.com/hardware/en/metasense/metasense-a010/metasense-a010.html
    SERIAL_A010.println("AT+BINN=" + String(BINNING_SIZE));     // pixel binning : 1=1x1 (100x100), 2=2x2 (50x50), 4=4x4 (25x25)
    SERIAL_A010.println("AT+DISP=5");                           // 2=usb, 3=usb+lcd, 4=uart, 5=uart+lcd, 6=usb+uart, 7=usb+uart+lcd
    SERIAL_A010.println("AT+UNIT=" + String(QUANTIZATION_MM));  // 1 to 9, Represents quantization in x mm. The smaller the value, the more details
                                                                // and the shorter the visual distance.
    SERIAL_A010.println("AT+FPS=" + String(FRAME_PER_SECOND));  // FPS from 1 to 20 (30?)
    SERIAL_A010.println("AT+ANTIMMI=-1");  // Automatic anti-multi-machine interference is turned on and off (susceptible to interference, the effect
                                           // of turning off is better)
    SERIAL_A010.println("AT+AE=0");        // Ev:Exposure gap control (leftmost represents AE, others are fixed exposure time)

    // TODO: receive command response to see if it's OK !

    // FIXME: fait planter le lcd et ne sauvegarde rien... SERIAL_A010.println("AT+SAVE"); // The current configuration of the TOF camera is cured,
    // and it needs to be reset afterwards
    SERIAL_A010.println("AT+BAUD=" + String(BAUD_RATE_STATE));  // 6=1M, 7=2M, 8=3M
    SERIAL_A010.flush();
    SERIAL_A010.end();

    InitTmpVariables();

    SERIAL_A010.setRxBufferSize(SERIAL_SIZE_RX);
    SERIAL_A010.begin(BAUD_RATE_SPEED);

    // Debugger::log("Init A010 COPY");
    // SERIAL_A010_COPY.begin(115200, SERIAL_8N1, RX1, TX1);
}

void A010::InitTmpVariables()
{
    cursorTmp = 0;
    indexTmp = 0;
    packetSize = 0;
    checksum = 0;
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
        Debugger::log("A010 Config 'Discontinuity' from ", a010Config.IDMaxDiscontinuity, "", INFO, false);
        Debugger::log(" to ", discontinuity, "", INFO);
        a010Config.IDMaxDiscontinuity = discontinuity;
    }
}
bool waitEndOfPacket = true;

boolean A010::ReadSerial()
{
    while (SERIAL_A010.available() > 0)
    {
        uint8_t tmpInt = SERIAL_A010.read();
        checksum += tmpInt;  // checksum calculation

        if (waitEndOfPacket)
        {
            if (tmpInt == A010_END_PACKET_BYTE)
            {
                waitEndOfPacket = false;
            }
            InitTmpVariables();
            return false;
        }

        if (cursorTmp >= 20 && packetSize > 0)
        {
            if (cursorTmp < packetSize + 4)
            {
                float tmpF = tmpInt;
                uint16_t hor = indexTmp % PICTURE_RES;  // plan horizontal => curseur
                uint16_t ver = indexTmp / PICTURE_RES;  // plan vertical => numéro de ligne

                if (indexTmp >= 0 && indexTmp < PICTURE_SIZE)
                {
                    a010Packet.payload[indexTmp] = tmpInt;
                    if (hor >= 0 && hor < PICTURE_RES && ver >= 0 && ver < PICTURE_RES)
                    {
                        cloudFrame[indexTmp].x = tmpF * coefX[hor];
                        cloudFrame[indexTmp].y = tmpF * coefY[hor];
                        cloudFrame[indexTmp].z = tmpF * coefZ[ver];
                    }
                }
                cursorTmp++;
                indexTmp++;
            }
            else if (cursorTmp == packetSize + 4)
            {
                a010Packet.frame_tail.checksum = tmpInt;
                cursorTmp++;
                checksum -= tmpInt;
                if (checksum == a010Packet.frame_tail.checksum)
                {
                    // Serial.println("Checksum OK !");
                }
                else
                {
                    // Serial.println();
                    // Serial.print("Checksum ");
                    // Serial.print(a010Packet.frame_tail.checksum);
                    // Serial.print(" - Calculated ");
                    // Serial.print(checksum);
                    // Serial.println();
                    // Serial.printf("'%X\n", checksum);
                    // Serial.println(SERIAL_A010.available());
                    waitEndOfPacket = true;
                    InitTmpVariables();
                }
            }
            else if (cursorTmp == packetSize + 4 + 1)  // length count from Byte4 to the Byte before Checksum
            {
                if (tmpInt == A010_END_PACKET_BYTE)
                {
                    a010Packet.frame_tail.frame_end_flag = tmpInt;
                    InitTmpVariables();
                    return true;
                }
                else
                {
                    waitEndOfPacket = true;
                    // Not the end of the frame
                    InitTmpVariables();
                }
            }
            else
            {
                // we should never come here !
                InitTmpVariables();
                Debugger::log("cursorTmp > packetSize + 4 + 2", WARN);
                Debugger::log("cursorTmp : ", cursorTmp, "", WARN);
                Debugger::log("packetSize : ", packetSize, "", WARN);
            }
        }
        else if (cursorTmp == 0)  // First byte of packet
        {
            if (tmpInt == A010_FIRST_PACKET_BYTE)
            {
                a010Packet.frame_head.frame_begin_flag = tmpInt;
                cursorTmp++;
            }
            else
            {
                // Not the start of the frame
                InitTmpVariables();
            }
        }
        else if (cursorTmp == 1)
        {
            if (tmpInt == A010_SECOND_PACKET_BYTE)
            {
                a010Packet.frame_head.frame_begin_flag |= tmpInt << 8;
                cursorTmp++;
            }
            else
            {
                // Not the start of the frame
                InitTmpVariables();
            }
        }
        else if (cursorTmp == 2)
        {
            packetSize = tmpInt;
            a010Packet.frame_head.frame_data_len = tmpInt;
            cursorTmp++;
        }
        else if (cursorTmp == 3)
        {
            packetSize |= tmpInt << 8;
            a010Packet.frame_head.frame_data_len |= tmpInt << 8;
            // Serial.printf("|%i|", packetSize);
            cursorTmp++;
        }
        else if (cursorTmp == 4)
        {
            a010Packet.frame_head.reserved1 = tmpInt;
            cursorTmp++;
        }
        else if (cursorTmp == 5)
        {
            a010Packet.frame_head.output_mode = tmpInt;
            cursorTmp++;
        }
        else if (cursorTmp == 6)
        {
            a010Packet.frame_head.senser_temp = tmpInt;
            cursorTmp++;
        }
        else if (cursorTmp == 7)
        {
            a010Packet.frame_head.driver_temp = tmpInt;
            cursorTmp++;
        }
        else if (cursorTmp == 8)
        {
            a010Packet.frame_head.exposure_time[0] = tmpInt;
            cursorTmp++;
        }
        else if (cursorTmp == 9)
        {
            a010Packet.frame_head.exposure_time[1] = tmpInt;
            cursorTmp++;
        }
        else if (cursorTmp == 10)
        {
            a010Packet.frame_head.exposure_time[2] = tmpInt;
            cursorTmp++;
        }
        else if (cursorTmp == 11)
        {
            a010Packet.frame_head.exposure_time[3] = tmpInt;
            cursorTmp++;
        }
        else if (cursorTmp == 12)
        {
            a010Packet.frame_head.error_code = tmpInt;
            cursorTmp++;
        }
        else if (cursorTmp == 13)
        {
            a010Packet.frame_head.reserved2 = tmpInt;
            cursorTmp++;
        }
        else if (cursorTmp == 14)
        {
            a010Packet.frame_head.resolution_rows = tmpInt;
            cursorTmp++;
        }
        else if (cursorTmp == 15)
        {
            a010Packet.frame_head.resolution_cols = tmpInt;
            cursorTmp++;
        }
        else if (cursorTmp == 16)
        {
            a010Packet.frame_head.frame_id = tmpInt;
            cursorTmp++;
        }
        else if (cursorTmp == 17)
        {
            a010Packet.frame_head.frame_id |= tmpInt << 8;
            cursorTmp++;
        }
        else if (cursorTmp == 18)
        {
            a010Packet.frame_head.isp_version = tmpInt;
            cursorTmp++;
        }
        else if (cursorTmp == 19)
        {
            a010Packet.frame_head.reserved3 = tmpInt;
            cursorTmp++;
            indexTmp = 0;
            // logHeader();
        }
        else
        {
            // we should never come here !
            InitTmpVariables();
            Debugger::log("cursorTmp < 0", WARN);
            Debugger::log("cursorTmp : ", cursorTmp, "", WARN);
            Debugger::log("packetSize : ", packetSize, "", WARN);
        }
    }
    return false;
}

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

/*
Point3D A010::DepthToPoint3D(uint8_t id, uint16_t depth)
{
    Point3D p;

    p.x = distance * coefX[id];
    p.y = distance * coefY[id];
    p.z = distance * coefZ[id];

    return p;
}*/

// camera parameter : horizontal and vertical resolution, horizontal and vertical field of view in degree,  horizontal and vertical offset angle setup
void A010::ComputeCartesianCoefficient(uint16_t horRes, uint16_t verRes, float horFOVdeg, float verFOVdeg, float horOFFSETdeg, float verOFFSETdeg)
{
    uint16_t i = 0;
    float angle;
    // radians !
    float horAngRes = (horFOVdeg * DEG_TO_RAD) / (float)horRes;
    float verAngRes = (verFOVdeg * DEG_TO_RAD) / (float)verRes;
    float horAngZero = ((horFOVdeg / 2) + horOFFSETdeg) * DEG_TO_RAD;
    float verAngZero = ((verFOVdeg / 2) + verOFFSETdeg) * DEG_TO_RAD;

    // compute Horizontal => X and Y
    for (i = 0; i < horRes; i++)
    {
        angle = (horAngRes * i) - horAngZero;
        coefX[i] = QUANTIZATION_MM * sin(angle);
        coefY[i] = QUANTIZATION_MM * cos(angle);
    }
    // compute Vertical => Z
    for (i = 0; i < verRes; i++)
    {
        angle = verAngZero - (verAngRes * i);
        coefZ[i] = QUANTIZATION_MM * sin(angle);
    }
}

void A010::logCartesianCoefficient()
{
    SERIAL_DEBUG.println("***");
    for (int16_t i = 0; i < PICTURE_SIZE; i++)
    {
        int row = i % PICTURE_RES;
        int col = i / PICTURE_RES;
        String color = "65520";
        String data = String(coefX[row]) + " " + String(coefY[row]) + " " + String(coefZ[col]) + " " + color;
        SERIAL_DEBUG.println(data);
    }
    SERIAL_DEBUG.println("---");
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
