#include "A010.h"

void MetaSenseA010::Initialisation()
{
    println("Init MetaSenseA010");

    // minDistance, maxDistance
    Config(100, 1500, 2);
    SERIAL_A010.begin(115200, SERIAL_8N1, SERIAL_A010_RX, SERIAL_A010_TX);

    // wait MetaSenseA010 power up
    delay(1000);

    // pre-computing of coefficient to convert depth data to cartesian coordinate point
    ComputeCartesianCoefficient(PICTURE_RES, PICTURE_RES, FOV_HOR, FOV_VER, 0, INCLINAISON_CAM);
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
    SERIAL_A010.begin(BAUD_RATE_SPEED, SERIAL_8N1, SERIAL_A010_RX, SERIAL_A010_TX);

    // println("Init MetaSenseA010 COPY");
    //  SERIAL_A010_COPY.begin(115200, SERIAL_8N1, RX1, TX1);
}

void MetaSenseA010::Update()
{
    if (ReadSerial())
    {
        println("New frame: distance");
        CheckContinuity();

        String data = "";
        uint16_t col = 0;  // plan horizontal => curseur
        uint16_t row = 0;  // plan vertical => numéro de ligne

        // parcourir image
        for (uint16_t i = 0; i < PICTURE_SIZE; i++)
        {
            // Point4D p = {a010.cloudFrame[i].x, 10};
            // teleplot("3D", cloudFrame[i]);

            // String data = "" + String(a010.cloudFrame[i].x) + " " + String(a010.cloudFrame[i].y) + " " +
            //               String(a010.cloudFrame[i].z) + " " + String("65520");
            // println(data);

            col = i % PICTURE_RES;  // plan horizontal => curseur
            row = i / PICTURE_RES;  // plan vertical => numéro de ligne

            /*if (col >= 0 && col < PICTURE_RES && row >= 0 && row < PICTURE_RES)
            {

                cloudFrame[i].w = a010Packet.payload[i];  // * QUANTIZATION_MM;  // distance en mm
            }*/

            // *************** test algo filtrage ********************
            /*if ((d < 100) || (d > 1000))  // garder entre 10cm et 1m
            {
                d = 0;
            }*/

            //**** Fonction pour appliquer le filtre médian 3x3 ****
            // Parcourir chaque pixel de l'image (sauf les bords)
            if ((col > 0) && (col < (PICTURE_RES - 1)) && (row > 0) && (row < (PICTURE_RES - 1)))
            { /* // filtre median par tri
                 int8_t window[9];

                 // Remplir la fenêtre 3x3
                 window[0] = a010Packet.payload[i - PICTURE_RES - 1];
                 window[1] = a010Packet.payload[i - PICTURE_RES];
                 window[2] = a010Packet.payload[i - PICTURE_RES + 1];
                 window[3] = a010Packet.payload[i - 1];
                 window[4] = a010Packet.payload[i];
                 window[5] = a010Packet.payload[i + 1];
                 window[6] = a010Packet.payload[i + PICTURE_RES - 1];
                 window[7] = a010Packet.payload[i + PICTURE_RES];
                 window[8] = a010Packet.payload[i + PICTURE_RES + 1];

                // Trier la fenêtre pour trouver la médiane
                for (int k = 0; k < 8; k++)
                {
                    for (int j = 0; j < 8 - k; j++)
                    {
                        if (window[j] > window[j + 1])
                        {
                            int8_t temp = window[j];
                            window[j] = window[j + 1];
                            window[j + 1] = temp;
                        }
                    }
                }
                // Assigner la médiane au pixel de sortie
                cloudFrame[i].w = window[4];*/

                // filtre median par histogramme (a priori plus rapide)
                int8_t histogram[255] = {0};
                // Remplir l'histogramme sur la fenêtre 3x3
                histogram[a010Packet.payload[i - PICTURE_RES - 1]]++;
                histogram[a010Packet.payload[i - PICTURE_RES]]++;
                histogram[a010Packet.payload[i - PICTURE_RES + 1]]++;
                histogram[a010Packet.payload[i - 1]]++;
                histogram[a010Packet.payload[i]]++;
                histogram[a010Packet.payload[i + 1]]++;
                histogram[a010Packet.payload[i + PICTURE_RES - 1]]++;
                histogram[a010Packet.payload[i + PICTURE_RES]]++;
                histogram[a010Packet.payload[i + PICTURE_RES + 1]]++;

                // Trouver la médiane à partir de l'histogramme
                int count = 0;
                for (int h = 0; h < 255; h++)
                {
                    count += histogram[h];
                    if (count >= 5)
                    {
                        cloudFrame[i].w = h;
                        break;
                    }
                }
            }
            else
            {
                cloudFrame[i].w = 255;  // bords de l'image
            }
            // ***********************************

            // cartesian coordinate
            float tmpF = cloudFrame[i].w;
            cloudFrame[i].x = tmpF * coefX[col];
            cloudFrame[i].y = tmpF * coefY[col];
            cloudFrame[i].z = tmpF * coefZ[row] + OFFSET_Z;

            uint16_t d = cloudFrame[i].w * QUANTIZATION_MM;

            // affichage en CSV
            if (col == (PICTURE_RES - 1))
            {
                data += String(d);  // end of line
                println(data);
                data = "";
                vTaskDelay(1);
            }
            else
            {
                data += String(d) + ";";
            }
        }
        // Do stuff

        println("Same frame: z coordinate");

        data = "";
        col = 0;  // plan horizontal => curseur
        row = 0;  // plan vertical => numéro de ligne

        // parcourir image
        for (uint16_t i = 0; i < PICTURE_SIZE; i++)
        {
            col = i % PICTURE_RES;  // plan horizontal => curseur
            row = i / PICTURE_RES;  // plan vertical => numéro de ligne

            // affichage en CSV
            if (col == (PICTURE_RES - 1))
            {
                data += String(cloudFrame[i].z);  // end of line
                println(data);
                vTaskDelay(1);
                data = "";
            }
            else
            {
                data += String(cloudFrame[i].z) + ";";
            }
        }
    }
}

void MetaSenseA010::HandleCommand(Command cmd)
{
    if (cmd.cmd == ("A010"))
    {
        // a010.HandleCommand(cmd);
        //  String s = cmd.cmd;
        //  s.remove(0, 4);
        //  SERIAL_A010.write(s.c_str());
    }
}

void MetaSenseA010::Config(int min = -1, int max = -1, int discontinuity = -1)
{
    if (min != -1)
    {
        print("MetaSenseA010 Config 'Distance Min' from ", a010Config.minDistance, "", LEVEL_INFO);
        println(" to ", min, "", LEVEL_INFO);
        a010Config.minDistance = min;
    }
    if (max != -1)
    {
        print("MetaSenseA010 Config 'Distance Max' from ", a010Config.maxDistance, "", LEVEL_INFO);
        println(" to ", max, "", LEVEL_INFO);
        a010Config.maxDistance = max;
    }
    if (discontinuity != -1)
    {
        print("MetaSenseA010 Config 'Discontinuity' from ", a010Config.IDMaxDiscontinuity, "", LEVEL_INFO);
        println(" to ", discontinuity, "", LEVEL_INFO);
        a010Config.IDMaxDiscontinuity = discontinuity;
    }
}

void MetaSenseA010::InitTmpVariables()
{
    cursorTmp = 0;
    indexTmp = 0;
    packetSize = 0;
    checksum = 0;
}

bool waitEndOfPacket = true;

boolean MetaSenseA010::ReadSerial()
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
                a010Packet.payload[cursorTmp - 20] = tmpInt;
                cursorTmp++;
            }
            else if (cursorTmp == packetSize + 4)
            {
                a010Packet.frame_tail.checksum = tmpInt;
                cursorTmp++;
                checksum -= tmpInt;
                if (checksum == a010Packet.frame_tail.checksum)
                {
                    Serial.println("Checksum OK !");
                }
                else
                {
                    Serial.println();
                    Serial.print("Checksum NOK !");
                    Serial.print(a010Packet.frame_tail.checksum);
                    Serial.print(" - Calculated ");
                    Serial.print(checksum);
                    Serial.println();
                    Serial.printf("'%X\n", checksum);
                    Serial.println(SERIAL_A010.available());
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
                print("cursorTmp > packetSize + 4 + 2", LEVEL_WARN);
                print("cursorTmp : ", cursorTmp, "", LEVEL_WARN);
                println("packetSize : ", packetSize, "", LEVEL_WARN);
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
            logHeader();
        }
        else
        {
            // we should never come here !
            InitTmpVariables();
            print("cursorTmp < 0", LEVEL_WARN);
            print("cursorTmp : ", cursorTmp, "", LEVEL_WARN);
            println("packetSize : ", packetSize, "", LEVEL_WARN);
        }
    }
    return false;
}

boolean MetaSenseA010::CheckContinuity()
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
        println("a010Packet.frame_head.frame_id : ", a010Packet.frame_head.frame_id, "", LEVEL_WARN);
        println("a010LastPacketHeader.frame_id : ", a010LastPacketHeader.frame_id, "", LEVEL_WARN);
        println("Discontinuity : ", delta, "", LEVEL_WARN);
        return false;
    }
    else
    {
        return true;
    }
}

/*
Point3D MetaSenseA010::DepthToPoint3D(uint8_t id, uint16_t depth)
{
    Point3D p;

    p.x = distance * coefX[id];
    p.y = distance * coefY[id];
    p.z = distance * coefZ[id];

    return p;
}*/

// camera parameter : horizontal and vertical resolution, horizontal and vertical field of view in degree,  horizontal and vertical offset angle setup
void MetaSenseA010::ComputeCartesianCoefficient(uint16_t horRes, uint16_t verRes, float horFOVdeg, float verFOVdeg, float horOFFSETdeg, float verOFFSETdeg)
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

void MetaSenseA010::logCartesianCoefficient()
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

void MetaSenseA010::logHeader()
{
    println("frame_data_len ", a010Packet.frame_head.frame_data_len);
    println("output_mode ", a010Packet.frame_head.output_mode);
    println("senser_temp ", a010Packet.frame_head.senser_temp);
    println("driver_temp ", a010Packet.frame_head.driver_temp);
    println("exposure_time[0] ", a010Packet.frame_head.exposure_time[0]);
    println("exposure_time[1] ", a010Packet.frame_head.exposure_time[1]);
    println("exposure_time[2] ", a010Packet.frame_head.exposure_time[2]);
    println("exposure_time[3] ", a010Packet.frame_head.exposure_time[3]);
    println("error_code ", a010Packet.frame_head.error_code);
    println("resolution_rows ", a010Packet.frame_head.resolution_rows);
    println("resolution_cols ", a010Packet.frame_head.resolution_cols);
    println("frame_id ", a010Packet.frame_head.frame_id);
    println("isp_version ", a010Packet.frame_head.isp_version);
}
