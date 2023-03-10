#include "LD06Driver.h"

int frameCounter = 0;
bool printDebug = false;
int dataLength = 0;
LidarPacket lidarPacket;

LinkedList<PointData> listPointData;
LinkedList<byte> buffer;

void LD06Driver::Initialisation() {
    frameCounter = 0;
    lidarPacket.Initialisation();
    listPointData.clear();
    buffer.clear();
    _serialPortPC.begin(115200);
    _serialPortPC.write("Start PC! ");
    _serialPortLidar.begin(230400);
    _serialPortPC.write("Start Lidar ! ");
}
// byte test2[6000];
int pointeur = 0;
void LD06Driver::ReadLidarData() {
    if (_serialPortLidar.available() > 0) {
        /*int av = _serialPort.available();
        // Fetch single byte from serial
        byte dataBytes[av];
        int available = _serialPort.readBytes(dataBytes, av);
        PRINT(av);
        PRINT(_serialPort.available());
        PRINT(buffer.size());
        for (int i = 0; i < available; i++) {
            buffer.add(dataBytes[i]);
        }*/
        //}
        /*
            for (int i = 0; i < 47; i++) {
                buffer.add(test[i]);
            }*/

        // if (_serialPort.available()) {
        // while (buffer.size() > 0) {
        // uint32_t dataByte = buffer.remove(0);
        // uint32_t dataByte = _serialPort.read();
        /*test2[pointeur++] = _serialPort.read();
        if (pointeur >= 6000) {
            pointeur = 0;
            for (int i = 0; i < 6000; i++) {
                _serialPortPC.print("0x");
                _serialPortPC.print(test2[i] < 0x10 ? "0" : "");
                _serialPortPC.print(test2[i], HEX);
                _serialPortPC.print(", ");
            }
        }*/

        uint32_t dataByte = _serialPortLidar.read();
        // PRINT(dataByte);
        /*
         * starting character：Length 1 Byte, fixed value 0x54, means the
         * beginning of data packet;
         */
        if (dataByte == 0x54 && frameCounter == 0) {
            frameCounter = 1;  // go to data length
            lidarPacket.Initialisation();
            lidarPacket.header = dataByte;
            lidarPacket.packetRaw.add(dataByte);
        }

        /*
         * Data Length: Length 1 Byte, the first three digits reserved, the
         * last five digits represent the number of measured points in a
         * packet, currently fixed value 12;
         */
        // data length
        else if (frameCounter == 1) {
            if (dataByte == 0x2C) {
                dataLength = 0;
                lidarPacket.dataLength = dataByte & 0x0F;
                lidarPacket.packetRaw.add(dataByte);
                frameCounter++;
                // PRINT(lidarPacket.dataLength);
            } else {
                frameCounter = 0;
            }
        }

        /*
         * Radar speed：Length 2 Byte, in degrees per second;
         */
        // Radar Speed /LSB
        else if (frameCounter == 2) {
            lidarPacket.radarSpeed |= dataByte;
            lidarPacket.packetRaw.add(dataByte);
            frameCounter++;
        }

        // Radar Speed /MSB
        else if (frameCounter == 3) {
            lidarPacket.radarSpeed |= dataByte << 8;
            lidarPacket.packetRaw.add(dataByte);
            frameCounter++;
            // PRINT(lidarPacket.radarSpeed);
        }

        /*
         * Start angle: Length: 2 Byte; unit: 0.01 degree;
         */
        // Start angle /LSB
        else if (frameCounter == 4) {
            lidarPacket.startAngle |= dataByte;
            lidarPacket.packetRaw.add(dataByte);
            frameCounter++;
        }

        // Start angle /MSB
        else if (frameCounter == 5) {
            lidarPacket.startAngle |= dataByte << 8;
            lidarPacket.packetRaw.add(dataByte);
            frameCounter++;
            // PRINT(lidarPacket.startAngle);
        }

        /*
         * A measurement data length is 3 bytes
         */
        // Data
        else if (frameCounter == 6) {
            // Add raw data to the lidar packet
            // 3 bytes in each data bin
            // PRINT(dataLength);
            // PRINT(lidarPacket.dataLength);
            if (dataLength < lidarPacket.dataLength * 3) {
                lidarPacket.data.add(dataByte);  // read angle data character
                lidarPacket.packetRaw.add(dataByte);
                dataLength++;
            }

            if (dataLength >= lidarPacket.dataLength * 3) {
                frameCounter++;
            }
        }

        /*
         * End Angle: Length: 2 Byte; unit: 0.01 degree
         */
        // End angle /LSB
        else if (frameCounter == 7) {
            lidarPacket.endAngle |= dataByte;
            lidarPacket.packetRaw.add(dataByte);
            frameCounter++;
        }

        // End angle /MSB
        else if (frameCounter == 8) {
            lidarPacket.endAngle |= dataByte << 8;
            lidarPacket.packetRaw.add(dataByte);
            frameCounter++;
            // PRINT(lidarPacket.endAngle);
        }

        /*
         * Timestamp: Length 2 Bytes in ms, recount if reaching to MAX 30000
         */
        // Timesamp /LSB
        else if (frameCounter == 9) {
            lidarPacket.timestamp |= dataByte;
            lidarPacket.packetRaw.add(dataByte);
            frameCounter++;
        }

        // Timestamp /MSB
        else if (frameCounter == 10) {
            lidarPacket.timestamp |= dataByte << 8;
            lidarPacket.packetRaw.add(dataByte);
            frameCounter++;
            // PRINT(lidarPacket.timestamp);
        }

        /*
         * CRC check：Checksum of all previous data
         */
        // CRC Check
        else if (frameCounter == 11) {
            lidarPacket.crcCheck = dataByte;
            lidarPacket.packetRaw.add(dataByte);

            frameCounter = 0;  // Reset frame counter

            byte crcCalculated = lidarPacket.CalCRC8();
/*
            for (int i = 0; i < lidarPacket.packetRaw.size(); i++) {
                _serialPortPC.print("0x");
                _serialPortPC.print(lidarPacket.packetRaw[i] < 16 ? "0" : "");
                _serialPortPC.print(lidarPacket.packetRaw[i], HEX);
                _serialPortPC.print(", ");
            }
            _serialPortPC.println();
*/
            // Debug output
            if (crcCalculated == lidarPacket.crcCheck) {
                listPointData.clear();

                float step =
                    (((float)lidarPacket.endAngle - lidarPacket.startAngle) / 100) / (lidarPacket.dataLength - 1);
                // PRINT(step);
                //  calculate angle values for every 3 bytes
                for (int i = 0; i < lidarPacket.dataLength; i++) {
                    PointData newPoint;

                    newPoint.distance |= lidarPacket.data[i * 3];
                    newPoint.distance |= lidarPacket.data[i * 3 + 1] << 8;
                    newPoint.confidence = lidarPacket.data[i * 3 + 2];
                    newPoint.angle = ((float)lidarPacket.startAngle / 100) + step * i;
                    newPoint.timestamp = lidarPacket.timestamp;

                    // Add point to the output
                    listPointData.add(newPoint);
                    // PRINT(newPoint.distance);
                    // PRINT(newPoint.angle);
                    // PRINT(newPoint.confidence);
                }

            } else {
                _serialPortPC.print("------------------------------> CRC Error : ");
                _serialPortPC.print(lidarPacket.crcCheck);
                _serialPortPC.print(" <> ");
                _serialPortPC.println(crcCalculated);
            }

            if (printDebug) {
                _serialPortPC.print("Data Length: ");
                _serialPortPC.println(lidarPacket.dataLength);
                _serialPortPC.print("Speed deg/sec: ");
                _serialPortPC.println(lidarPacket.radarSpeed);
                _serialPortPC.print("Start Angle: ");
                _serialPortPC.println((double)lidarPacket.startAngle / 100);
                _serialPortPC.print("End Angle: ");
                _serialPortPC.println((double)lidarPacket.endAngle / 100);
                _serialPortPC.print("Timestamp: ");
                _serialPortPC.println(lidarPacket.timestamp);
                _serialPortPC.print("CRC Check: ");
                _serialPortPC.println(lidarPacket.crcCheck);
                _serialPortPC.print("CRC Calc: ");
                _serialPortPC.println(crcCalculated);
            }
        }
    }
    // else {
    //    _serialPortPC.write("Lidar KO ! ");
    //    delay(500);
    //}
}
