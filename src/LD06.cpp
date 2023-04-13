#include "LD06.h"

namespace LD06
{
    PacketLidar lidar;
    uint32_t tmpChars[serial_packet_size];
    uint8_t cursorTmp = 0;
    ConfigLidar config;
    Obstacle lidar_obstacle[obs_length];

    void Init()
    {
        for (size_t i = 0; i < obs_length; i++)
        {
            for (size_t j = 0; j < Obstacle::kMaxPoints; j++)
            {
                lidar_obstacle[i].data[j].angle = 0;
                lidar_obstacle[i].data[j].distance = 0;
                lidar_obstacle[i].data[j].confidence = 0;
                lidar_obstacle[i].data[j].x = 0;
                lidar_obstacle[i].data[j].y = 0;
            }
            lidar_obstacle[i].size = 0;
        }
        Config(0, 500, 200, 200, 5);
        SERIAL_LIDAR.begin(230400);
    }

    void Config(int min, int max, int quality, int distance, int angle)
    {
        if (config.min_distance != -1)
            config.min_distance = min;

        if (config.max_distance != -1)
            config.max_distance = max;

        if (config.min_quality != -1)
            config.min_quality = quality;

        if (config.obs_distance != -1)
            config.obs_distance = distance;

        if (config.obs_angle != -1)
            config.obs_angle = angle;
    }

    boolean ReadSerial()
    {
        if (SERIAL_LIDAR.available() > 0)
        {
            uint32_t tmpInt = SERIAL_LIDAR.read();
            if (tmpInt == 0x54 && cursorTmp == 0)
            {
                tmpChars[cursorTmp++] = tmpInt;
            }
            else if (cursorTmp > 0)
            {
                tmpChars[cursorTmp++] = tmpInt;

                if (tmpChars[1] != 0x2C)
                {
                    cursorTmp = 0;
                }
                if (cursorTmp == serial_packet_size)
                {
                    cursorTmp = 0;
                    return true;
                }
            }
        }
        return false;
    }

    void Analyze()
    {
        lidar.header = tmpChars[0];
        lidar.dataLength = 0x1F & tmpChars[1];
        lidar.radarSpeed = tmpChars[3] << 8 | tmpChars[2];
        lidar.startAngle = tmpChars[5] << 8 | tmpChars[4];

        lidar.endAngle = tmpChars[43] << 8 | tmpChars[42];
        lidar.timestamp = tmpChars[45] << 8 | tmpChars[44];
        lidar.crcCheck = tmpChars[46];

        int angleStep = 0;

        if (lidar.endAngle > lidar.startAngle)
        {
            angleStep = (lidar.endAngle - lidar.startAngle) / (lidar.dataLength - 1);
        }
        else
        {
            angleStep = (lidar.endAngle + (360 * 100 - lidar.startAngle)) / (lidar.dataLength - 1);
        }
        for (int i = 0; i < data_packet_size; i++)
        {
            int raw_deg = lidar.startAngle + i * angleStep;
            lidar.dataPoint[i].angle = (raw_deg <= 360 * 100 ? raw_deg : raw_deg - 360 * 100);
            lidar.dataPoint[i].confidence = (tmpChars[8 + i * 3]);
            lidar.dataPoint[i].distance = (int(tmpChars[8 + i * 3 - 1] << 8 | tmpChars[8 + i * 3 - 2]));

            // if the point is out of bound, we will not use it
            if (lidar.dataPoint[i].distance < config.min_distance || lidar.dataPoint[i].distance > config.max_distance || lidar.dataPoint[i].confidence < config.min_quality)
            {
                lidar.dataPoint[i].confidence = 0;
            }
        }
    }

    PacketLidar GetData() { return lidar; }

    void Print()
    {
        SERIAL_PC.print("dataLength:");
        SERIAL_PC.print(lidar.dataLength);
        SERIAL_PC.print(" radarSpeed:");
        SERIAL_PC.print(lidar.radarSpeed);
        SERIAL_PC.print(" startAngle:");
        SERIAL_PC.print(lidar.startAngle);
        SERIAL_PC.print(" endAngle: ");
        SERIAL_PC.print(lidar.endAngle);
        SERIAL_PC.print(" timestamp: ");
        SERIAL_PC.println(lidar.timestamp);
        for (size_t i = 0; i < data_packet_size; i++)
        {
            SERIAL_PC.print("   Point ");
            SERIAL_PC.print(i);
            SERIAL_PC.print(") ");
            SERIAL_PC.print("A:");
            SERIAL_PC.print(lidar.dataPoint[i].angle);
            SERIAL_PC.print(" D:");
            SERIAL_PC.print(lidar.dataPoint[i].distance);
            SERIAL_PC.print(" C:");
            SERIAL_PC.println(lidar.dataPoint[i].confidence);
        }
    }

    int16_t data_count = 0;
    int16_t obs_count = 0;
    void AggregatePoint(PointLidar point)
    {
        const int kObsMaxPoints = Obstacle::kMaxPoints;
        Robot_t robot = Robot::GetData();

        // Compute detected position
        const float lidar_x = robot.x + point.distance * cos((point.angle / 100 + robot.angle / 100) * M_PI / 180);
        const float lidar_y = robot.y + point.distance * sin((point.angle / 100 + robot.angle / 100) * M_PI / 180);

        // Ignore points outside of the table
        boolean filterTable = false;
        if (filterTable)
        {
            // the margin represent the distance between the center of the obstacle and the edges of the table
            const float table_margin = 50;
            if (lidar_x < table_margin || lidar_x > 2000 - table_margin || lidar_y < table_margin || lidar_y > 3000 - table_margin)
                return;
        }

        if (obs_count < obs_length && data_count < kObsMaxPoints)
        {
            // Determine if it is a new obstacle
            if (data_count > 0)
            {
                // the limit of passing to new obstacle
                if (fabsf(lidar_obstacle[obs_count].data[data_count - 1].distance - point.distance) > config.obs_distance ||
                    fabsf(lidar_obstacle[obs_count].data[data_count - 1].angle) - fabsf(point.angle) > config.obs_angle * 100)
                {
                    // if we have sufficient data for this obstacle, we move to save another obstacle
                    if (data_count >= obs_min_point)
                    {
                        lidar_obstacle[obs_count].size = data_count;
                        ComputeCenter();
                        obs_count++;
                        data_count = 0;
                    }
                    else
                        data_count = 0;
                }
            }
            if (obs_count < obs_length)
            {
                // save the coord of current lidar point
                lidar_obstacle[obs_count].data[data_count].angle = point.angle;
                lidar_obstacle[obs_count].data[data_count].distance = point.distance;
                lidar_obstacle[obs_count].data[data_count].confidence = point.confidence;
                lidar_obstacle[obs_count].data[data_count].x = lidar_x;
                lidar_obstacle[obs_count].data[data_count].y = lidar_y;
                data_count++;
            }
        }

        // if we have too much data for this obstacle, we move to save another obstacle
        if (data_count >= kObsMaxPoints)
        {
            lidar_obstacle[obs_count].size = kObsMaxPoints;
            ComputeCenter();
            obs_count++;
            data_count = 0;
        }
        if (obs_count >= obs_length)
        {
            obs_count = 0;
        }
    }

    void ComputeCenter()
    {
        // get the middle point of all the data
        Point mid = {0, 0};
        for (int16_t d = 0; d < lidar_obstacle[obs_count].size; d++)
        {
            mid.x += lidar_obstacle[obs_count].data[d].x;
            mid.y += lidar_obstacle[obs_count].data[d].y;
        }
        mid.x = mid.x / lidar_obstacle[obs_count].size;
        mid.y = mid.y / lidar_obstacle[obs_count].size;

        Robot::WriteSerial(obs_count, mid.x, mid.y);

        SERIAL_PC.print(obs_count);
        SERIAL_PC.print(";");
        SERIAL_PC.print((int)mid.x);
        SERIAL_PC.print(";");
        SERIAL_PC.print((int)mid.y);
        SERIAL_PC.print('\n');
    }

    // Function to find the circle on
    // which the given three points lie
    Point findCircle(Point p1, Point p2, Point p3) { return findCircle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y); }
    Point findCircle(float x1, float y1, float x2, float y2, float x3, float y3)
    {
        float x12 = x1 - x2;
        float x13 = x1 - x3;

        float y12 = y1 - y2;
        float y13 = y1 - y3;

        float y31 = y3 - y1;
        float y21 = y2 - y1;

        float x31 = x3 - x1;
        float x21 = x2 - x1;

        // x1^2 - x3^2
        float sx13 = pow(x1, 2) - pow(x3, 2);

        // y1^2 - y3^2
        float sy13 = pow(y1, 2) - pow(y3, 2);

        float sx21 = pow(x2, 2) - pow(x1, 2);
        float sy21 = pow(y2, 2) - pow(y1, 2);

        float f = ((sx13) * (x12) + (sy13) * (x12) + (sx21) * (x13) + (sy21) * (x13)) / (2 * ((y31) * (x12) - (y21) * (x13)));
        float g = ((sx13) * (y12) + (sy13) * (y12) + (sx21) * (y13) + (sy21) * (y13)) / (2 * ((x31) * (y12) - (x21) * (y13)));

        // float c = -pow(x1, 2) - pow(y1, 2) - 2 * g * x1 - 2 * f * y1;

        // eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
        // where centre is (h = -g, k = -f) and radius r
        // as r^2 = h^2 + k^2 - c
        float h = -g;
        float k = -f;
        // float sqr_of_r = h * h + k * k - c;

        // r is the radius
        // float r = sqrt(sqr_of_r);

        // cout << "Centre = (" << h << ", " << k << ")" << endl;
        // cout << "Radius = " << r;
        Point center = {h, k};
        return center;
    }
} // namespace LD06
