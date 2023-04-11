#include "LD06.h"

namespace LD06
{
uint32_t tmpChars[TOTAL_DATA_BYTE];
uint8_t cursorTmp = 0;

Obstacle lidar_obstacle[obs_length];

void Init()
{
    for (size_t i = 0; i < obs_length; i++)
    {
        for (size_t j = 0; j < Obstacle::kMaxPoints; j++)
        {
            lidar_obstacle[i].data[j] = {0, 0, 0, 0, 0};
            // lidar_obstacle[i].data[j].angle = 0;
            // lidar_obstacle[i].data[j].distance = 0;
            // lidar_obstacle[i].data[j].confidence = 0;
            // lidar_obstacle[i].data[j].x = 0;
            // lidar_obstacle[i].data[j].y = 0;
        }
        lidar_obstacle[i].size = 0;
    }
    // we change the TX pin to 4 because the pin 10 is not available, and we do not use RX pin for now
    SERIAL_ROBOT.begin(125000, SERIAL_8N1, 9, 4);
    SERIAL_LIDAR.begin(230400);
}

void Read_lidar_data()
{
    boolean loopFlag = true;
    uint32_t tmpInt;
    cursorTmp = 0;
    while (loopFlag)
    {
        if (SERIAL_LIDAR.available() > 0)
        {
            tmpInt = SERIAL_LIDAR.read();
            // SERIAL_ROBOT.write(tmpInt);
            if (tmpInt == 0x54 && cursorTmp == 0)
            {
                tmpChars[cursorTmp++] = tmpInt;
                // continue;
            }
            else if (cursorTmp > 0)
            {
                tmpChars[cursorTmp++] = tmpInt;

                if (tmpChars[1] != 0x2C)
                {
                    cursorTmp = 0;
                }
                if (cursorTmp == TOTAL_DATA_BYTE)
                {
                    loopFlag = false;
                }
            }
        }
    }
}

PacketLidar Calc_lidar_data()
{
    PacketLidar data;
    data.header = tmpChars[0];
    data.dataLength = 0x1F & tmpChars[1];
    data.radarSpeed = tmpChars[3] << 8 | tmpChars[2];
    data.startAngle = tmpChars[5] << 8 | tmpChars[4];

    data.endAngle = tmpChars[43] << 8 | tmpChars[42];
    data.timestamp = tmpChars[45] << 8 | tmpChars[44];
    data.crcCheck = tmpChars[46];

    int angleStep = 0;

    if (data.endAngle > data.startAngle)
    {
        angleStep = (data.endAngle - data.startAngle) / (data.dataLength - 1);
    }
    else
    {
        angleStep = (data.endAngle + (360 * 100 - data.startAngle)) / (data.dataLength - 1);
    }
    for (int i = 0; i < PACKET_SIZE; i++)
    {
        int raw_deg = data.startAngle + i * angleStep;
        data.dataPoint[i].angle = (raw_deg <= 360 * 100 ? raw_deg : raw_deg - 360 * 100);
        data.dataPoint[i].confidence = (tmpChars[8 + i * 3]);
        data.dataPoint[i].distance = (int(tmpChars[8 + i * 3 - 1] << 8 | tmpChars[8 + i * 3 - 2]));
    }
    return data;
}

void Filter_lidar_data(PointLidar p[], int size)
{
    int16_t data_count = 0;
    int16_t obs_count = 0;
    const int kObsMaxPoints = Obstacle::kMaxPoints;

    for (int j = 0; j < size; j++)
    {
        const auto node = p[j];

        const int dist = node.distance;    // mm
        const double angle = node.angle;   // degrees
        const uint16_t conf = node.confidence;  // 0-255

        int robot_x = 500;
        int robot_y = 500;
        double robot_theta = 0;

        // Compute detected position
        const float lidar_x = robot_x + dist * cos((angle / 100 + robot_theta) * M_PI / 180);
        const float lidar_y = robot_y + dist * sin((angle / 100 + robot_theta) * M_PI / 180);

        // Ignore points outside of the table
        boolean filterTable = false;
        if (filterTable)
        {
            // the margin represent the distance between the center of the obstacle and the edges of the table
            const float table_margin = 0;
            if (lidar_x < table_margin || lidar_x > 2000 - table_margin || lidar_y < table_margin || lidar_y > 3000 - table_margin) continue;
        }

        if (obs_count < obs_length && data_count < kObsMaxPoints)
        {
            // Start scan

            // Determine if it is a new obstacle
            if (data_count > 0)
            {
                // the limit of passing to new obstacle
                if (fabsf(lidar_obstacle[obs_count].data[data_count - 1].distance - dist) > 100 ||
                    fabsf(lidar_obstacle[obs_count].data[data_count - 1].angle) - fabsf(angle) > 5 * 100)
                {
                    // if we have sufficient data for this obstacle
                    if (data_count >= obs_min_point)
                    {
                        // Reset the rest of the data for this obstacle
                        /*for (int16_t d = data_count; d < kObsMaxPoints; d++)
                        {
                            lidar_obstacle[obs_count].data[d].angle = 0;
                            lidar_obstacle[obs_count].data[d].distance = 0;
                            lidar_obstacle[obs_count].data[d].confidence = 0;
                            lidar_obstacle[obs_count].data[d].x = 0;
                            lidar_obstacle[obs_count].data[d].y = 0;
                        }*/
                        lidar_obstacle[obs_count].size = data_count;
                        obs_count++;
                        data_count = 0;
                    }
                    else
                        data_count = 0;
                }
            }
            if (obs_count < obs_length)
            {
                // calculate the coord of current lidar point
                lidar_obstacle[obs_count].data[data_count] = {angle, dist, conf, lidar_x, lidar_y};
                // lidar_obstacle[obs_count].data[data_count].angle = angle;
                // lidar_obstacle[obs_count].data[data_count].distance = dist;
                // lidar_obstacle[obs_count].data[data_count].confidence = conf;
                // lidar_obstacle[obs_count].data[data_count].x = lidar_x;
                // lidar_obstacle[obs_count].data[data_count].y = lidar_y;
                data_count++;
            }
        }

        if (data_count >= kObsMaxPoints)
        {
            lidar_obstacle[obs_count].size = kObsMaxPoints;
            obs_count++;
            data_count = 0;
        }
    }

    if (obs_count == 0 && data_count == 0)
    {
        // Nothing found !

        // Reset all of the data
        for (int16_t o = 0; o < obs_length; o++)
        {
            for (int16_t d = 0; d < kObsMaxPoints; d++)
            {
                lidar_obstacle[o].data[d] = {0, 0, 0, 0, 0};
                // lidar_obstacle[o].data[d].angle = 0;
                // lidar_obstacle[o].data[d].distance = 0;
                // lidar_obstacle[o].data[d].confidence = 0;
                // lidar_obstacle[o].data[d].x = 0;
                // lidar_obstacle[o].data[d].y = 0;
            }
            lidar_obstacle[o].size = 0;
        }
    }
    else
    {
        if (obs_count < obs_length)
        {
            // Reset the rest of the data for the current obstacle
            for (int16_t d = data_count; d < kObsMaxPoints; d++)
            {
                lidar_obstacle[obs_count].data[d] = {0, 0, 0, 0, 0};
                // lidar_obstacle[obs_count].data[d].angle = 0;
                // lidar_obstacle[obs_count].data[d].distance = 0;
                // lidar_obstacle[obs_count].data[d].confidence = 0;
                // lidar_obstacle[obs_count].data[d].x = 0;
                // lidar_obstacle[obs_count].data[d].y = 0;
            }
            lidar_obstacle[obs_count].size = data_count;
            if (obs_count < obs_length) obs_count++;

            // Reset the rest of the data for the remaining obstacle
            for (int16_t o = obs_count; o < obs_length; o++)
            {
                for (int16_t d = 0; d < kObsMaxPoints; d++)
                {
                    lidar_obstacle[o].data[d] = {0, 0, 0, 0, 0};
                    // lidar_obstacle[o].data[d].angle = 0;
                    // lidar_obstacle[o].data[d].distance = 0;
                    // lidar_obstacle[o].data[d].confidence = 0;
                    // lidar_obstacle[o].data[d].x = 0;
                    // lidar_obstacle[o].data[d].y = 0;
                }
                lidar_obstacle[o].size = 0;
            }
        }
        for (int16_t o = 0; o < obs_length; o++)
        {
            // if we have enough point for the batch
            if (lidar_obstacle[o].size >= obs_min_point)
            {
                Point center = {0, 0};

                /*if (lidar_obstacle[o].size >= 3)
                {
                    // get the center with the farthest 3 points of the batch
                    Point p1 = {(lidar_obstacle[o].data[0].x), (lidar_obstacle[o].data[0].y)};
                    Point p2 = {(lidar_obstacle[o].data[lidar_obstacle[o].size - 1].x),
                                (lidar_obstacle[o].data[lidar_obstacle[o].size - 1].y)};
                    Point p3 = {(lidar_obstacle[o].data[(int)((lidar_obstacle[o].size - 1) / 2)].x),
                                (lidar_obstacle[o].data[(int)((lidar_obstacle[o].size - 1) / 2)].y)};
                    center = findCircle(p1, p2, p3);
                }*/
                // get the middle point of all the data
                Point mid = {0, 0};
                for (int16_t d = 0; d < lidar_obstacle[o].size; d++)
                {
                    mid.x += lidar_obstacle[o].data[d].x;
                    mid.y += lidar_obstacle[o].data[d].y;
                }
                mid.x = mid.x / lidar_obstacle[o].size;
                mid.y = mid.y / lidar_obstacle[o].size;

                // center found by circle is too far from the data
                // if (fabsf(center.x - mid.x) > 5 || fabsf(center.y - mid.y) > 5)
                //{
                SERIAL_ROBOT.print(o);
                SERIAL_ROBOT.print(";");
                SERIAL_ROBOT.print((int)mid.x);
                SERIAL_ROBOT.print(";");
                SERIAL_ROBOT.print((int)mid.y);
                SERIAL_ROBOT.print('\n');
                SERIAL_PC.print(o);
                SERIAL_PC.print(";");
                SERIAL_PC.print((int)mid.x);
                SERIAL_PC.print(";");
                SERIAL_PC.print((int)mid.y);
                SERIAL_PC.print('\n');
                // if (obstacle_debug)
                //     L::d << sl << "Obstacle mid " << o << " : x=" << mid.x * 1000 << "mm y=" << mid.y * 1000 <<
                //     "mm" << el;
                //}
                /*else
                {
                    SERIAL_PC.print(o);
                    SERIAL_PC.print(";");
                    SERIAL_PC.print((int)center.x);
                    SERIAL_PC.print(";");
                    SERIAL_PC.print((int)center.y);
                    SERIAL_PC.print('\n');
                    // if (obstacle_debug)
                    //     L::d << sl << "Obstacle center " << o << " : x=" << center.x * 1000 << "mm y=" <<
                center.y *
                    //     1000 << "mm" << el;
                }*/
            }
            else
            {
                lidar_obstacle[o].size = 0;
            }
        }
    }
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
}  // namespace LD06

PolarPoint::PolarPoint()
{
    angle = 0;
    distance = 0;
    confidence = 0;
    x = 0;
    y = 0;
}

PolarPoint::PolarPoint(double _angle, int _distance, uint16_t _confidence, float _x, float _y)
{
    angle = _angle;
    distance = _distance;
    confidence = _confidence;
    x = _x;
    y = _y;
}

void PolarPoint::Print(HardwareSerial s, PolarPoint p, bool debug)
{
    if (debug) s.print("Angle:");
    s.print((int)p.angle);
    if (debug)
        s.print(" Distance:");
    else
        s.print(";");
    s.print(p.distance);
    if (debug)
        s.print(" Confidence:");
    else
        s.print(";");
    s.println(p.confidence);
}

Point::Point()
{
    x = 0;
    y = 0;
}

Point::Point(float _x, float _y)
{
    x = _x;
    y = _y;
}

void Point::Print(HardwareSerial s, Point p, bool debug)
{
    if (debug) s.print("X:");
    s.print((int)p.x);
    if (debug)
        s.print(" Y:");
    else
        s.print(";");
    s.print((int)p.y);
}