#include "LD06.h"

LinkedList<uint32_t> tmpChars;

void LD06::Init()
{
    for (size_t i = 0; i < obs_length; i++)
    {
        for (size_t j = 0; j < max_data_obs; j++)
        {
            lidar_obstacle[i].data[j].angle = 0;
            lidar_obstacle[i].data[j].dist = 0;
            lidar_obstacle[i].data[j].x = 0;
            lidar_obstacle[i].data[j].y = 0;
        }
        lidar_obstacle[i].dataT = 0;
    }
    SERIAL_LIDAR.begin(230400);
}

void LD06::Read_lidar_data()
{
    bool loopFlag = true;
    uint32_t tmpInt;

    while (loopFlag)
    {
        if (SERIAL_LIDAR.available() > 0)
        {
            tmpInt = SERIAL_LIDAR.read();

            if (tmpInt == 0x54 && tmpChars.size() == 0)
            {
                tmpChars.add(tmpInt);
                continue;
            }
            else if (tmpChars.size() == TOTAL_DATA_BYTE - 1)
            {
                loopFlag = false;
                Calc_lidar_data(tmpChars);
                tmpChars.clear();
                continue;
            }
            else if (tmpChars.size() > 0)
            {
                if (tmpChars[0] == 0x54)
                {
                    tmpChars.add(tmpInt);
                }
                if (tmpChars.size() > 1)
                {
                    if (tmpChars[1] != 0x2C)
                    {
                        tmpChars.clear();
                    }
                }
            }
        }
    }
}

LD06::PacketLidar data;
void LD06::Calc_lidar_data(LinkedList<uint32_t> &values)
{
    data.header = values[0];
    data.dataLength = 0x1F & values[1];
    data.radarSpeed = values[3] << 8 | values[2];
    data.startAngle = values[5] << 8 | values[4];

    data.endAngle = values[43] << 8 | values[42];
    data.timestamp = values[45] << 8 | values[44];
    data.crcCheck = values[46];

    float packetAngle = data.endAngle - data.startAngle;
    float angleStep = (packetAngle / float(PACKSIZE - 1)); // Calculate the angle step

    for (int i = 0; i < PACKSIZE; i++)
    {
        data.dataPoint[i].angle = float(data.startAngle) + i * angleStep;
        data.dataPoint[i].confidence = (values[8 + i * 3]);
        data.dataPoint[i].distance = (int(values[8 + i * 3 - 1] << 8 | values[8 + i * 3 - 2]));
        // Print_lidar_data(data.dataPoint[i]);
    }
    Filter_lidar_data();
}

void LD06::Filter_lidar_data()
{
    int16_t dataN = 0;
    int16_t obsN = 0;
    for (int i = 0; i < PACKSIZE; i++)
    {

        auto node = data.dataPoint[i];
        int dist = node.distance;  // mm
        double angle = node.angle; // degrees

        // Ignore too close points
        if (dist < MIN_DISTANCE || node.confidence < MIN_QUALITY)
            continue;

        int robot_x = 0;
        int robot_y = 0;
        double robot_theta = 0;

        // Compute detected position
        const float lidar_x = robot_x + dist * cos((angle / 100 + robot_theta) * M_PI / 180);
        const float lidar_y = robot_y + dist * sin((angle / 100 + robot_theta) * M_PI / 180);

        // Ignore points outside of the table
        bool filterTable = false;
        if (filterTable)
        {
            const float table_margin = 0;
            if (lidar_x < table_margin || lidar_x > 2000 - table_margin ||
                lidar_y < table_margin || lidar_y > 3000 - table_margin)
                continue;
        }

        if (obsN < LD06::obs_length && dataN < LD06::max_data_obs)
        {
            // if (obsN == 0 && dataN == 0 ) "Start scan"

            // Determine if it is a new obstacle
            if (dataN > 0)
            {
                // the limit of passing to new obstacle
                if (fabsf(lidar_obstacle[obsN].data[dataN - 1].dist - dist) > 100 ||
                    fabsf(lidar_obstacle[obsN].data[dataN - 1].angle) - fabsf(angle) > 5)
                {
                    // if we have sufficient data for this obstacle
                    if (dataN >= LD06::min_data_obs)
                    {
                        // Reset the rest of the data for this obstacle
                        for (int16_t d = dataN; d < LD06::max_data_obs; d++)
                        {
                            lidar_obstacle[obsN].data[d].angle = 0;
                            lidar_obstacle[obsN].data[d].dist = 0;
                            lidar_obstacle[obsN].data[d].x = 0;
                            lidar_obstacle[obsN].data[d].y = 0;
                        }
                        lidar_obstacle[obsN].dataT = dataN;
                        obsN++;
                        dataN = 0;
                    }
                    else
                        dataN = 0;
                }
            }
            if (obsN < LD06::obs_length)
            {
                // calculate the coord of current lidar point
                lidar_obstacle[obsN].data[dataN].angle = angle;
                lidar_obstacle[obsN].data[dataN].dist = dist;
                lidar_obstacle[obsN].data[dataN].x = lidar_x;
                lidar_obstacle[obsN].data[dataN].y = lidar_y;
                // if (obstacle_debug)
                //{
                // if (dataN == 0)
                //     L::d << sl << "    Obstacle " << obsN << el;
                // L::d << sl << "        Data " << dataN << " : angle=" << angle << " dist=" << dist << " x=" << lidar_obstacle[obsN].data[dataN].x * 1000 << "mm y=" << lidar_obstacle[obsN].data[dataN].y * 1000 << "mm" << el;
                //}
                dataN++;
            }
        }
        if (dataN >= max_data_obs)
        {
            lidar_obstacle[obsN].dataT = max_data_obs;
            obsN++;
            dataN = 0;
        }

        // Print_lidar_data(node, lidar_x, lidar_y);
    }

    if (obsN == 0 && dataN == 0)
    {
        // if (obstacle_debug)
        //{
        //  L::d << sl << "Nothing found !" << el;
        //}

        // Reset all of the data
        for (int16_t o = 0; o < obs_length; o++)
        {
            for (int16_t d = 0; d < max_data_obs; d++)
            {
                lidar_obstacle[o].data[d].angle = 0;
                lidar_obstacle[o].data[d].dist = 0;
                lidar_obstacle[o].data[d].x = 0;
                lidar_obstacle[o].data[d].y = 0;
            }
            lidar_obstacle[o].dataT = 0;
        }
    }
    else
    {
        if (obsN < obs_length)
        {
            // Reset the rest of the data for the current obstacle
            for (int16_t d = dataN; d < max_data_obs; d++)
            {
                lidar_obstacle[obsN].data[d].angle = 0;
                lidar_obstacle[obsN].data[d].dist = 0;
                lidar_obstacle[obsN].data[d].x = 0;
                lidar_obstacle[obsN].data[d].y = 0;
            }
            lidar_obstacle[obsN].dataT = dataN;
            if (obsN < obs_length)
                obsN++;

            // Reset the rest of the data for the remaining obstacle
            for (int16_t o = obsN; o < obs_length; o++)
            {
                for (int16_t d = 0; d < max_data_obs; d++)
                {
                    lidar_obstacle[o].data[d].angle = 0;
                    lidar_obstacle[o].data[d].dist = 0;
                    lidar_obstacle[o].data[d].x = 0;
                    lidar_obstacle[o].data[d].y = 0;
                }
                lidar_obstacle[o].dataT = 0;
            }
        }
        for (int16_t o = 0; o < obs_length; o++)
        {
            // if we have enough point for the batch
            if (lidar_obstacle[o].dataT >= min_data_obs)
            {
                Point center = {0, 0};
                if (lidar_obstacle[o].dataT >= 3)
                {
                    // get the center with the farthest 3 points of the batch
                    Point p1 = {(lidar_obstacle[o].data[0].x), (lidar_obstacle[o].data[0].y)};
                    Point p2 = {(lidar_obstacle[o].data[lidar_obstacle[o].dataT - 1].x), (lidar_obstacle[o].data[lidar_obstacle[o].dataT - 1].y)};
                    Point p3 = {(lidar_obstacle[o].data[(int)((lidar_obstacle[o].dataT - 1) / 2)].x), (lidar_obstacle[o].data[(int)((lidar_obstacle[o].dataT - 1) / 2)].y)};
                    center = findCircle(p1, p2, p3);
                }
                // get the middle point of all the data
                Point mid = {0, 0};
                for (int16_t d = 0; d < lidar_obstacle[o].dataT; d++)
                {
                    mid.x += lidar_obstacle[o].data[d].x;
                    mid.y += lidar_obstacle[o].data[d].y;
                }
                mid.x = mid.x / lidar_obstacle[o].dataT;
                mid.y = mid.y / lidar_obstacle[o].dataT;

                // center found by circle is too far from the data
                if (fabsf(center.x - mid.x) > 0.05 || fabsf(center.y - mid.y) > 0.05)
                {
                    // SERIAL_PC.print("Obstacle mid ");
                    // SERIAL_PC.print(lidar_obstacle[o].dataT);
                    // SERIAL_PC.print(";");
                    SERIAL_PC.print(lidar_obstacle[o].data[0].angle);
                    SERIAL_PC.print(";");
                    SERIAL_PC.print(lidar_obstacle[o].data[0].dist);
                    SERIAL_PC.print(";");
                    SERIAL_PC.print(mid.x);
                    SERIAL_PC.print(";");
                    SERIAL_PC.print(mid.y);
                    SERIAL_PC.print('\n');
                    // if (obstacle_debug)
                    //     L::d << sl << "Obstacle mid " << o << " : x=" << mid.x * 1000 << "mm y=" << mid.y * 1000 << "mm" << el;
                }
                else
                {
                    // SERIAL_PC.print("Obstacle center ");
                    // SERIAL_PC.print(lidar_obstacle[o].dataT);
                    // SERIAL_PC.print(";");
                    SERIAL_PC.print(lidar_obstacle[o].data[0].angle);
                    SERIAL_PC.print(";");
                    SERIAL_PC.print(lidar_obstacle[o].data[0].dist);
                    SERIAL_PC.print(";");
                    SERIAL_PC.print(center.x);
                    SERIAL_PC.print(";");
                    SERIAL_PC.print(center.y);
                    SERIAL_PC.print('\n');
                    // if (obstacle_debug)
                    //     L::d << sl << "Obstacle center " << o << " : x=" << center.x * 1000 << "mm y=" << center.y * 1000 << "mm" << el;
                }
            }
            else
            {
                lidar_obstacle[o].dataT = 0;
            }
        }
    }
}

// Function to find the circle on
// which the given three points lie
LD06::Point LD06::findCircle(Point p1, Point p2, Point p3)
{
    return findCircle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
}
LD06::Point LD06::findCircle(float x1, float y1, float x2, float y2, float x3, float y3)
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

void LD06::Print_lidar_data(PointLidar data, float x, float y)
{
    SERIAL_PC.print(data.angle);
    SERIAL_PC.print(";");
    SERIAL_PC.print(data.distance);
    SERIAL_PC.print(";");
    SERIAL_PC.print(data.confidence);
    SERIAL_PC.print(";");
    SERIAL_PC.print(x);
    SERIAL_PC.print(";");
    SERIAL_PC.print(y);
    SERIAL_PC.print('\n');
}