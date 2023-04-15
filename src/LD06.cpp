#include "LD06.h"

Lidar::Lidar(void)
{
    for (size_t i = 0; i < obs_length; i++)
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
}

void Lidar::Config(int min, int max, int quality, int distance, int angle)
{

    if (min != -1)
    {
        lidar_config.min_distance = min;
    }
    if (max != -1)
    {
        lidar_config.max_distance = max;
    }
    if (quality != -1)
    {
        lidar_config.min_quality = quality;
    }
    if (distance != -1)
    {
        lidar_config.obs_distance = distance;
    }
    if (angle != -1)
    {
        lidar_config.obs_angle = angle;
    }
}

boolean Lidar::ReadSerial()
{
    while (SERIAL_LIDAR.available() > 0)
    {
        // we are building a lidar packet
        // it always start with 0x54 0x2C and is 47 bytes long
        uint32_t tmpInt = SERIAL_LIDAR.read();
        if (tmpInt == 0x54 && cursorTmp == 0)
        {
            serial_buffer[cursorTmp++] = tmpInt;
        }
        else if (cursorTmp > 0)
        {
            serial_buffer[cursorTmp++] = tmpInt;

            if (serial_buffer[1] != 0x2C)
            {
                cursorTmp = 0;
            }
            if (cursorTmp == LIDAR_SERIAL_PACKET_SIZE)
            {
                cursorTmp = 0;
                return true;
            }
        }
    }
    return false;
}

void Lidar::Analyze()
{
    lidar_packet.header = serial_buffer[0];
    lidar_packet.dataLength = 0x1F & serial_buffer[1];
    lidar_packet.radarSpeed = serial_buffer[3] << 8 | serial_buffer[2];
    lidar_packet.startAngle = serial_buffer[5] << 8 | serial_buffer[4];

    lidar_packet.endAngle = serial_buffer[43] << 8 | serial_buffer[42];
    lidar_packet.timestamp = serial_buffer[45] << 8 | serial_buffer[44];
    lidar_packet.crcCheck = serial_buffer[46];

    int angleStep = 0;
    // fix angle step to negative if we cross 0° during scan
    // which means first angle is bigger than the last one
    // else positive
    if (lidar_packet.endAngle > lidar_packet.startAngle)
    {
        angleStep = (lidar_packet.endAngle - lidar_packet.startAngle) / (lidar_packet.dataLength - 1);
    }
    else
    {
        angleStep = (lidar_packet.endAngle + (360 * 100 - lidar_packet.startAngle)) / (lidar_packet.dataLength - 1);
    }

    // compute lidar result with previously defined angle step
    for (int i = 0; i < LIDAR_DATA_PACKET_SIZE; i++)
    {
        int raw_deg = lidar_packet.startAngle + i * angleStep;
        lidar_packet.dataPoint[i].angle = (raw_deg <= 360 * 100 ? raw_deg : raw_deg - 360 * 100);
        lidar_packet.dataPoint[i].confidence = (serial_buffer[8 + i * 3]);
        lidar_packet.dataPoint[i].distance = (int(serial_buffer[8 + i * 3 - 1] << 8 | serial_buffer[8 + i * 3 - 2]));

        // if the point is out of bound, we will not use it
        if (lidar_packet.dataPoint[i].distance < lidar_config.min_distance ||
            lidar_packet.dataPoint[i].distance > lidar_config.max_distance ||
            lidar_packet.dataPoint[i].confidence < lidar_config.min_quality)
        {
            lidar_packet.dataPoint[i].confidence = 0;
        }
    }
}

PacketLidar Lidar::GetData() { return lidar_packet; }

void Lidar::Print()
{
    Debugger::print("dataLength:");
    Debugger::print(lidar_packet.dataLength);
    Debugger::print(" radarSpeed:");
    Debugger::print(lidar_packet.radarSpeed);
    Debugger::print(" startAngle:");
    Debugger::print(lidar_packet.startAngle);
    Debugger::print(" endAngle: ");
    Debugger::print(lidar_packet.endAngle);
    Debugger::print(" timestamp: ");
    Debugger::println(lidar_packet.timestamp);
    for (uint8_t i = 0; i < LIDAR_DATA_PACKET_SIZE; i++)
    {
        Debugger::print("   Point ");
        Debugger::print(i);
        Debugger::print(") ");
        Debugger::print("A:");
        Debugger::print(lidar_packet.dataPoint[i].angle);
        Debugger::print(" D:");
        Debugger::print(lidar_packet.dataPoint[i].distance);
        Debugger::print(" C:");
        Debugger::println(lidar_packet.dataPoint[i].confidence);
    }
}

void Lidar::AggregatePoint(Robot robot, PointLidar point)
{
    RobotPosition_t robot_position = robot.GetData();
    // Compute detected position
    const float lidar_x = robot_position.x + point.distance * cos((point.angle / 100 + robot_position.angle / 100) * M_PI / 180);
    const float lidar_y = robot_position.y + point.distance * sin((point.angle / 100 + robot_position.angle / 100) * M_PI / 180);

    // Ignore points outside of the table
    // the margin represent the distance between the center of the obstacle and the edges of the table
    const float table_margin = 50;
    if (lidar_x < table_margin || lidar_x > 2000 - table_margin || lidar_y < table_margin || lidar_y > 3000 - table_margin)
    {
        return;
    }

    if (obs_count < obs_length && data_count < Obstacle::kMaxPoints)
    {
        // Determine if it is a new obstacle
        if (data_count > 0)
        {
            // the limit of passing to new obstacle
            if (fabsf(lidar_obstacle[obs_count].data[data_count - 1].distance - point.distance) > lidar_config.obs_distance ||
                fabsf(lidar_obstacle[obs_count].data[data_count - 1].angle) - fabsf(point.angle) > lidar_config.obs_angle * 100)
            {
                // if we have sufficient data for this obstacle, we move to save another obstacle
                if (data_count >= obs_min_point)
                {
                    lidar_obstacle[obs_count].size = data_count;
                    Point mid = ComputeCenter();
                    robot.WriteSerial(obs_count, mid.x, mid.y);
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
    if (data_count >= Obstacle::kMaxPoints)
    {
        lidar_obstacle[obs_count].size = Obstacle::kMaxPoints;
        Point mid = ComputeCenter();
        robot.WriteSerial(obs_count, mid.x, mid.y);

        obs_count++;
        data_count = 0;
    }
    if (obs_count >= obs_length)
    {
        obs_count = 0;
    }
}

Point Lidar::ComputeCenter()
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

    Debugger::log(">obstacle center: x:", mid.x, " |", VERBOSE, false);
    Debugger::log("y:", mid.y, "", VERBOSE, true);

    return mid;
}

Point Lidar::findCircle(Point p1, Point p2, Point p3) { return findCircle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y); }

Point Lidar::findCircle(float x1, float y1, float x2, float y2, float x3, float y3)
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
