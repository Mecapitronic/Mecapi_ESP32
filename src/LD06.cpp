#include "LD06.h"

Lidar::Lidar()
{
    Debugger::println("Init Lidar");

    // min_distance, max_distance, min_quality, distance_threshold, angle_threshold;
    Config(0, 500, 200, 200, 5);
    SERIAL_LIDAR.begin(230400);
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
        lidar_config.distance_threshold = distance;
    }
    if (angle != -1)
    {
        lidar_config.angle_threshold = angle;
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

        // We compare the first point of this packet with the last point of the previous packet
        // we do not care about distance and confidence as we only seek continuity in angle
        if (i == 0)
        {
            int delta = 36000;
            if (lidar_packet.dataPoint[i].angle > lidar_last_data.angle)
            {
                delta = lidar_packet.dataPoint[i].angle - lidar_last_data.angle;
                // Print_Point(lidar_packet.dataPoint[i]);
            }
            else
            {
                delta = lidar_packet.dataPoint[i].angle + (36000 - lidar_last_data.angle);
            }
            if (delta > 160) // TODO put in parameter
            {
                Debugger::log("Discontinuity detected : ", delta, " 1/100 deg", WARN);
            }
            else
            {
                // Debugger::log("Continuity OK : ", delta, " 1/100 deg", VERBOSE);
            }
        }

        // We save the last point and it will be compared with the first point to the packet
        if (i == LIDAR_DATA_PACKET_SIZE - 1)
        {
            lidar_last_data = lidar_packet.dataPoint[i];
            // Print_Point(lidar_last_data);
        }

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

void Lidar::Print_Packet()
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

void Lidar::Print_Point(PointLidar p)
{
    Debugger::print("Point ");
    Debugger::print("A:");
    Debugger::print(p.angle);
    Debugger::print(" D:");
    Debugger::print(p.distance);
    Debugger::print(" C:");
    Debugger::println(p.confidence);
}

Point Lidar::polarToCartesian(PointLidar polar_point, Robot robot)
{
    Point point;
    RobotPosition_t robot_position = robot.GetData();
    point.x = robot_position.x + polar_point.distance * cos((polar_point.angle / 100 + robot_position.angle / 100) * M_PI / 180);
    point.y = robot_position.y + polar_point.distance * sin((polar_point.angle / 100 + robot_position.angle / 100) * M_PI / 180);

    return point;
}

bool Lidar::isOutsideTable(Point point)
{
    // the margin represents the distance between the center of the obstacle
    // and the edges of the table (which is 3000mm long and 2000mm large)
    const float table_margin = 50;
    return (point.x < table_margin || point.x > 2000 - table_margin || point.y < table_margin || point.y > 3000 - table_margin);
}

bool Lidar::isOutsideTable(PointLidar polar_point, Robot robot)
{
    return isOutsideTable(polarToCartesian(polar_point, robot));
}

void Lidar::searchForObstacles(PointLidar polar_point, Tracker *tracker, Robot robot)
{
    // Ignore points outside of the table
    Point point = polarToCartesian(polar_point, robot);

    if (isOutsideTable(point))
    {
        return;
    }

    AggregatePoint(polar_point, point, tracker);
}

void Lidar::ObstacleDetected(Tracker *tracker, uint8_t size)
{
    tmp_obstacle.size = size;
    Point mid = ComputeCenter(tmp_obstacle);
    tracker->trackNewObstacle(mid);
    points_counter = 0;
}

bool Lidar::newObstacleThreshold(PointLidar polar_point)
{
    return (fabsf(tmp_obstacle.data[points_counter - 1].distance - polar_point.distance) > lidar_config.distance_threshold ||
            fabsf(tmp_obstacle.data[points_counter - 1].angle) - fabsf(polar_point.angle) > lidar_config.angle_threshold * 100);
}

void Lidar::AggregatePoint(PointLidar polar_point, Point point, Tracker *tracker)
{
    // if we have too much data for this obstacle, we move to save another obstacle
    if (points_counter >= Obstacle::kMaxPoints)
    {
        ObstacleDetected(tracker, Obstacle::kMaxPoints);
        return;
    }

    // if we are still under the maximum size of an obstacle

    // Determine if it is a new obstacle
    if (newObstacleThreshold(polar_point))
    {
        // if we have sufficient data for this obstacle
        // we move to save another obstacle
        if (points_counter >= obs_min_point)
        {
            ObstacleDetected(tracker, points_counter);
        }
    }

    // save the coord of current lidar point
    tmp_obstacle.data[points_counter++] = {(double)polar_point.angle,
                                           polar_point.distance,
                                           polar_point.confidence,
                                           point.x,
                                           point.y};
}

Point Lidar::ComputeCenter(Obstacle lidar_obstacle)
{
    Point mid = {0, 0};
    for (int8_t d = 0; d < lidar_obstacle.size; d++)
    {
        mid.x += lidar_obstacle.data[d].x;
        mid.y += lidar_obstacle.data[d].y;
    }
    mid.x = mid.x / lidar_obstacle.size;
    mid.y = mid.y / lidar_obstacle.size;

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
