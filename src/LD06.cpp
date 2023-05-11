#include "LD06.h"

Lidar::Lidar()
{
    Debugger::println("Init Lidar");

    // minDistance, maxDistance, minQuality, distanceThreshold, angleThreshold;
    Config(0, 1000, 200, 50, 0.8 * 5);
    SERIAL_LIDAR.begin(230400);
}

void Lidar::Config(int min = -1, int max = -1, int quality = -1, int distance = -1, int angle = -1)
{
    if (min != -1)
    {
        lidarConfig.minDistance = min;
    }
    if (max != -1)
    {
        lidarConfig.maxDistance = max;
    }
    if (quality != -1)
    {
        lidarConfig.minQuality = quality;
    }
    if (distance != -1)
    {
        lidarConfig.distanceThreshold = distance;
    }
    if (angle != -1)
    {
        lidarConfig.angleThreshold = angle;
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
            serialBuffer[cursorTmp++] = tmpInt;
        }
        else if (cursorTmp > 0)
        {
            serialBuffer[cursorTmp++] = tmpInt;

            if (serialBuffer[1] != 0x2C)
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
    lidarPacket.header = serialBuffer[0];
    lidarPacket.dataLength = 0x1F & serialBuffer[1];
    lidarPacket.radarSpeed = serialBuffer[3] << 8 | serialBuffer[2];
    lidarPacket.startAngle = serialBuffer[5] << 8 | serialBuffer[4];

    lidarPacket.endAngle = serialBuffer[43] << 8 | serialBuffer[42];
    lidarPacket.timestamp = serialBuffer[45] << 8 | serialBuffer[44];
    lidarPacket.crcCheck = serialBuffer[46];

    int angleStep = 0;
    // fix angle step to negative if we cross 0° during scan
    // which means first angle is bigger than the last one
    // else positive
    if (lidarPacket.endAngle > lidarPacket.startAngle)
    {
        angleStep = (lidarPacket.endAngle - lidarPacket.startAngle) / (lidarPacket.dataLength - 1);
    }
    else
    {
        angleStep = (lidarPacket.endAngle + (360 * 100 - lidarPacket.startAngle)) / (lidarPacket.dataLength - 1);
    }

    // compute lidar result with previously defined angle step
    for (int i = 0; i < LIDAR_DATA_PACKET_SIZE; i++)
    {
        int rawDeg = lidarPacket.startAngle + i * angleStep;
        // Raw angle are inverted
        lidarPacket.dataPoint[i].angle = 360 * 100 - (rawDeg <= 360 * 100 ? rawDeg : rawDeg - 360 * 100);
        lidarPacket.dataPoint[i].confidence = (serialBuffer[8 + i * 3]);
        lidarPacket.dataPoint[i].distance = (int(serialBuffer[8 + i * 3 - 1] << 8 | serialBuffer[8 + i * 3 - 2]));

        // if the point is out of bound, we will not use it
        if (lidarPacket.dataPoint[i].distance < lidarConfig.minDistance ||
            lidarPacket.dataPoint[i].distance > lidarConfig.maxDistance ||
            lidarPacket.dataPoint[i].confidence < lidarConfig.minQuality)
        {
            lidarPacket.dataPoint[i].confidence = 0;
        }
    }
}

boolean Lidar::CheckContinuity()
{
    // We compare the first point of this packet with the last point of the previous packet
    // we do not care about distance and confidence as we only seek continuity in angle
    int delta = lidarLastPacket.dataPoint[LIDAR_DATA_PACKET_SIZE - 1].angle - lidarPacket.dataPoint[0].angle;
    // previous packet was before 0° and current after
    if (lidarLastPacket.dataPoint[LIDAR_DATA_PACKET_SIZE - 1].angle <= lidarPacket.dataPoint[0].angle)
    {
        delta += 36000;
    }

    // save the last point to compare to the next packet's first point
    lidarLastPacket = lidarPacket;

    if (delta > angleMaxDiscontinuity)
    {
        Debugger::log("Discontinuity : ", (float)delta / 100, " deg", WARN);
        return false;
    }
    else
    {
        return true;
    }
}

PacketLidar Lidar::GetData() { return lidarPacket; }

void Lidar::PrintPacket(PacketLidar packet)
{
    Debugger::print("dataLength:");
    Debugger::print(packet.dataLength);
    Debugger::print(" radarSpeed:");
    Debugger::print(packet.radarSpeed);
    Debugger::print(" startAngle:");
    Debugger::print(packet.startAngle);
    Debugger::print(" endAngle: ");
    Debugger::print(packet.endAngle);
    Debugger::print(" timestamp: ");
    Debugger::println(packet.timestamp);
    for (uint8_t i = 0; i < LIDAR_DATA_PACKET_SIZE; i++)
    {
        Debugger::print("   Point ");
        Debugger::print(i);
        Debugger::print(") ");
        Debugger::print("A:");
        Debugger::print(packet.dataPoint[i].angle);
        Debugger::print(" D:");
        Debugger::print(packet.dataPoint[i].distance);
        Debugger::print(" C:");
        Debugger::println(packet.dataPoint[i].confidence);
    }
}

void Lidar::PrintPoint(PointLidar point)
{
    Debugger::print("Point ");
    Debugger::print("A:");
    Debugger::print(point.angle);
    Debugger::print(" D:");
    Debugger::print(point.distance);
    Debugger::print(" C:");
    Debugger::println(point.confidence);
}

Point Lidar::PolarToCartesian(PointLidar polar_point, Robot robot)
{
    Point point;
    RobotPosition_t robotPosition = robot.GetPosition();
    float angle = polar_point.angle + robotPosition.angle;
    angle /= 100;
    point.x = robotPosition.x + polar_point.distance * cos(angle * M_PI / 180);
    point.y = robotPosition.y + polar_point.distance * sin(angle * M_PI / 180);

    return point;
}

bool Lidar::IsOutsideTable(Point point)
{
    // the margin represents the distance between the center of the obstacle
    // and the edges of the table (which is 3000mm long and 2000mm large)
    const float table_margin = 50;
    return (point.x < table_margin || point.x > 2000 - table_margin || point.y < table_margin || point.y > 3000 - table_margin);
}

void Lidar::SearchForObstacles(PointLidar polar_point, Tracker *tracker, Robot robot)
{
    // Ignore points outside of the table
    Point point = PolarToCartesian(polar_point, robot);

    if (IsOutsideTable(point))
    {
        return;
    }

    AggregatePoint(polar_point, point, tracker);
}

void Lidar::ObstacleDetected(Tracker *tracker, uint8_t size)
{
    obstacleTmp.size = size;
    Point mid = ComputeCenter(obstacleTmp);
    tracker->track(mid, obstacleTmp.data, obstacleTmp.size);
    pointsCounter = 0;
}

bool Lidar::NewObstacleThreshold(PointLidar polar_point)
{
    return (fabsf(obstacleTmp.data[pointsCounter - 1].distance - polar_point.distance) > lidarConfig.distanceThreshold ||
            fabsf(obstacleTmp.data[pointsCounter - 1].angle) - fabsf(polar_point.angle) > lidarConfig.angleThreshold * 100);
}

void Lidar::AggregatePoint(PointLidar polar_point, Point point, Tracker *tracker)
{
    // if we have too much data for this obstacle, we move to save another obstacle
    if (pointsCounter >= Obstacle::kMaxPoints)
    {
        ObstacleDetected(tracker, Obstacle::kMaxPoints);
        return;
    }

    // if we are still under the maximum size of an obstacle

    // Determine if it is a new obstacle
    if (NewObstacleThreshold(polar_point))
    {
        // if we have sufficient data for this obstacle
        // we move to save another obstacle
        if (pointsCounter >= obstacleMinPoints)
        {
            ObstacleDetected(tracker, pointsCounter);
        }
    }

    // save the coord of current lidar point
    obstacleTmp.data[pointsCounter++] = {(double)polar_point.angle,
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

Point Lidar::FindCircle(Point p1, Point p2, Point p3) { return FindCircle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y); }

Point Lidar::FindCircle(float x1, float y1, float x2, float y2, float x3, float y3)
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
