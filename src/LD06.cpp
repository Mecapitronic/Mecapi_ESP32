#include "LD06.h"

void LidarLD06::Initialisation()
{
    println("Init LidarLD06");
    scan.clear();
    // minDistance, maxDistance, minQuality, distanceThreshold, angleThreshold, countThreshold;
    Config(100, 1500, 200, 200, 0.8 * 5, 2);
    SERIAL_LIDAR.begin(230400);

    int pwmChannel = 0;    // Choisit le canal 0
    int frequence = 30000; // Fréquence PWM de 30 KHz
    int resolution = 8;    // Résolution de 8 bits, 256 valeurs possibles
    int pwmPin = 23;

    // Configuration du canal 0 avec la fréquence et la résolution choisie
    ledcSetup(pwmChannel, frequence, resolution);

    // Assigne le canal PWM à la pin choisie
    ledcAttachPin(pwmPin, pwmChannel);

    // Créer le rapport cyclique en fonction de la résolution
    uint32_t max_duty = (1 << resolution) - 1;
    uint32_t duty = max_duty * 25 / 100;

    ledcWrite(pwmChannel, duty);
}

void LidarLD06::Config(int min = -1, int max = -1, int quality = -1, int distance = -1, int angle = -1, int count = -1)
{
    if (min != -1)
    {
        print("LidarLD06 Config 'Distance Min' from ", lidarConfig.minDistance, "",
              LEVEL_INFO);
        println(" to ", min, "", LEVEL_INFO);
        lidarConfig.minDistance = min;
    }
    if (max != -1)
    {
        print("LidarLD06 Config 'Distance Max' from ", lidarConfig.maxDistance, "",
              LEVEL_INFO);
        println(" to ", max, "", LEVEL_INFO);
        lidarConfig.maxDistance = max;
    }
    if (quality != -1)
    {
        print("LidarLD06 Config 'Quality' from ", lidarConfig.minQuality, "", LEVEL_INFO);
        println(" to ", quality, "", LEVEL_INFO);
        lidarConfig.minQuality = quality;
    }
    if (distance != -1)
    {
        print("LidarLD06 Config 'Distance Threshold' from ",
              lidarConfig.distanceThreshold, "", LEVEL_INFO);
        println(" to ", distance, "", LEVEL_INFO);
        lidarConfig.distanceThreshold = distance;
    }
    if (angle != -1)
    {
        print("LidarLD06 Config 'Angle Threshold' from ", lidarConfig.angleThreshold, "", LEVEL_INFO);
        println(" to ", angle, "", LEVEL_INFO);
        lidarConfig.angleThreshold = angle;
    }
    if (count != -1)
    {
        print("LidarLD06 Config 'Count Threshold' from ", lidarConfig.countThreshold, "", LEVEL_INFO);
        println(" to ", count, "", LEVEL_INFO);
        lidarConfig.countThreshold = count;
    }
}

ConfigLidar LidarLD06::GetConfig() { return lidarConfig; }

void LidarLD06::ChangePWM(uint32_t duty_cycle)
{
    uint32_t duty = duty_cycle;
    // Limit the min and max
    if (duty < 20)
        duty = 20;
    if (duty > 50)
        duty = 50;
    // Créer le rapport cyclique en fonction de la résolution
    uint32_t max_duty = (1 << 8) - 1;
    duty = max_duty * duty / 100;

    ledcWrite(0, duty);
}

uint32_t LidarLD06::GetPWM() { return ledcRead(0) * 100 / ((1 << 8) - 1); }

boolean LidarLD06::ReadSerial()
{
    while (SERIAL_LIDAR.available() > 0)
    {
        // we are building a lidar packet
        // it always start with 0x54 0x2C and is 47 bytes long
        uint32_t tmpInt = SERIAL_LIDAR.read();
        // SERIAL_ROBOT.write(tmpInt);
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
                Analyze();
                return true;
            }
        }
    }
    return false;
}

void LidarLD06::Analyze()
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
        int rawDeg = lidarPacket.startAngle + i * angleStep + LIDAR_ROBOT_ANGLE_OFFSET;
        // Raw angle are inverted
        lidarPacket.dataPoint[i].angle = 360 * 100 - (rawDeg <= 360 * 100 ? rawDeg : rawDeg - 360 * 100);
        lidarPacket.dataPoint[i].confidence = (serialBuffer[8 + i * 3]);
        lidarPacket.dataPoint[i].distance = (int(serialBuffer[8 + i * 3 - 1] << 8 | serialBuffer[8 + i * 3 - 2]));
    }
}

boolean LidarLD06::CheckContinuity()
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
        println("Discontinuity : ", (float)delta / 100, " deg", LEVEL_WARN);
        return false;
    }
    else
    {
        return true;
    }
}

PacketLidar LidarLD06::GetData() { return lidarPacket; }

Point LidarLD06::PolarToCartesian(PolarPoint lidar_point, Robot robot)
{
    Point point;
    RobotPosition robotPosition = robot.GetPosition();
    float angle = lidar_point.angle + robotPosition.angle;
    angle /= 100;
    point.x = robotPosition.x + lidar_point.distance * cos(angle * M_PI / 180);
    point.y = robotPosition.y + lidar_point.distance * sin(angle * M_PI / 180);

    return point;
}

bool LidarLD06::IsOutsideTable(Point point)
{
    // the margin represents the distance between the center of the obstacle
    // and the edges of the table (which is 3000mm long and 2000mm large)
    const float table_margin = 100;
    //! TODO : Remove -5000 (it's for test) and put correct map boundaries
    return (point.x < -5000 + table_margin || point.x > 5000 - table_margin || point.y < -5000 + table_margin ||
            point.y > 5000 - table_margin);
}

bool LidarLD06::IsOutsideConfig(PolarPoint point)
{
    return (point.distance < lidarConfig.minDistance ||
            point.distance > lidarConfig.maxDistance ||
            point.confidence < lidarConfig.minQuality);
}

uint8_t thresholdCount = 0;
void LidarLD06::AggregatePoint(PolarPoint lidar_point, Tracker *tracker, Robot robot)
{
    boolean aggregate = true;

    Point point = PolarToCartesian(lidar_point, robot);
    lidar_point.x = point.x;
    lidar_point.y = point.y;
    scan.push_back(lidar_point);

    // Ignore points outside of the table and outside config
    if (IsOutsideTable(point))
    {
        print("Outside table : ", lidar_point);
        aggregate = false;
    }

    if (IsOutsideConfig(lidar_point))
    {
        print("Outside config : ", lidar_point);
        aggregate = false;
    }

    // TODO TEMP FIX TO DETECT ONLY ON FRONT OF THE ROBOT (90°)
    // if (lidar_point.angle > 4500 && lidar_point.angle < 31500)
    // {
    //     // println("Not in front of robot : ", point);
    //     aggregate = false;
    // }

    // if we have too much data for this obstacle
    if (pointsCounter >= kMaxPoints)
    {
        // TODO do not take this as obstacle because it's too large (wall, person, ...)
        // ObstacleDetected(tracker, kMaxPoints);

        // we don't need this point into the batch : overflow of point number
        aggregate = false;

        // we drop the current batch of point
        if (NewObstacleThreshold(lidar_point))
        {
            println("Drop batch, too much points : ", pointsCounter);
            pointsCounter = 0;
            thresholdCount = 0;
        }
    }
    else
    {
        // if we are still under the maximum size of the batch

        // Determine if it is a new obstacle
        if (pointsCounter > 0 && NewObstacleThreshold(lidar_point))
        {
            thresholdCount++;
            println("thresholdCount : ", thresholdCount);
            if (thresholdCount >= lidarConfig.countThreshold)
            {
                // TODO : To test countThreshold if accurate
                // TODO : We need to test if the batch of point is really a beacon
                thresholdCount = 0;

                //  if we have sufficient data for this obstacle
                //  we save current batch of points into an obstacle
                if (pointsCounter >= obstacleMinPoints)
                {
                    println("Obstacle Detected : ", pointsCounter);
                    ObstacleDetected(tracker, pointsCounter);

                    // we start another batch of points
                    pointsCounter = 0;
                }
                else
                {
                    // not enough point, we drop the current obstacle
                    println("Drop batch, not enough point : ", pointsCounter);
                    pointsCounter = 0;
                }
            }
        }
    }
    // println("PointsCounter : ", pointsCounter);
    if (aggregate)
    {
        // save the coord of current lidar point (a copy is made)
        // print("Aggregate : ", lidar_point);
        pointsTmp.data[pointsCounter++] =
            PolarPoint(lidar_point.angle, lidar_point.distance, lidar_point.confidence, point.x, point.y);
    }
    else
    {
        // println("Do not Aggregate");
    }
}

void LidarLD06::ObstacleDetected(Tracker *tracker, uint8_t size)
{
    // println("Size obs :", size);
    pointsTmp.size = size;
    Point mid = ComputeCenter(pointsTmp);
    plotPoint(mid, "Mid");
    plotPolarPoints(pointsTmp.data, size, "Aggregation");

    tracker->track(mid, pointsTmp.data, pointsTmp.size);
    // pointsCounter = 0;
}

bool LidarLD06::NewObstacleThreshold(PolarPoint currentPoint)
{
    // TODO recalculate minimum angle threshold, or convert it to distance to have a better threshold config
    return (abs(pointsTmp.data[pointsCounter - 1].distance - currentPoint.distance) > lidarConfig.distanceThreshold ||
            abs((int)(pointsTmp.data[pointsCounter - 1].angle)) - abs((int)(currentPoint.angle)) >
                lidarConfig.angleThreshold * 100);
}

Point LidarLD06::ComputeCenter(PointAggregation points)
{
    Point mid = {0, 0};
    for (int8_t d = 0; d < points.size; d++)
    {
        mid.x += points.data[d].x;
        mid.y += points.data[d].y;
    }
    mid.x = mid.x / points.size;
    mid.y = mid.y / points.size;

    return mid;
}

Point LidarLD06::FindCircle(Point p1, Point p2, Point p3)
{
    return FindCircle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
}

Point LidarLD06::FindCircle(float x1, float y1, float x2, float y2, float x3, float y3)
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
