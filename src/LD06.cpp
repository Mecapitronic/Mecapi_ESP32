#include "LD06.h"

void LidarLD06::Initialisation()
{
    println("Init LidarLD06", LEVEL_INFO);
    scan.clear();
    clusterCenterPoints.clear();

    // minDistance, maxDistance, minQuality, distanceThreshold, angleThreshold, countThreshold;
    Config(100, 1500, 200, 200, 0.8 * 5, 2);
    SERIAL_LIDAR.begin(230400);

    int pwmChannel = 0;     // Choisit le canal 0
    int frequence = 30000;  // Fréquence PWM de 30 KHz
    int resolution = 8;     // Résolution de 8 bits, 256 valeurs possibles
    int pwmPin = PWM_PIN;

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
        print("LidarLD06 Config 'Distance Min' from ", lidarConfig.minDistance, "", LEVEL_INFO);
        println(" to ", min, "", LEVEL_INFO);
        lidarConfig.minDistance = min;
    }
    if (max != -1)
    {
        print("LidarLD06 Config 'Distance Max' from ", lidarConfig.maxDistance, "", LEVEL_INFO);
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
        print("LidarLD06 Config 'Distance Threshold' from ", lidarConfig.distanceThreshold, "", LEVEL_INFO);
        println(" to ", distance, "", LEVEL_INFO);
        lidarConfig.distanceThreshold = distance;
    }
    if (angle != -1)
    {
        print("LidarLD06 Config 'Angle Threshold' from ", lidarConfig.angleThreshold, "", LEVEL_INFO);
        println(" to ", angle, "", LEVEL_INFO);
        lidarConfig.angleThreshold = angle;
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

void LidarLD06::SetRobotPosition(PolarPoint robot) { robotPosition = robot; }

void LidarLD06::Update()
{
    clusterCenterPoints.clear();
    if (ReadSerial())
    {
        Analyze();
        CheckContinuity();

        for (int i = 0; i < LIDAR_DATA_PACKET_SIZE; i++)
        {
            // Ignore points outside of the table and outside config
            if (IsOutsideTable(lidarPacket.dataPoint[i]))
            {
                // print("Outside table : ", lidarPacket.dataPoint[i]);
            }
            else if (IsOutsideConfig(lidarPacket.dataPoint[i]))
            {
                // print("Outside config : ", lidarPacket.dataPoint[i]);
            }
            else
            {
                AggregatePoint(lidarPacket.dataPoint[i]);
                scan.push_back(lidarPacket.dataPoint[i]);
            }
        }

        CheckCluster(lidarPacket.dataPoint[LIDAR_DATA_PACKET_SIZE - 1]);
    }
}

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
        PolarToCartesian(lidarPacket.dataPoint[i]);
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

    if (delta > ANGLE_MAX_DISCONTINUITY)
    {
        println("Discontinuity : ", (float)delta / 100, " deg", LEVEL_WARN);
        return false;
    }
    else
    {
        return true;
    }
}

void LidarLD06::PolarToCartesian(PolarPoint& polarPoint)
{
    float angle = polarPoint.angle + robotPosition.angle;
    angle /= 100;
    polarPoint.x = robotPosition.x + polarPoint.distance * cos(angle * M_PI / 180);
    polarPoint.y = robotPosition.y + polarPoint.distance * sin(angle * M_PI / 180);
}

bool LidarLD06::IsOutsideTable(PolarPoint polarPoint)
{
    // the margin represents the distance between the center of the obstacle
    // and the edges of the table (which is 3000mm long and 2000mm large)
    const float table_margin = 100;
    //! TODO : Remove -5000 (it's for test) and put correct map boundaries
    return (polarPoint.x < -5000 + table_margin || polarPoint.x > 5000 - table_margin ||
            polarPoint.y < -5000 + table_margin || polarPoint.y > 5000 - table_margin);
}

bool LidarLD06::IsOutsideConfig(PolarPoint polarPoint)
{
    return (polarPoint.distance < lidarConfig.minDistance || polarPoint.distance > lidarConfig.maxDistance ||
            polarPoint.confidence < lidarConfig.minQuality);
}

int clusterNum = 0;
void LidarLD06::AggregatePoint(PolarPoint polarPoint)
{
    bool clusterFounded = false;
    for (auto& c : cluster)
    {
        //  cluster empty or polarPoint inside cluster
        if (c.data.size() == 0 ||
            (abs(c.data.back().distance - polarPoint.distance) < lidarConfig.distanceThreshold &&
             abs((int)(c.data.back().angle)) - abs((int)(polarPoint.angle)) < lidarConfig.angleThreshold * 100))
        {
            c.data.push_back(polarPoint);
            // recalculate the average point of the cluster
            c.mid.angle = (c.mid.angle * (c.data.size() - 1) + polarPoint.angle) / c.data.size();
            c.mid.distance = (c.mid.distance * (c.data.size() - 1) + polarPoint.distance) / c.data.size();
            // exist for as we fond which cluster the point is inside
            clusterFounded = true;
            break;
        }
    }

    if (!clusterFounded)
    {
        println("Creating new cluster N°", clusterNum);
        Cluster aggPoint = {{polarPoint}, polarPoint, clusterNum};
        cluster.push_back(aggPoint);
        clusterNum++;
    }
}

void LidarLD06::CheckCluster(PolarPoint polarPoint)
{
    println("Last Angle : ", polarPoint.angle);
    vector<vector<Cluster>::iterator> iterators;

    // TODO to be replaced by : for (auto& c : cluster)
    for (int i = cluster.size() - 1; i >= 0; i--)
    {
        println("Cluster Angle : ", cluster[i].data[cluster[i].data.size() - 1].angle);
        // the angle are descending
        float angle = cluster[i].data[cluster[i].data.size() - 1].angle - 10 * 100;
        if (angle < 0)
            angle += 360 * 100;

        PolarToCartesian(cluster[i].mid);

        if (polarPoint.angle < angle)
        {
            print("Cluster N° ", cluster[i].index);
            println(" with Size ", cluster[i].data.size(), " will be checked");

            // Minimum amount of points needed for a 60 mm balise's diameter //TODO var 60
            float minPoint = ((60 * 180) / (PI * cluster[i].mid.distance)) / 0.8;
            println("Min point : ", minPoint);
            // Maximum amount of points needed for a 120 mm balise's diameter //TODO var 120
            float maxPoint = ((120 * 180) / (PI * cluster[i].mid.distance)) / 0.8;
            println("Max point : ", maxPoint);

            // Arc Length s = 2 π r(θ / 360°)
            // TODO check for > 360°
            float angle = fabsf(cluster[i].data.front().angle - cluster[i].data.back().angle) / 100;
            float arc = (angle * cluster[i].mid.distance) * PI / 180;
            println("Arc length : ", arc);

            // At 300mm we need 20 points, at 1000mm we need 10 points, at 1500mm 5 points

            if (cluster[i].data.size() < 3)
            {
                println("Absolutely not enough points !");
            }
            else if (cluster[i].data.size() > ceil(maxPoint))
            {
                println("Too many points");
            }
            else if (cluster[i].data.size() < floor(minPoint))
            {
                println("Not enough points");
            }
            else
            {
                /*
                float s = (angle * cluster[i].mid.distance) * PI / 180;
                float theta1 = (60 * 180) / (PI * cluster[i].mid.distance);
                float theta2 = (80 * 180) / (PI * cluster[i].mid.distance);
                float theta3 = (100 * 180) / (PI * cluster[i].mid.distance);
                float theta4 = (110 * 180) / (PI * cluster[i].mid.distance);
                float theta5 = (120 * 180) / (PI * cluster[i].mid.distance);
                print("θ1 : ", theta1);
                println(" points : ", (float)(theta1 / 0.8));
                print("θ2 : ", theta2);
                println(" points : ", (float)(theta2 / 0.8));
                print("θ3 : ", theta3);
                println(" points : ", (float)(theta3 / 0.8));
                print("θ4 : ", theta4);
                println(" points : ", (float)(theta4 / 0.8));
                print("θ5 : ", theta5);
                println(" points : ", (float)(theta5 / 0.8));
                */

                print("Obstacle Detected mid Polar: ", cluster[i].mid);
                plotPolarPoint(cluster[i].mid, "MidPolar" + String(cluster[i].index));
                ObstacleDetected(cluster[i]);
                plotScanXY(cluster[i].data, "cluster", LEVEL_WARN);
            }
            iterators.push_back(cluster.begin() + i);
        }
    }

    for (auto& it : iterators)
    {
        cluster.erase(it);
    }
}

void LidarLD06::ObstacleDetected(Cluster& c) { clusterCenterPoints.push_back(c.mid); }

void LidarLD06::ComputeCenter(Cluster& c)
{
    for (int8_t d = 0; d < c.data.size(); d++)
    {
        // c.mid.angle += c.data[d].angle;
        // c.mid.distance += c.data[d].distance;

        // c.mid.x += c.data[d].x;
        // c.mid.y += c.data[d].y;
    }

    //c.mid.angle = c.mid.angle / c.data.size();
    //c.mid.distance = c.mid.distance / c.data.size();
    //PolarToCartesian(c.mid);
    
    // c.mid.x = mid.x / c.data.size();
    // c.mid.y = mid.y / c.data.size();
}
