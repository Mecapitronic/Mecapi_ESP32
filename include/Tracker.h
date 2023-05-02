#ifndef TRACKER_H
#define TRACKER_H

#include <Arduino.h>

#include "Structure.h"
#include "Debugger.h"
#include "Robot.h"

/**
 * In charge of tracking objects on the field based on Lidar detections and Kalman filter
 */
class Tracker
{

public:
    Tracker();
    void trackNewObstacle(Point obstacle);
    void sendObstacleToRobot(Robot robot);

private:
    // all obstacles tracked right now
    // std::vector<Point> obstacleTracked;
    // counter of obstacles tracked right now: tracked_object_list.size

    bool newObstacle = false;
    Point obstacleTracked;
};
#endif /* TRACKER_H */
