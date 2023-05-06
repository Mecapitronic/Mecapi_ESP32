#ifndef TRACKER_H
#define TRACKER_H

#include <Arduino.h>
#include <vector>

#include "Structure.h"
#include "Debugger.h"
#include "Robot.h"

#define DEFAULT_LPF_CUTOFF 5000.0
/**
 * In charge of tracking objects on the field based on Lidar detections and Kalman filter
 */
class Tracker
{

public:
    /**
     * Construct the tracker, if no cutoff distance is provided use DEFAULT_LPF_CUTOFF
     * as maxmimal distance between to points to match them as the same point
     */
    Tracker(float cutoff = DEFAULT_LPF_CUTOFF);

    /**
     * returns the list of tracked points
     */
    std::vector<TrackPoint> getPoints();

    /**
     * send new point to tracker, it will automatically detects if it is new one or not
     */
    void track(Point newPoint);

    /**
     * filter the given point to search if it is already tracked with a low pass filter
     * search for the closest point
     * if the point is already tracked returns the index of the matching point
     * else return -1
     * TODO add cleanup to delete the point if no more tracked/existing
     */
    int findMatchingPoint(Point newPoint);

    /**
     * send all tracked obstacles to robot
     * TODO reduce amount of data sent by filtering not big enough changes
     */
    void sendObstaclesToRobot(Robot robot);

private:
    // filtered status of obstacles/points
    std::vector<TrackPoint> tracked_points;

    /**
     * cut off of the low pass filter
     * limits to define the closest robot to track matching points
     */
    float lpf_cutoff;
};
#endif /* TRACKER_H */
