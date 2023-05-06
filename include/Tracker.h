#ifndef TRACKER_H
#define TRACKER_H

#include <Arduino.h>
#include <vector>

#include "Structure.h"
#include "Debugger.h"
#include "Robot.h"

#define DEFAULT_LPF_CUTOFF 5000.0

#define HUNDRED_MILISECONDS 100000
#define SECOND 1000000

/**
 * lidar make 10 turns in 1 second, data are updated every 100 miliseconds
 * data are send every 25 miliseconds to robot
 * because robot updates its data every 50 miliseconds
 * a recently updated data is younger than at least 200 second
 */
#define HAS_CHANGE_RECENTLY 2 * HUNDRED_MILISECONDS

/**
 * amount of time needed to delete a point from tracker
 * if it is not detected in this term
 */
#define IS_TOO_OLD 3 * SECOND

/**
 * @brief In charge of tracking objects on the field based on Lidar detections and Kalman filter
 */
class Tracker
{

public:
    /**
     * @brief Construct a new Tracker object
     * if no cutoff distance is provided use DEFAULT_LPF_CUTOFF
     *
     * @param cutoff maxmimal distance between to points to match them as the same point
     */
    Tracker(float cutoff = DEFAULT_LPF_CUTOFF);

    /**
     * @brief Get the list of tracked points
     *
     * @return std::vector<PointTracker> list of tracked points
     */
    std::vector<PointTracker> getPoints();

    /**
     * @brief send new point to tracker
     * automatically detects if the point is new,
     * in that case add it to list of tracked points
     */
    void track(Point newPoint);

    /**
     * @brief filter (low pass) the given point to search if it is already tracked
     * search for the closest point: we assume it's the same point
     * if the distance between a tracked point and newPoint is smaller than lpf_cutoff,
     *
     * @param newPoint
     * @return int index of the matching point if the point is already tracked else return -1
     */
    int findMatchingPoint(Point newPoint);

    /**
     * @brief send all tracked obstacles to robot
     * TODO reduce amount of data sent by filtering not big enough changes
     * TODO detect big changes not to send too much data
     *
     * @param robot the robot object to send data to
     */
    void sendObstaclesToRobot(Robot robot);

    /**
     * @brief Get the current time with milisecond precision
     *
     * @return int64_t current time in milisecond
     */
    int64_t getTimeNowMs();

    /**
     * @brief Get the current time with microsecond precision
     *
     * @return int64_t current time in microseconds
     */
    int64_t getTimeNowUs();

private:
    /**
     * @brief list of obstacles/points being tracked
     * the list is updarted with new data
     * and cleaned up if some points are not updated for a long time
     */
    std::vector<PointTracker> tracked_points;

    /**
     * @brief cut off of the low pass filter
     * limits to define the closest robot to track matching points
     */
    float lpf_cutoff;
};
#endif /* TRACKER_H */
