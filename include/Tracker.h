#ifndef TRACKER_H
#define TRACKER_H

#include <Arduino.h>
#include <vector>

#include "Structure.h"
#include "Debugger.h"
#include "Robot.h"

// maximal distance between to points to match them as the same point
#define DEFAULT_LPF_CUTOFF 300.0

// minimum movement needed to update position of tracked point
#define DEFAULT_HPF_CUTOFF 100.0

/**
 * amount of time needed to delete a point from tracker
 * if it is not detected in this term
 * we choose 3 seconds, therefore 3000 miliseconds
 */
#define IS_TOO_OLD 3000

#define TRACKED_POINTS_SIZE 5

/**
 * @brief In charge of tracking objects on the field based on Lidar detections and Kalman filter
 */
class Tracker
{
public:
    /**
     * @brief Construct a new Tracker object with settings for filters
     *
     * @param lpf_cutoff_distance maximal distance between to points to match them as the same point; default DEFAULT_LPF_CUTOFF
     * @param hpf_cutoff_distance minimum movement needed to update position of tracked point; default DEFAULT_HPF_CUTOFF
     */
    Tracker(float lpf_cutoff_distance = DEFAULT_LPF_CUTOFF, float hpf_cutoff_distance = DEFAULT_HPF_CUTOFF);

    /**
     * @brief send new point to tracker
     * automatically detects if the point is new,
     * in that case add it to list of tracked points
     */
    void track(Point newPoint, PolarPoint data[], uint8_t size);

    /**
     * @brief filter (low pass) the given point to search if it is already tracked
     * search for the closest point: we assume it's the same point
     * if the distance between a tracked point and newPoint is smaller than lpf_cutoff,
     *
     * @param newPoint
     * @return int index of the matching point if the point is already tracked;
     *  -1 if the point is not already tracked;
     *  -2 if the point is tracked but shouldn't be updated
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
     * @brief Delete old obstacles from tracked list
     * if the object has not been updated for IS_TOO_OLD time we should stop tracking it
     *
     * @param robot the robot object to send untracked points
     */
    void untrackOldObstacles(Robot robot);

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

    bool PointIsEqual(Point a, Point b);

private:
    /**
     * @brief list of obstacles/points being tracked
     * the list is updarted with new data
     * and cleaned up if some points are not updated for a long time
     */
    PointTracker tracked_points[TRACKED_POINTS_SIZE];

    /**
     * @brief cut off of the low pass filter
     * limits to define the closest robot to track matching points
     */
    float lpf_cutoff = DEFAULT_LPF_CUTOFF;

    /**
     * @brief cut off of the high pass filter
     * limits to define the minimum movement a robot should do to be updated
     * this covers the false positives due to lidar lack of precision
     */
    float hpf_cutoff = DEFAULT_HPF_CUTOFF;
};
#endif /* TRACKER_H */
