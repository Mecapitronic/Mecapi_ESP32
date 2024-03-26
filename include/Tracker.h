#ifndef TRACKER_H
#define TRACKER_H

#include "ESP32_Helper.h"
#include "Robot.h"

using namespace Printer;

// maximal distance between to points to match them as the same point
#define DEFAULT_LPF_CUTOFF 300.0  // a robot diameter ish~

// minimum movement needed to update position of tracked point
#define DEFAULT_HPF_CUTOFF 50.0  // 5 cm

/**
 * amount of time needed to delete a point from tracker
 * if it is not detected in this term
 * we choose 3 seconds, therefore 3000 miliseconds
 */
#define IS_TOO_OLD 3000

struct ConfigTracker
{
    /**
     * @brief cut off of the low pass filter
     * limits to define the closest robot to track matching points
     */
    float lpf_cutoff;

    /**
     * @brief cut off of the high pass filter
     * limits to define the minimum movement a robot should do to be updated
     * this covers the false positives due to lidar lack of precision
     */
    float hpf_cutoff;
};

/**
 * @brief In charge of tracking objects on the field based on LidarLD06 detections and Kalman filter
 */
class Tracker
{
   public:
    void Initialisation();
    void Update();

    /**
     * @brief Configure a new Tracker object with settings for filters
     *
     * @param lpf_cutoff_distance maximal distance between to points to match them as the same point; default
     * DEFAULT_LPF_CUTOFF
     * @param hpf_cutoff_distance minimum movement needed to update position of tracked point; default
     * DEFAULT_HPF_CUTOFF
     */
    void Config(float lpf_cutoff_distance, float hpf_cutoff_distance);

    /**
     * @brief send new point to tracker
     * automatically detects if the point is new,
     * in that case add it to list of tracked points
     */
    void Track(vector<PolarPoint>& newPoints);

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
    int findMatchingPoint(PolarPoint newPoint);

    /**
     * @brief send all tracked obstacles to robot
     * TODO reduce amount of data sent by filtering not big enough changes
     * TODO detect big changes not to send too much data
     */
    void sendObstaclesToRobot();

    /**
     * @brief Delete old obstacles from tracked list
     * if the object has not been updated for IS_TOO_OLD time we should stop tracking it
     */
    void untrackOldObstacles();

    /**
     * @brief Get the current time with miliseconds precision
     *
     * @return int64_t current time in miliseconds
     */
    int64_t getTimeNowMs();

    /**
     * @brief Get the current time with microsecond precision
     *
     * @return int64_t current time in microseconds
     */
    int64_t getTimeNowUs();

    bool PointIsEqual(PolarPoint a, PolarPoint b);

   private:
    /**
     * @brief list of obstacles/points being tracked
     * the list is updated with new data
     * and cleaned up if some points are not updated for a long time
     */
    vector<PointTracker> trackedPoints;
    ConfigTracker config = {0, 0};
    int indexGlobal = 0;
};
#endif
