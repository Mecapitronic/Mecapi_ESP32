#ifndef TRACKER_H
#define TRACKER_H

#include "ESP32_Helper.h"
#include "Robot.h"

using namespace Printer;

// maximal distance between to points to match them as the same cluster : a robot diameter ish~
#define DEFAULT_LPF_CUTOFF 300.0

// minimum movement needed to update position of tracked point : 5 cm
#define DEFAULT_HPF_CUTOFF 50.0

// number of time a point has been seen before trigger sending point to robot
#define CONFIDENCE_TRIGGER 5

// maximum number of time a point has been seen
#define CONFIDENCE_MAXIMUM 50

/**
 * amount of time needed to decrement the confidence of a point from tracker
 * if it is not detected in this time
 * we make 10 turns each second, 1 turn every 100ms
 */
#define IS_TOO_OLD 3000  // TODO : adjust this to 200 ?

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

    /**
     * @brief ConfidenceTrigger : number of time a point has been seen
     * before trigger sending point to robot
     */
    int confidenceTrigger;

    /**
     * @brief Confidence Maximum : Maximum number of time a point has been seen
     */
    int confidenceMax;
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
     * @param lpf_cutoff_distance maximal distance between to points to match them as the same point
     * @param hpf_cutoff_distance minimum movement needed to update position of tracked point
     * @param confidenceTrigger number of time a point has been seen before trigger sending point to robot
     * @param confidenceMax maximum number of time a point has been seen
     */
    void Config(float lpf_cutoff_distance, float hpf_cutoff_distance, int confidenceTrigger, int confidenceMax);

    /**
     * @brief send new point to tracker
     * automatically detects if the point is new,
     * in that case add it to list of tracked points
     */
    void Track(vector<PolarPoint>& newPoints);

    bool PointIsEqual(PolarPoint a, PolarPoint b);

   private:
    /**
     * @brief list of obstacles/points being tracked
     * the list is updated with new data
     * and cleaned up if some points are not updated for a long time
     */
    vector<PointTracker> trackedPoints;

    ConfigTracker config = {0, 0, 0, 0};
};
#endif  // TRACKER_H
