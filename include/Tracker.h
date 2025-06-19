#ifndef TRACKER_H
#define TRACKER_H

#include "ESP32_Helper.h"
#include "Robot.h"
extern Robot robot;

using namespace Printer;

// maximal distance between to points to match them as the same cluster : a robot diameter ish~
#define DEFAULT_LPF_CUTOFF 300.0

// minimum movement needed to update position of tracked point : 5 cm
#define DEFAULT_HPF_CUTOFF 50.0

// number of time a point has been seen before trigger sending point to robot
#define CONFIDENCE_TRIGGER 3

// maximum number of time a point has been seen
#define CONFIDENCE_MAXIMUM 10

// maximum number of time a point has been seen
#define POINT_SEND_TO_ROBOT_MAXIMUM 5

/**
 * amount of time needed to decrement the confidence of a point from tracker
 * if it is not detected in this time
 * we make 10 turns each second, 1 turn every 100ms
 */
#define IS_TOO_OLD 200

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
    int8_t confidenceTrigger;

    /**
     * @brief Confidence Maximum : Maximum number of time a point has been seen
     */
    int8_t confidenceMax;

    /**
     * @brief Number of point to send to robot
     *
     */
    int8_t maxPointToSendToRobot;
};

/**
 * @brief In charge of tracking objects on the field based on LidarLD06 detections and Kalman filter
 */
class Tracker
{
   public:
    void Initialisation();
    void Update();
    void HandleCommand(Command cmd);
    void SendToRobot();
    void Teleplot(bool all);

    /**
     * @brief Configure a new Tracker object with settings for filters
     *
     * @param lpf_cutoff_distance maximal distance between to points to match them as the same point
     * @param hpf_cutoff_distance minimum movement needed to update position of tracked point
     * @param confidenceTrigger number of time a point has been seen before trigger sending point to robot
     * @param confidenceMax maximum number of time a point has been seen
     * @param maxPointToSendToRobot number of point to send to robot
     */
    void Config(float lpf_cutoff_distance, float hpf_cutoff_distance, int8_t confidenceTrigger, int8_t confidenceMax,
                int8_t maxPointToSendToRobot);

    /**
     * @brief send new point to tracker
     * automatically detects if the point is new,
     * in that case add it to list of tracked points
     */
    void Track(std::vector<PolarPoint>& newPoints);

    bool PointIsEqual(PolarPoint a, PolarPoint b);

   private:
    /**
     * @brief list of obstacles/points being tracked
     * the list is updated with new data
     * and cleaned up if some points are not updated for a long time
     */
    std::vector<PointTracker> trackedPoints;

    ConfigTracker config = {0, 0, 0, 0, 0};
};
#endif
