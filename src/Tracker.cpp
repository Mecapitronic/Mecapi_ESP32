#include "Tracker.h"

void Tracker::Initialisation()
{
    println("Init Tracker", LEVEL_INFO);
    Config(DEFAULT_LPF_CUTOFF, DEFAULT_HPF_CUTOFF);
    trackedPoints.clear();
}

void Tracker::Config(float lpf_cutoff_distance, float hpf_cutoff_distance)
{
    config.lpf_cutoff = lpf_cutoff_distance;
    config.hpf_cutoff = hpf_cutoff_distance;

    println("Track new point if nothing close enough: ", config.lpf_cutoff, " mm", LEVEL_INFO);
    println("Ignore movements under ", config.hpf_cutoff, " mm", LEVEL_INFO);
}

bool Tracker::PointIsEqual(PolarPoint a, PolarPoint b) { return (a.x == b.x && a.y == b.y); }

void Tracker::Track(vector<PolarPoint>& newPoints)
{
    for (auto& newPoint : newPoints)
    {
        // A point can not be further than 5000.0 as it is the field size
        float best_match = 5000.0;  // hold the closest point to the new one
        int matching_point_index = -1;
        // int first_available_slot = -1;  // free slot to add new point

        print("Search point:", newPoint);

        int i = 0;
        for (auto& trackPoint : trackedPoints)
        {
            print("Compare to:", trackPoint.point);

            // TODO remove sqrt and use x and y comparison
            float dist = sqrt(pow(newPoint.x - trackPoint.point.x, 2) + pow(newPoint.y - trackPoint.point.y, 2));
            println("Distance:", dist);

            // If distance is smaller than lpf_cutoff, assume it's the same point
            if ((dist < config.lpf_cutoff) && (dist < best_match))
            {
                // found matching point
                best_match = dist;
                matching_point_index = i;
            }
            i++;
        }

        if (matching_point_index != -1 && best_match < config.hpf_cutoff)
        {
            println("This is exactly the same point, update time");
            trackedPoints[matching_point_index].lastUpdateTime = getTimeNowMs();
        }
        else
        {
            PointTracker newPointTracker;
            newPointTracker.point = newPoint;
            newPointTracker.lastUpdateTime = getTimeNowMs();
            newPointTracker.hasBeenSent = false;  // not yet sent to robot
            newPointTracker.index = indexGlobal++;

            if (matching_point_index == -1)
            {
                println("New point detected, add to tracked");
                trackedPoints.push_back(newPointTracker);
            }
            else
            {
                print("Updating point ", matching_point_index, " ");
                print("from ", trackedPoints[matching_point_index].point, "");
                print("  to ", newPointTracker.point, "");
                trackedPoints[matching_point_index] = newPointTracker;
            }
        }
    }
    newPoints.clear();
}

void Tracker::Update()
{
    sendObstaclesToRobot();
    untrackOldObstacles();
}

void Tracker::sendObstaclesToRobot()
{
    String varName = "obs";
    for (auto& trackPoint : trackedPoints)
    {
        if (!trackPoint.hasBeenSent)
        {
            // robot.WriteSerial(trackPoint.index, trackPoint.point);
            trackPoint.hasBeenSent = true;
            plotPolarPoint(trackPoint.point, varName + String(trackPoint.index));
            plotTrackerPoint(trackPoint, "TrackPoint" + String(trackPoint.index));
        }
    }
}

void Tracker::untrackOldObstacles()
{
    String varName = "obs";
    // iterate to find matching point to the predicate
    for (auto& trackPoint : trackedPoints)
    {
        if (!PointIsEqual(trackPoint.point, {0, 0}) && getTimeNowMs() - trackPoint.lastUpdateTime > IS_TOO_OLD)
        {
            trackPoint.point = {0, 0};
            // robot.WriteSerial(trackPoint.index, {0, 0});

            plotPoint({0, 0}, varName + trackPoint.index);
            println("Un-tracking point: ", trackPoint.index, "");
        }
    }
}

int64_t Tracker::getTimeNowMs()
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);

    int64_t time_ms = (int64_t)tv_now.tv_sec * 1000 + ((int64_t)tv_now.tv_usec / 1000);
    return time_ms;
}

int64_t Tracker::getTimeNowUs()
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);

    int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    return time_us;
}
