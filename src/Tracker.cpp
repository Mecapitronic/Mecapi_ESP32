#include "Tracker.h"

Tracker::Tracker(float lpf_cutoff_distance, float hpf_cutoff_distance) : lpf_cutoff(lpf_cutoff_distance), hpf_cutoff(hpf_cutoff_distance)
{
    Debugger::println("Init Tracker");
    Debugger::log("Track new point if nothing close enough: ", lpf_cutoff, " mm", VERBOSE, true);
    Debugger::log("Ignore movements under ", hpf_cutoff, " mm", VERBOSE, true);
}
bool Tracker::PointIsEqual(Point a, Point b)
{
    return (a.x == b.x && a.y == b.y);
}

void Tracker::track(Point newPoint, PolarPoint data[], uint8_t size)
{
    // A point can not be further than 5000.0 as it is the field size
    float best_match = 5000.0; // hold the closest point to the new one
    int matching_point_index = -1;
    int first_available_slot = -1; // free slot to add new point

    Debugger::logPoint("Search point:", newPoint, "", VERBOSE, true);

    for (int i = 0; i < TRACKED_POINTS_SIZE; i++)
    {
        if (PointIsEqual(tracked_points[i].point, {0, 0}))
        {
            if (first_available_slot == -1)
            {
                first_available_slot = i;
            }
        }
        else
        {
            Debugger::logPoint("Compare to:", tracked_points[i].point, "", VERBOSE, true);

            // TODO remove sqrt and use x and y comparison
            float dist = sqrt(pow(newPoint.x - tracked_points[i].point.x, 2) + pow(newPoint.y - tracked_points[i].point.y, 2));

            // If distance is smaller than lpf_cutoff, assume it's the same point
            if ((dist < lpf_cutoff) && (dist < best_match))
            {
                // found matching point
                best_match = dist;
                matching_point_index = i;
            }
        }
    }

    if (best_match < hpf_cutoff)
    {
        Debugger::println("This is exactly the same point; do nothing");
        return;
    }

    // TODO what to do if found nothing and no slot???
    if (matching_point_index == -1 && first_available_slot == -1)
    {
        Debugger::println("No slot available for new point: drop it");
        return;
    }

    if (matching_point_index == -1 && first_available_slot != -1)
    {
        matching_point_index = first_available_slot;
        Debugger::println("New point detected, add to tracked");
    }

    PointTracker newPointTracker;
    newPointTracker.point = newPoint;
    newPointTracker.lastUpdateTime = getTimeNowMs();
    newPointTracker.hasBeenSent = false; // not yet sent to robot
    newPointTracker.size = size;         // Size of the polar points array
    for (size_t i = 0; i < size; i++)
    {
        newPointTracker.data[i] = data[i]; // Polar points associated
    }

    Debugger::log("Updating point ", matching_point_index, " ", INFO, false);
    Debugger::logPoint("from ", tracked_points[matching_point_index].point, "", VERBOSE, true);
    Debugger::logPoint("  to ", newPointTracker.point, "", VERBOSE, true);
    tracked_points[matching_point_index] = newPointTracker;
}

void Tracker::sendObstaclesToRobot(Robot robot)
{
    String varName = "obs";
    for (int i = 0; i < TRACKED_POINTS_SIZE; i++)
    {
        if (!tracked_points[i].hasBeenSent)
        {
            robot.WriteSerialdsPic(i, tracked_points[i].point);
            tracked_points[i].hasBeenSent = true;
            Debugger::plotPoint(tracked_points[i].point, varName + i);
            Debugger::plotTrackerPoints(tracked_points[i], tracked_points[i].size, "points");
        }
    }
}

void Tracker::untrackOldObstacles(Robot robot)
{
    String varName = "obs";
    // iterate to find matching point to the predicate
    for (int i = 0; i < TRACKED_POINTS_SIZE; i++)
    {
        if (!PointIsEqual(tracked_points[i].point, {0, 0}) && getTimeNowMs() - tracked_points[i].lastUpdateTime > IS_TOO_OLD)
        {
            tracked_points[i].point = {0, 0};
            robot.WriteSerialdsPic(i, {0, 0});

            Debugger::plotPoint({0, 0}, varName + i);
            Debugger::println("Untracking point: " + i);
        }
    }
}

int64_t Tracker::getTimeNowMs()
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);

    int64_t time_ms = (int64_t)tv_now.tv_sec * 1000 + ((int64_t)tv_now.tv_usec / 1000);

    // Debugger::print("time is: ");
    // Debugger::println(String(time_ms));

    return time_ms;
}

int64_t Tracker::getTimeNowUs()
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);

    int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    return time_us;
}