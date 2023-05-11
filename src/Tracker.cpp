#include "Tracker.h"

Tracker::Tracker(float lpf_cutoff_distance, float hpf_cutoff_distance) : lpf_cutoff(lpf_cutoff_distance), hpf_cutoff(hpf_cutoff_distance)
{
    Debugger::println("Init Tracker");
    Debugger::log("Track new point if nothing close enough: ", lpf_cutoff, " mm", VERBOSE, true);
    Debugger::log("Ignore movements under ", hpf_cutoff, " mm", VERBOSE, true);
}

std::vector<PointTracker> Tracker::getPoints() { return tracked_points; }

int Tracker::findMatchingPoint(Point newPoint)
{
    // to hold the closest point to the new one
    // A point can not be further than 5000.0 as it is the field size
    float best_match = 5000.0;
    int matching_point_index = -1;

    Debugger::logPoint("Search point:", newPoint, "", VERBOSE, true);
    Debugger::log("tracked size: ", (int)tracked_points.size(), "", VERBOSE, true);

    for (int i = 0; i < tracked_points.size(); i++)
    {
        Debugger::logPoint("Compare to:", tracked_points.at(i).point, "", VERBOSE, true);

        float dist = sqrt(pow(newPoint.x - tracked_points.at(i).point.x, 2) + pow(newPoint.y - tracked_points.at(i).point.y, 2));

        // If distance is smaller than lpf_cutoff, assume it's the same point
        if ((dist < lpf_cutoff) && (dist < best_match))
        {
            // found matching point
            best_match = dist;
            matching_point_index = i;
        }
    }

    if (best_match < hpf_cutoff)
    {
        matching_point_index = -2;
    }

    return matching_point_index;
}

void Tracker::track(Point newPoint, PolarPoint data[], uint8_t size)
{
    int point_index = findMatchingPoint(newPoint);
    PointTracker newPointTracker;
    newPointTracker.point = newPoint;                 // point
    newPointTracker.lastUpdateTime = getTimeNowMs();  // lastUpdateTime
    newPointTracker.size = size;                      // Size of the polar points array
    for (size_t i = 0; i < size; i++)
    {
        newPointTracker.data[i] = data[i];  // Polar points associated
    }

    if (point_index == -1)
    {
        Debugger::println("New point detected, add to tracked");
        tracked_points.push_back(newPointTracker);
        return;
    }

    if (point_index == -2)
    {
        Debugger::println("This is exactly the same point, dropping");
        return;
    }

    Debugger::println("Updating point");
    Debugger::logPoint("from ", tracked_points[point_index].point, "", VERBOSE, true);
    Debugger::logPoint("  to ", newPointTracker.point, "", VERBOSE, true);
    tracked_points[point_index] = newPointTracker;
}

void Tracker::sendObstaclesToRobot(Robot robot)
{
    String varName = "obs";
    for (int i = 0; i < tracked_points.size(); i++)
    {
        int64_t delta = getTimeNowMs() - tracked_points[i].lastUpdateTime;
        if (delta < HAS_CHANGE_RECENTLY_MS)
        {
            robot.WriteSerial(i, tracked_points[i].point);
            Debugger::plotPoint(tracked_points[i].point, varName + i);
            Debugger::plotTrackerPoints(tracked_points[i], "points");
        }
    }
}

void Tracker::untrackOldObstacles(Robot robot)
{
    // iterate to find matching point to the predicate
    for (auto it = tracked_points.begin(); it < tracked_points.end(); it++)
    {
        if (getTimeNowMs() - it->lastUpdateTime > IS_TOO_OLD)
        {
            tracked_points.erase(it);
            robot.WriteSerial(it - tracked_points.begin(), {0, 0});
            Debugger::print("Untracking point: ");
            Debugger::println(it - tracked_points.begin());
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