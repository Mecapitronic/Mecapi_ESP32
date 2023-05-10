#include "Tracker.h"

Tracker::Tracker(float cutoff) : lpf_cutoff(cutoff) { Debugger::println("Init Tracker"); }

std::vector<PointTracker> Tracker::getPoints() { return tracked_points; }

int Tracker::findMatchingPoint(Point newPoint)
{
    // to hold the closest point to the new one
    // A point can not be further than 5000.0 as it is the field size
    float best_match = 5000.0;
    int matching_point_index = -1;

    // Debugger::logPoint("Search point:", newPoint, "", VERBOSE, false);
    // Debugger::log("tracked size: ", (int)tracked_points.size(), "", VERBOSE, true);

    for (int i = 0; i < tracked_points.size(); i++)
    {
        // Debugger::logPoint("Compare to:", tracked_points.at(i).point, "", VERBOSE, true);

        float dist = sqrt(pow(newPoint.x - tracked_points.at(i).point.x, 2) + pow(newPoint.y - tracked_points.at(i).point.y, 2));

        // If distance is smaller than lpf_cutoff, assume it's the same point
        if ((dist < lpf_cutoff) && (dist < best_match))
        {
            // found matching point
            best_match = dist;
            matching_point_index = i;
        }
    }

    return matching_point_index;
}

void Tracker::track(Point newPoint, PolarPoint data[], uint8_t size)
{
    int point_index = findMatchingPoint(newPoint);
    PointTracker newPointTracker = {
        newPoint,        // point
        getTimeNowMs(),  // lastUpdateTime
    };

    if (point_index == -1)
    {
        Debugger::println("New point detected, add to tracked");
        tracked_points.push_back(newPointTracker);
    }
    else
    {
        Debugger::println("Point already tracked");
        // TODO change to as high pass filter
        if (tracked_points[point_index].point.x == newPointTracker.point.x && tracked_points[point_index].point.y == newPointTracker.point.y)
        {
            Debugger::println("This is exactly the same point, dropping");
        }
        else
        {
            Debugger::print("Updating point from ");
            Debugger::print((int)tracked_points[point_index].point.x);
            Debugger::print(",");
            Debugger::print((int)tracked_points[point_index].point.y);
            Debugger::print(" to ");
            Debugger::print((int)newPointTracker.point.x);
            Debugger::print(",");
            Debugger::println((int)newPointTracker.point.y);

            tracked_points[point_index] = newPointTracker;
        }
    }
}

void Tracker::sendObstaclesToRobot(Robot robot)
{
    String varName = "obs";
    for (int i = 0; i < tracked_points.size(); i++)
    {
        int64_t delta = getTimeNowMs() - tracked_points[i].lastUpdateTime;
        if (delta < HAS_CHANGE_RECENTLY_HMS)
        {
            robot.WriteSerial(i, tracked_points[i].point);
            Debugger::plotPoint(tracked_points[i].point, varName + i);
            Debugger::plotTrackerPoints(tracked_points[i], "points");
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