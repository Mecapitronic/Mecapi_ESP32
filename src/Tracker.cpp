#include "Tracker.h"

Tracker::Tracker(float cutoff) : lpf_cutoff(cutoff)
{
    Debugger::println("Init Tracker");
}

std::vector<TrackPoint> Tracker::getPoints()
{
    return tracked_points;
}

int Tracker::findMatchingPoint(Point newPoint)
{
    // to hold the closest point to the new one
    // A point can not be further than 5000.0 as it is the field size
    float best_match = 5000.0;
    int matching_point_index = -1;

    Debugger::log("search point: x: ", newPoint.x, "  ", VERBOSE, false);
    Debugger::log("y: ", newPoint.y, "", VERBOSE);
    Debugger::log("tracked size: ", (int)tracked_points.size(), "", VERBOSE);

    for (int i = 0; i < tracked_points.size(); i++)
    {
        Debugger::log("Compare to: x: ", tracked_points.at(i).point.x, "  ", VERBOSE, false);
        Debugger::log("y: ", tracked_points.at(i).point.y, "  ", VERBOSE);

        float dist = sqrt(pow(newPoint.x - tracked_points.at(i).point.x, 2) + pow(newPoint.y - tracked_points.at(i).point.y, 2));

        // If distance is smaller than lpf_cutoff, assume it's the same point
        if ((dist < lpf_cutoff) && (dist < best_match))
        {
            Debugger::println("it's the same point");
            best_match = dist;
            matching_point_index = i;
        }
    }

    return matching_point_index;
}

void Tracker::track(Point newPoint, PolarPoint data[], uint8_t size)
{
    int point_index = findMatchingPoint(newPoint);
    TrackPoint newTrackPoint;
    newTrackPoint.point = newPoint;
    newTrackPoint.isNew = true;
    newTrackPoint.size = size;
    for (size_t i = 0; i < size; i++)
    {
        newTrackPoint.data[i] = data[i];
    }

    if (point_index == -1)
    {
        Debugger::println("New point detected, add to tracked");
        // Add point to list of tracked points

        tracked_points.push_back(newTrackPoint);
    }
    else
    {
        // update with new value
        Debugger::println("Point already tracked, updating");
        tracked_points[point_index] = newTrackPoint;
    }
}

void Tracker::sendObstaclesToRobot(Robot robot)
{
    // TODO detect big changes not to send too much data
    for (int i = 0; i < tracked_points.size(); i++)
    {
        if (tracked_points[i].isNew)
        {
            tracked_points[i].isNew = false;
            robot.WriteSerial(i, tracked_points[i].point);
            // Debugger::log("Obstacle : ", i, ") ", VERBOSE, false);
            // Debugger::log("x= ", (int)tracked_points[i].point.x, " ", VERBOSE, false);
            // Debugger::log("y= ", (int)tracked_points[i].point.y, "", VERBOSE);
            Debugger::plotPoint(tracked_points[i].point, "obs");
            Debugger::plotTrackerPoints(tracked_points[i], "points");
        }
    }
}