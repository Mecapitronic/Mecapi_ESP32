#include "Tracker.h"

void Tracker::Initialisation()
{
    println("Init Tracker", LEVEL_INFO);
    Config(DEFAULT_LPF_CUTOFF, DEFAULT_HPF_CUTOFF, CONFIDENCE_TRIGGER);
    trackedPoints.clear();
}

void Tracker::Config(float lpf_cutoff_distance, float hpf_cutoff_distance, int confidence)
{
    config.lpf_cutoff = lpf_cutoff_distance;
    config.hpf_cutoff = hpf_cutoff_distance;
    config.confidence = confidence;

    println("Track new point if nothing close enough: ", config.lpf_cutoff, " mm", LEVEL_INFO);
    println("Ignore movements under ", config.hpf_cutoff, " mm", LEVEL_INFO);
    println("Confidence trigger for sending tracking point to robot: ", config.confidence, " times", LEVEL_INFO);
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

        if (matching_point_index == -1)
        {
            PointTracker newPointTracker;
            newPointTracker.point = newPoint;
            newPointTracker.lastUpdateTime = millis();
            newPointTracker.hasBeenSent = false;  // not yet sent to robot
            newPointTracker.confidence = 1;
            println("New point detected, add to tracked");
            trackedPoints.push_back(newPointTracker);

            continue;
        }

        // TODO implémenter un poids par point en fonction du nombre de fois où on le voit
        // dans la limite entre 3 et 5 fois max par seconde
        if (best_match < config.hpf_cutoff)
        {
            println("This is exactly the same point, update only time");
            trackedPoints[matching_point_index].lastUpdateTime = millis();
            if (trackedPoints[matching_point_index].confidence < config.confidence)  // TODO param max confidence
            {
                trackedPoints[matching_point_index].confidence++;
            }
        }
        else
        {
            print("Updating point ", matching_point_index, " ");
            print("from ", trackedPoints[matching_point_index].point, "");
            print("  to ", newPoint, "");
            trackedPoints[matching_point_index].point = newPoint;
            trackedPoints[matching_point_index].hasBeenSent = false;
            trackedPoints[matching_point_index].lastUpdateTime = millis();
            if (trackedPoints[matching_point_index].confidence < config.confidence)
            {
                trackedPoints[matching_point_index].confidence++;
            }
        }
    }
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
        if (!trackPoint.hasBeenSent && trackPoint.confidence > config.confidence)
        {
            // robot.WriteSerial(trackPoint.index, trackPoint.point);
            trackPoint.hasBeenSent = true;
            // plotPolarPoint(trackPoint.point, varName + String(trackPoint.index));
            // print("TrackPoint: ", trackPoint);
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
            // TODO decrement confidence when the point has not been seen for a long time
            // decrement should be faster then increment
            // e.g -2 every 300ms
            // trackPoint.point = {0, 0};
            //  TODO Remove from vector if confidence is dropped too low
            //   robot.WriteSerial(trackPoint.index, {0, 0});

            // plotPoint({0, 0}, varName + trackPoint.index);
            print("Un-tracking point: ", trackPoint.point);
        }
    }
}
