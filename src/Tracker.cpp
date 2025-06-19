#include "Tracker.h"

void Tracker::Initialisation()
{
    println("Init Tracker", Level::LEVEL_INFO);
    Config(DEFAULT_LPF_CUTOFF, DEFAULT_HPF_CUTOFF, CONFIDENCE_TRIGGER, CONFIDENCE_MAXIMUM, POINT_SEND_TO_ROBOT_MAXIMUM);
    trackedPoints.clear();
}

void Tracker::Config(float lpf_cutoff_distance = -1, float hpf_cutoff_distance = -1, int8_t confidenceTrigger = -1,
                     int8_t confidenceMax = -1, int8_t maxPointToSendToRobot = -1)
{
    if (lpf_cutoff_distance != -1)
    {
        config.lpf_cutoff = lpf_cutoff_distance;
        println("Track new point if nothing close enough: ", config.lpf_cutoff, " mm", Level::LEVEL_INFO);
    }
    if (hpf_cutoff_distance != -1)
    {
        config.hpf_cutoff = hpf_cutoff_distance;
        println("Update point only if moved more than: ", config.hpf_cutoff, " mm", Level::LEVEL_INFO);
    }
    if (confidenceTrigger != -1)
    {
        config.confidenceTrigger = confidenceTrigger;
        println("Confidence trigger for sending tracking point to robot: ", config.confidenceTrigger, " times",
                Level::LEVEL_INFO);
    }
    if (confidenceMax != -1)
    {
        config.confidenceMax = confidenceMax;
        println("Confidence Maximum: ", config.confidenceMax, "", Level::LEVEL_INFO);
    }
    if (maxPointToSendToRobot != -1)
    {
        config.maxPointToSendToRobot = maxPointToSendToRobot;
        println("Max points to send to robot: ", config.maxPointToSendToRobot, "", Level::LEVEL_INFO);
    }
}

void Tracker::HandleCommand(Command cmd)
{
    if (cmd.cmd == ("TrackerTeleplot"))
    {
        Teleplot(true);
    }
    else if (cmd.cmd == ("TrackerConfig"))
    {
        // TrackerConfig:300;50;3;10;5
        // prevent changing config if there is less param than needed.
        if (cmd.size <= 4)
            cmd.data[4] = -1;
        if (cmd.size <= 3)
            cmd.data[3] = -1;
        if (cmd.size <= 2)
            cmd.data[2] = -1;
        if (cmd.size <= 1)
            cmd.data[1] = -1;
        if (cmd.size <= 0)
            cmd.data[0] = -1;
        Config(cmd.data[0], cmd.data[1], cmd.data[2], cmd.data[3], cmd.data[4]);
    }
}

bool Tracker::PointIsEqual(PolarPoint a, PolarPoint b) { return (a.x == b.x && a.y == b.y); }

void Tracker::Track(std::vector<PolarPoint>& newPoints)
{
    for (auto& newPoint : newPoints)
    {
        // A point can not be further than 5000.0 as it is the field size
        float best_match = 5000.0;  // hold the closest point to the new one
        int matching_point_index = -1;
        // int first_available_slot = -1;  // free slot to add new point

        // println("Search point:", newPoint);

        int i = 0;
        for (auto& trackPoint : trackedPoints)
        {
            // println("Compare to:", trackPoint.point);

            // TODO remove sqrt and use x and y comparison
            float dist = sqrt(pow(newPoint.x - trackPoint.point.x, 2) + pow(newPoint.y - trackPoint.point.y, 2));
            // println("Distance:", dist);

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
            // println("New point detected, add to tracked");
            trackedPoints.push_back(newPointTracker);

            continue;
        }

        // TODO implémenter un poids par point en fonction du nombre de fois où on le voit
        // dans la limite entre 3 et 5 fois max par seconde
        if (best_match < config.hpf_cutoff)
        {
            // println("This is exactly the same point, update only time");
            trackedPoints[matching_point_index].lastUpdateTime = millis();
            if (trackedPoints[matching_point_index].confidence < config.confidenceMax)
            {
                trackedPoints[matching_point_index].confidence++;
            }
        }
        else
        {
            // print("Updating point ", matching_point_index, " ");
            // print("from ", trackedPoints[matching_point_index].point, "");
            // println("  to ", newPoint, "");
            trackedPoints[matching_point_index].point = newPoint;
            trackedPoints[matching_point_index].hasBeenSent = false;
            trackedPoints[matching_point_index].lastUpdateTime = millis();
            if (trackedPoints[matching_point_index].confidence < config.confidenceMax)
            {
                trackedPoints[matching_point_index].confidence++;
            }
        }
    }
}

void Tracker::Update()
{
    int index = 0;
    for (auto& trackPoint : trackedPoints)
    {
        // TODO plutôt que de faire avec le temps, faire avec le nombre de tours du lidar ?
        if (millis() - trackPoint.lastUpdateTime > IS_TOO_OLD)
        {
            // decrement is faster then increment
            if (trackPoint.confidence >= 0)
            {
                trackPoint.confidence -= 2;
                // update the time to not decrement to fast
                trackPoint.lastUpdateTime = millis();
            }
            else
            {
                // Will be removed later
            }
        }
        index++;
    }

    // Removing the cluster checked
    trackedPoints.erase(remove_if(trackedPoints.begin(), trackedPoints.end(),
                                  [](PointTracker const& pt) { return (pt.confidence < 0); }),
                        trackedPoints.end());
}

void Tracker::SendToRobot()
{
    int index = 0;
    PolarPoint zero = {0, 0, 0, 0, 0};

    // First we sort by distance to get closer obstacle first
    std::sort(trackedPoints.begin(), trackedPoints.end(),
              [](const PointTracker& a, const PointTracker& b) { return a.point.distance < b.point.distance; });

    for (int index = 0; index < 5; index++)
    {
        if (trackedPoints.size() > index && trackedPoints[index].confidence > config.confidenceTrigger)
        {
            robot.WriteSerial(index, trackedPoints[index].point);
            // teleplot("obs", trackedPoints[index].point);
        }
        else
        {
            robot.WriteSerial(index, zero);
            // teleplot("obs", zero);
        }
    }
    /*
        for (auto& trackPoint : trackedPoints)
        {
            if (index < 5 && !trackPoint.hasBeenSent && trackPoint.confidence > config.confidenceTrigger)
            {
                robot.WriteSerial(index, trackPoint.point);
                trackPoint.hasBeenSent = true;
                teleplot("obs", trackPoint.point, index, Level::LEVEL_WARN);
                // println("Send N°" + String(index) + " to Robot : ", trackPoint.point, "", Level::LEVEL_WARN);
            }
            index++;
        }
    */
}

PolarPoint lastSend[5] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};
void Tracker::Teleplot(bool all)
{
    int8_t index = 0;
    PolarPoint zero = {0, 0, 0, 0, 0};

    for (int index = 0; index < 5; index++)
    {
        if (trackedPoints.size() > index && trackedPoints[index].confidence > config.confidenceTrigger)
        {
            if (lastSend[index].x != trackedPoints[index].point.x ||
                lastSend[index].y != trackedPoints[index].point.y || all)
            {
                teleplot("LD06obs", trackedPoints[index].point);
            }
            lastSend[index] = trackedPoints[index].point;
        }
        else
        {
            if (all)
            {
                teleplot("LD06obs", zero);
            }
            lastSend[index] = zero;
        }
    }
    // teleplot("mapBoundaries", MapBoundaries, 4, Level::LEVEL_WARN);
    // teleplot("robot", robot.position, Level::LEVEL_WARN);
}
