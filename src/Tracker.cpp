#include "Tracker.h"

Tracker::Tracker()
{
    Debugger::println("Init Tracker");
}

void Tracker::trackNewObstacle(Point obstacle)
{
    Debugger::println("Obstacle: ");
    Debugger::log(" x: ", obstacle.x, "", VERBOSE);
    Debugger::log(" y: ", obstacle.y, "", VERBOSE);

    tracked_obstacle = obstacle;

    new_obstacle = true;
}

void Tracker::sendObstacleToRobot(Robot robot)
{
    if (new_obstacle)
    {
        Debugger::print("Sent to robot: ");
        Debugger::log("x= ", (int)tracked_obstacle.x, " ", VERBOSE, false);
        Debugger::log("y= ", (int)tracked_obstacle.y, "", VERBOSE);

        robot.WriteSerial(1, tracked_obstacle);
        new_obstacle = false;
    }
}