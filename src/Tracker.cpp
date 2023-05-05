#include "Tracker.h"

Tracker::Tracker()
{
    Debugger::println("Init Tracker");
}

void Tracker::trackNewObstacle(Point obstacle)
{
    // Debugger::println("Obstacle: ");
    // Debugger::log(" x: ", obstacle.x, "", VERBOSE);
    // Debugger::log(" y: ", obstacle.y, "", VERBOSE);

    obstacleTracked = obstacle;

    newObstacle = true;
}

void Tracker::sendObstacleToRobot(Robot robot)
{
    if (newObstacle)
    {
        Debugger::print("Obstacle : ");
        Debugger::log("x= ", (int)obstacleTracked.x, " ", VERBOSE, false);
        Debugger::log("y= ", (int)obstacleTracked.y, "", VERBOSE);

        robot.WriteSerial(1, obstacleTracked);
        newObstacle = false;
    }
}