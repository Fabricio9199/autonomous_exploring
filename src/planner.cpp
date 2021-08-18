#include "planner.h"

/**
 * Class Constructor
 */

Planner::Planner() : private_nh("~")
{
    //Declare Our Subscribers
    

    //Declare Our Publishers
}

/** 
 * Class Destructor
 */

Planner::~Planner() {}

/** 
 * Do Delay
 * t is second
 */

void Planner::PersonalDelay(float t)
{
    double begin = ros::Time::now().toSec(); //Init Count Time
    double currentTime = 0;
    while (currentTime < t)
    {
        currentTime = ros::Time::now().toSec() - begin;
        ros::spinOnce();
    }
}