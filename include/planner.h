#pragma once

#ifndef PROJECT_PLANNER_H
#define PROJECT_PLANNER_H

#include <math.h>
#include <stdlib.h>
#include <cstdlib>
#include <vector>
#include <string>

//ROS Msgs
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ParamSet.h>

#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>

//Set Namespace
using namespace std;

class Planner {

    public:
        Planner();
        ~Planner();
        
        void Loop();
        void PersonalDelay(float t);

    private:
        ros::NodeHandle private_nh;

};

#endif