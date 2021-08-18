#pragma once

#ifndef PROJECT_AUTONOMOUS_EXPLORING_H
#define PROJECT_AUTONOMOUS_EXPLORING_H

#include <math.h>
#include <stdlib.h>
#include <cstdlib>
#include <vector>
#include <string>
#include <unistd.h>

//ROS Msgs
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "autonomous_exploring/setGoalPose.h"
#include "mission_uav_assign/setPoint.h"
#include "mission_uav_assign/cancelMission.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ParamSet.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <ompl/config.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"

#include <Eigen/Dense>

//Set Namespace
using namespace octomap;
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;


class Autonomous_Exploring {

    public:
        Autonomous_Exploring();
        ~Autonomous_Exploring();
        
        void Loop();
        void PersonalDelay(float t);
        bool pathDone;

    private:
        ros::NodeHandle private_nh;        
        ros::ServiceClient getOctomapBinary, sendGoaltoUAV, cancelMission;
        
        ros::Publisher pubMarkers; //setPoint, setStatusMission;
        ros::Subscriber getCurrentPose, getOctomap, getOctomapFull, getGoalPosition, getStatusMission;
        ros::ServiceServer mySrvGetGoal;
        
        bool setSrvGoalPose(autonomous_exploring::setGoalPose::Request &req, autonomous_exploring::setGoalPose::Response &resp);

        void callbackGetPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void callbackGetOctomap(const octomap_msgs::Octomap::ConstPtr &msg); 
        void callbackGetOctomapFull(const octomap_msgs::Octomap::ConstPtr &msg);
        void callbackgetGoalPosition(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void callbackgetStatusMission(const std_msgs::Bool::ConstPtr &msg);

        void doPlan();
        bool isStateValid(const ob::State *state);
        void updateMap(std::shared_ptr<fcl::CollisionGeometry> map);
        void pubPathOnRviz(og::PathGeometric* pth);
        void pubPathOnUAV(og::PathGeometric* pth);
        void replan();
        void MonitoringWaypoint();
        void refreshLocalOctomap();
        void setMapStatus();

        bool checkCollision();

        void takeOff(double coordZ);
        void cancelMissionFunction();
        
        double constrainAngle(double x);

        ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);
        string pathsrvOctomapFull, pathsrvOctomapBinary, pathSubPose, pathSubMap, pathsrvSetGoal;

        geometry_msgs::PoseStamped currentPoseStamped, goalPose;
        geometry_msgs::Pose currentPose;
        octomap_msgs::Octomap currentOctomap, currentOctomapFull;
        
        ob::StateSpacePtr space; //Declare our generic space        
        ob::SpaceInformationPtr si; // Construct an instance of  space information from this state space        
        ob::ProblemDefinitionPtr pdef; // Declare our problem definition
        ob::RealVectorBounds bounds;    //Declare our bounds class
        ob::ScopedStatePtr start, goal; //Declare generic start and goals
	    og::PathGeometric* path_smooth = NULL; //Declare our path_smooth
        og::PathSimplifierPtr pathBSpline; //Declare our generic path simplifier
        
        std::shared_ptr<fcl::CollisionGeometry> quadcopter, quadcopterSafe;
	    std::shared_ptr<fcl::CollisionGeometry> treeObjCollision;

        vector<geometry_msgs::Pose> storeSetPoints;

        tf2::Quaternion quatTfStart, quatTfGoal, quatWaypoint;

        OcTree localTree;

        int myMapStatus[200][200][50];

        double prev_goal[3]; // previous goal state

        int idPlanner;
        float xLimit, yLimit, zLimit; //Limits to accepet new waypoint       
        float rangeToDefineSetPointReached, resolutionOctomap;

        bool firstMapGet, flagReplan, setReplan, statusMission, firstPoseDone, pathFound, usePathApproximate;

        const int PI = 3.14159265;

        unsigned int microsecond = 1000000;
};

#endif