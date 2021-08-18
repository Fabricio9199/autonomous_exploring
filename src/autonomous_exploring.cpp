#include "autonomous_exploring.h"

/**
 * Class Constructor
 */

Autonomous_Exploring::Autonomous_Exploring(void) : private_nh("~"), bounds(1), localTree(0.5)
{

    //Read Yaml  
    private_nh.param<string>("pathsrvOctomapFull", pathsrvOctomapFull, "/octomap_full");
    private_nh.param<string>("pathsrvOctomapBinary", pathsrvOctomapBinary, "/octomap_binary");
    private_nh.param<string>("pathSubMap", pathSubMap, "map");
    private_nh.param<string>("pathsrvSetGoal", pathsrvSetGoal, "setsrvGoal");
    
    private_nh.param<float>("xLimit", xLimit, 250);
    private_nh.param<float>("yLimit", yLimit, 250);
    private_nh.param<float>("zLimit", zLimit, 50);

    private_nh.param<float>("rangeToDefineSetPointReached", rangeToDefineSetPointReached, 0.1);
    private_nh.param<float>("resolutionOctomap", resolutionOctomap, 0.5);


    private_nh.param<bool>("setReplan", setReplan, false);    
    private_nh.param<bool>("usePathApproximate", usePathApproximate, false);    

    //Start Variables
    firstMapGet = false;

    //Declare Our Subscribers
    getCurrentPose = private_nh.subscribe<geometry_msgs::PoseStamped>("getPose", 1, &Autonomous_Exploring::callbackGetPose, this);
    getOctomap = private_nh.subscribe<octomap_msgs::Octomap>(pathSubMap, 1, &Autonomous_Exploring::callbackGetOctomap, this);
    //getOctomapFull = private_nh.subscribe<octomap_msgs::Octomap>("/octomap_full", 1, &Autonomous_Exploring::callbackGetOctomapFull, this);
    getGoalPosition = private_nh.subscribe<geometry_msgs::PoseStamped>("setGoal", 1, &Autonomous_Exploring::callbackgetGoalPosition, this);
    getStatusMission = private_nh.subscribe<std_msgs::Bool>("getStatusMission", 1, &Autonomous_Exploring::callbackgetStatusMission, this);
    
    //Declare Our Publishers
    pubMarkers = private_nh.advertise<visualization_msgs::Marker>("markers", 50);

    //Declare Our Services
    mySrvGetGoal = private_nh.advertiseService("srvSetGoalPose", &Autonomous_Exploring::setSrvGoalPose, this);

    //Declare out Local Map
    //localTree = OcTreePtr(new Octree(0.5));

    //Declare Our Clients
    //getOctomapFull = private_nh.serviceClient<octomap_msgs::GetOctomap>(pathsrvOctomapFull);
    getOctomapBinary = private_nh.serviceClient<octomap_msgs::GetOctomap>(pathsrvOctomapBinary);
    sendGoaltoUAV = private_nh.serviceClient<mission_uav_assign::setPoint>(pathsrvSetGoal);
    cancelMission = private_nh.serviceClient<mission_uav_assign::cancelMission>("/mission_uav_assign/srvCancelMission");

    //Define Bounds   
    bounds.resize(3);               //Derifne 3D work space
    bounds.setLow(0,-xLimit);
    bounds.setHigh(0,xLimit);
    bounds.setLow(1,-yLimit);
    bounds.setHigh(1,yLimit);
    bounds.setLow(2, 0);
    bounds.setHigh(2,zLimit);

    space = ob::StateSpacePtr(new ob::SE3StateSpace()); //Declare our Space
    
    space->as<ob::SE3StateSpace>()->setBounds(bounds); //Set our Bounds

    si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));  // construct an instance of  Space Information from this state space
    si->setStateValidityChecker(std::bind(&Autonomous_Exploring::isStateValid, this, std::placeholders::_1 )); //Define our function to validate if any state is collide with any obstacles

    pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si)); // create a Problem Definition
    pdef->setOptimizationObjective(Autonomous_Exploring::getPathLengthObjWithCostToGo(si)); //Define how process optimization is work

    pathBSpline = og::PathSimplifierPtr(new og::PathSimplifier(si));

    quadcopter = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(1.5, 1.5, 1));
    quadcopterSafe = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(2.5, 2.5, 2.5));
    fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.2)));
    treeObjCollision = std::shared_ptr<fcl::CollisionGeometry>(tree);
    
    this->pathDone = false;

    firstPoseDone = false;
    while (!firstPoseDone) PersonalDelay(0.01);    
    ROS_INFO( "----- First Pose Done!! -----" );

    ROS_INFO( "----- All instances for Mission are initialized!! -----" );
    
    takeOff(2); //Check if necessary take off UAV


    //setMapStatus();
}

/** 
 * Class Destructor
 */

Autonomous_Exploring::~Autonomous_Exploring() {}

/** 
 * Our Main Loop
 */
void Autonomous_Exploring::Loop()
{
    PersonalDelay(0.01);
}

/** 
 * Do Delay
 * t is second
 */

void Autonomous_Exploring::PersonalDelay(float t)
{
    ros::spinOnce();
    double begin = ros::Time::now().toSec(); //Init Count Time
    double currentTime = 0;
    while (currentTime < t)
    {
        currentTime = ros::Time::now().toSec() - begin;
        ros::spinOnce();
    }
}

/** 
 * Service to get a Goal Point
 */

bool Autonomous_Exploring::setSrvGoalPose(autonomous_exploring::setGoalPose::Request &req, autonomous_exploring::setGoalPose::Response &resp)
{
   goalPose.pose = req.Pose;

   if (statusMission) cancelMissionFunction();

   doPlan();
   resp.response = true;
   return true;
}

/** 
 * Planner
 */

void Autonomous_Exploring::doPlan()
{
    //Reset Path
    pathFound = false;

    //Define start and Goal
    ob::ScopedState<ob::SE3StateSpace> start(space);		
    ob::ScopedState<ob::SE3StateSpace> goal(space);

    start->setXYZ(currentPoseStamped.pose.position.x,currentPoseStamped.pose.position.y,currentPoseStamped.pose.position.z); //Cartesian Position
    goal->setXYZ(goalPose.pose.position.x, goalPose.pose.position.y,goalPose.pose.position.z); //Cartesian Position

    tf2::convert(currentPoseStamped.pose.orientation , quatTfStart);
    tf2::convert(goalPose.pose.orientation, quatTfGoal);

    cout << "**My Goal** X: " << goal->getX() << " Y: " << goal->getY() << " Z: " << goal->getZ()<< endl;
    cout << "**My Start** X: " << start->getX() << " Y: " << start->getY() << " Z: " << start->getZ()<< endl;

    //start->as<ob::SO3StateSpace::StateType>(1)->setIdentity(); //Orientation Position
    start->rotation().setAxisAngle(currentPoseStamped.pose.orientation.x,currentPoseStamped.pose.orientation.y,currentPoseStamped.pose.orientation.z,currentPoseStamped.pose.orientation.w);
    //goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity(); //Orientation Position
    goal->rotation().setAxisAngle(goalPose.pose.orientation.x, goalPose.pose.orientation.y,goalPose.pose.orientation.z,goalPose.pose.orientation.w);

    pdef->setStartAndGoalStates(start, goal);    	    
	
    //og::InformedRRTstar* rrt = new og::InformedRRTstar(si); // create a planner for the defined space 
    og::RRTstar* rrt = new og::InformedRRTstar(si);

    rrt->setRange(2);
    ob::PlannerPtr plan(rrt);

	plan->setProblemDefinition(pdef); // set the problem we are trying to solve for the planner
	    
	plan->setup(); // perform setup steps for the planner
	    
	//si->printSettings(cout); // print the settings for this space

    //pdef->print(cout); // print the problem settings
	    
	ob::PlannerStatus solved = plan->solve(3); // attempt to solve the problem within three second of planning time

    pathFound = pdef->hasExactSolution(); //Store status of planner

    if (solved && (pathFound || usePathApproximate))
    {
        path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
        pathBSpline->smoothBSpline(*path_smooth,3);

        og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
        
        pubPathOnRviz(path_smooth);
        pubPathOnUAV(path_smooth);
        //path_smooth->printAsMatrix(std::cout);    

        path_smooth->clear();
        pdef->clearSolutionPaths();
    }
    else
    {
        ROS_ERROR("Path not found!");
    }

}

/** 
 * Publish path using Rviz
 */
void Autonomous_Exploring::pubPathOnUAV(og::PathGeometric* pth)
{   
    geometry_msgs::Pose genericWaypoint;
    double angleOrientation;

    for (std::size_t idx = 0; idx < path_smooth->getStateCount(); idx++)
    {
        // cast the abstract state type to the type we expect
        const ob::SE3StateSpace::StateType *se3state = path_smooth->getState(idx)->as<ob::SE3StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
        
        genericWaypoint.position.x = pos->values[0];
        genericWaypoint.position.y = pos->values[1];
        genericWaypoint.position.z = pos->values[2];
        
        if (idx<((path_smooth->getStateCount())-1))
        {
            const ob::SE3StateSpace::StateType *se3state_aux = path_smooth->getState(idx+1)->as<ob::SE3StateSpace::StateType>();
            const ob::RealVectorStateSpace::StateType *pos_aux = se3state_aux->as<ob::RealVectorStateSpace::StateType>(0);
            angleOrientation = constrainAngle((atan2(pos_aux->values[1]-pos->values[1], pos_aux->values[0]-pos->values[0])*180/PI));
            angleOrientation = angleOrientation*PI/180;

            quatWaypoint.setRPY(0,0,angleOrientation);
            tf2::convert(quatWaypoint, genericWaypoint.orientation);

        }else{        
            genericWaypoint.orientation.x = rot->x;
            genericWaypoint.orientation.y = rot->y;
            genericWaypoint.orientation.z = rot->z;
            genericWaypoint.orientation.w = rot->w;
        }

        tf2::convert(genericWaypoint.orientation, quatTfGoal);

        mission_uav_assign::setPoint srv;
        srv.request.Pose = genericWaypoint;                

        if(sendGoaltoUAV.call(srv) && srv.response.response)
        {
            this->storeSetPoints.push_back(genericWaypoint);
            //ROS_INFO("SetPoint added");
        }

    }

    //Send Last Wapypoint
    //genericWaypoint = goalPose;
    //tf2::convert(genericWaypoint.orientation, quatTfGoal);
    //cout << "Waypoint Final: " <<quatTfGoal.getAngle() << endl;

    //mission_uav_assign::setPoint srv;
    //srv.request.Pose = genericWaypoint;                

    //if(sendGoaltoUAV.call(srv) && srv.response.response)
    //{
    //    this->storeSetPoints.push_back(genericWaypoint);
    //    ROS_INFO("SetPoint added");
    //}

}

/** 
 * Cancel a mission
 */

void Autonomous_Exploring::cancelMissionFunction()
{    
    mission_uav_assign::cancelMission srv;
    std_msgs::Bool doCancel;
    doCancel.data = true;
    srv.request.cancelMission = true;

    if(cancelMission.call(srv) && srv.response.response)
    {
        ROS_INFO("Mission Canceled");
    }

}

/** 
 * Publish path using Rviz
 */
void Autonomous_Exploring::pubPathOnRviz(og::PathGeometric* pth)
{    
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    pubMarkers.publish(marker);

    for (std::size_t idx = 0; idx < pth->getStateCount (); idx++)
    {
        // cast the abstract state type to the type we expect
        const ob::SE3StateSpace::StateType *se3state = pth->getState(idx)->as<ob::SE3StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

        marker.header.frame_id = "local_origin";
        marker.header.stamp = ros::Time();
        marker.ns = "path";
        marker.id = idx;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pos->values[0];
        marker.pose.position.y = pos->values[1];
        marker.pose.position.z = pos->values[2];
        marker.pose.orientation.x = rot->x;
        marker.pose.orientation.y = rot->y;
        marker.pose.orientation.z = rot->z;
        marker.pose.orientation.w = rot->w;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        pubMarkers.publish(marker);

        ros::Duration(0.005).sleep();

    }

}

/** 
 * Callback to get a goal Position
 */

void Autonomous_Exploring::callbackgetGoalPosition(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goalPose = *msg;
    goalPose.pose.position.z = rand() % 10 + 1;
    if (statusMission) cancelMissionFunction();

    doPlan();
}

/** 
 * Callback to get state UAV
 */
void Autonomous_Exploring::callbackGetPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    firstPoseDone = true;
    currentPoseStamped = *msg;
    currentPose =  currentPoseStamped.pose ;
    if(statusMission)
         this->MonitoringWaypoint();
    else
        this->storeSetPoints.clear();
    ros::spinOnce();
}

/** 
 * Callback to get Ocomap
 */
void Autonomous_Exploring::callbackGetOctomap(const octomap_msgs::Octomap::ConstPtr &msg)
{
    currentOctomap = *msg;
    
	// convert octree to collision object
	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));

    //  for(int i=-50; i<=100; i++)
    //  {
    //      point3d query(i/10., 0., 1.); 
    //      OcTreeNode* result = tree_oct->search(query);
    //      if (result != NULL) {
    //      cout << "occupancy probability at " << query << ":\t " << result->getOccupancy() << endl;
    //      }
    //      else 
    //      cout << "occupancy probability at " << query << ":\t is unknown" << endl;   
    //  }



    //   for (octomap::OcTree::leaf_iterator it = tree_oct->begin_leafs(); it != tree_oct->end_leafs(); ++it) 
    //   {
    //       std::cout<< "Node center: " << it.getCoordinate();
    //       std::cout<< " value: " << it->getValue()<< "\n";
    //   }

    this->updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));
    if (!firstMapGet)
    {
        firstMapGet = true;
        ROS_INFO("First Map Get!");
    }

    if (setReplan && statusMission)
    {
        replan();
    }


    if (statusMission)
        checkCollision();

    ros::spinOnce();
}

/** 
 * Callback to get Ocomap
 */
void Autonomous_Exploring::callbackGetOctomapFull(const octomap_msgs::Octomap::ConstPtr &msg)
{   
	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));

    for (octomap::OcTree::leaf_iterator it = tree_oct->begin_leafs(); it != tree_oct->end_leafs(); ++it) 
    {
        std::cout<< "Node center: " << it.getCoordinate();
        std::cout<< " value: " << it->getValue()<< "\n";
    }

    ros::spinOnce();
}


/** 
 * Check IF state is Valid
 */
bool Autonomous_Exploring::isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    fcl::CollisionObject treeObj(treeObjCollision);
    fcl::CollisionObject aircraftObject(quadcopter);
    // check validity of state defined by pos & rot
    fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
    fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
    aircraftObject.setTransform(rotation, translation);
    fcl::CollisionRequest requestType(1,false,1,true);
    fcl::CollisionResult collisionResult;
    fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

    return(!collisionResult.isCollision());
    //return true;
}

void Autonomous_Exploring::updateMap(std::shared_ptr<fcl::CollisionGeometry> map)
{
    treeObjCollision = map;
}

/** 
 * Update Status Map
 */
void Autonomous_Exploring::setMapStatus()
{
    float resolution=0.5;
    int maxX = 100;
    int minX = -100;
    int maxY = 100;
    int minY = -100;
    int maxZ = 40;
    int minZ = 0;

    float posX, posY, posZ;
    posX = 10.51;
    posY = 3;
    posZ = 2.99;

    cout << "X: " << floor((posX*10)/(resolution*10)+(maxX/2)) << endl;
    cout << "Y: " << floor((posY*10)/(resolution*10)+(maxX/2)) << endl;
    cout << "Z: " << floor((posZ*10)/(resolution*10)+(maxX/2)) << endl;

    exit(0);

}

ob::OptimizationObjectivePtr Autonomous_Exploring::getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
    return obj;
}

/** 
 * Check Necessity of Replan
 */

void Autonomous_Exploring::replan()
{
    
}

/** 
 * Check check collision during the place
 */

bool Autonomous_Exploring::checkCollision()
{
    vector<geometry_msgs::Pose> localStoreSetPoints; 
    localStoreSetPoints = storeSetPoints;

    fcl::CollisionObject treeObj(treeObjCollision);
    fcl::CollisionObject aircraftObject(quadcopter);
    fcl::CollisionRequest requestType(1,false,1,true);
    fcl::CollisionResult collisionResult;

     for (int i=0; i<localStoreSetPoints.size(); i++)
    {

        fcl::Vec3f translation(localStoreSetPoints[i].position.x,localStoreSetPoints[i].position.y,localStoreSetPoints[i].position.z);
        fcl::Quaternion3f rotation(localStoreSetPoints[i].orientation.w,localStoreSetPoints[i].orientation.x, localStoreSetPoints[i].orientation.y, localStoreSetPoints[i].orientation.z);
        aircraftObject.setTransform(rotation, translation);

        fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);
        if(collisionResult.isCollision())
        {
            ROS_INFO("COOOOOLLISION");
            cancelMissionFunction();
            break;
            return false;
        }
    }


    fcl::Vec3f translationSafe(currentPose.position.x,currentPose.position.y,currentPose.position.z);
    fcl::Quaternion3f rotationSafe(currentPose.orientation.w,currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z);
    aircraftObject.setTransform(rotationSafe, translationSafe);

    fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

    if(collisionResult.isCollision())
    {
        //ROS_INFO("COOOOOLLISION");
        //cancelMissionFunction();
    }
                
    return false;
}

/** 
 * Callback to get status mission
 *
 */
void Autonomous_Exploring::callbackgetStatusMission(const std_msgs::Bool::ConstPtr &msg)
{
    statusMission = msg->data;
}

/**
 * Monitoring Waypoints during Mission
 * 
 */
 void Autonomous_Exploring::MonitoringWaypoint()
 {
        if (this->storeSetPoints.size()>=1)
        { 
            if (this->storeSetPoints.size()==1)
            {                
                this->storeSetPoints.erase(this->storeSetPoints.begin());
                ROS_INFO("Goal Reached!!");
            }
            else{
                long double myDistance = sqrt(pow(currentPoseStamped.pose.position.x - storeSetPoints.front().position.x, 2.0) + pow(currentPoseStamped.pose.position.y - storeSetPoints.front().position.y, 2.0) + pow(currentPoseStamped.pose.position.z - storeSetPoints.front().position.z, 2.0));
                if (myDistance < rangeToDefineSetPointReached)
                {
                    this->storeSetPoints.erase(this->storeSetPoints.begin());
                    //ROS_INFO("Waypoint deleted, now we have %ld waypoints", this->storeSetPoints.size());        
                }
            }
        }
 }


/** 
 * Refresh Local Octomap
 *
 */
void Autonomous_Exploring::refreshLocalOctomap()
{
    
}


/**
 * Wraping angle orientation
 * 
 */

 double Autonomous_Exploring::constrainAngle(double x){
    x = fmod(x + 180,360);
    if (x < 0)
        x += 360;
    return x - 180;
}

/**
 * take off UAV
 * 
 */

void Autonomous_Exploring::takeOff(double coordZ)
{
    geometry_msgs::Pose genericWaypoint;

    if(currentPose.position.z < 0.2)
    {
        cout << currentPose.position.z << endl;
        genericWaypoint.position.x = currentPose.position.x;
        genericWaypoint.position.y = currentPose.position.y;
        genericWaypoint.position.z = coordZ;
        genericWaypoint.orientation = currentPose.orientation;

        mission_uav_assign::setPoint srv;
        srv.request.Pose = genericWaypoint;                

        if(sendGoaltoUAV.call(srv) && srv.response.response)
        {
            this->storeSetPoints.push_back(genericWaypoint);
            ROS_INFO("UAV Take OFF");
        }
    }

}