#include "autonomous_exploring.h"

int main(int argc, char **argv)
{
    //////////////////// ROS INITIALIZATION //////////////////////////

    ros::init(argc, argv, "autonomous_exploring");
    ros::NodeHandle nh;
    ROS_INFO( "-----Initialize Autonomous Exploring :)-----");
    Autonomous_Exploring exploring;

    ros::Rate loop_rate(10);
    ros::spinOnce();

    while (ros::ok()) {
        
        exploring.Loop();
        ros::spinOnce();
    }


    return 0;
}

