//I/O Stream
//std::cout
#include <iostream>

#include <string>

#include "sdf_planner.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    //Init
    ros::init(argc, argv, "sdf_planner_node"); //Say to ROS the name of the node and the parameters


    ros::NodeHandle nh_("");
    ros::NodeHandle nh_private_("~");


    TR_PLANNER_SDF sdf_planner(nh_,nh_private_);

     sdf_planner.setUp();
     //sdf_planner.start();

     //ros::Rate r(100);


    while(ros::ok())
    {
        ros::spinOnce();

        sdf_planner.run();
       // r.sleep();
    }

    return 1;

}

