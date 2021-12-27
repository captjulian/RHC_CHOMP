//I/O Stream
//std::cout
#include <iostream>
#include <string>
#include "ros/ros.h"

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>



//Gazebo messages

#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SetModelState.h"

ros::Publisher uav_trans_pub;
geometry_msgs::TransformStamped uav_pos;
std_msgs::Header point_cloud_header;

void time_stamp_callback(const sensor_msgs::PointCloud2 &msg)
{
    point_cloud_header = msg.header;
}

void drone_pos_callback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{

//    Eigen::Vector3d position_estimate_W;

    for (int i=0; i<msg->name.size(); i++)
    {
        if (msg->name[i].compare("hummingbird1") == 0)
         {
            uav_pos.header.stamp = point_cloud_header.stamp;
            uav_pos.header.seq = point_cloud_header.seq;
            uav_pos.header.frame_id = "world";
            uav_pos.child_frame_id = point_cloud_header.frame_id;
            uav_pos.transform.translation.x = msg->pose[i].position.x;
            uav_pos.transform.translation.y = msg->pose[i].position.y;
            uav_pos.transform.translation.z = msg->pose[i].position.z;

            uav_pos.transform.rotation.x = msg->pose[i].orientation.x;
            uav_pos.transform.rotation.y = msg->pose[i].orientation.y;
            uav_pos.transform.rotation.z = msg->pose[i].orientation.z;
            uav_pos.transform.rotation.w = msg->pose[i].orientation.w;
         }
    }
   uav_trans_pub.publish(uav_pos);
}



int main(int argc, char **argv)
{
    //Init
    ros::init(argc, argv, "pose_transform_node"); //Say to ROS the name of the node and the parameters
    ros::NodeHandle n; //Este nodo admite argumentos!!

    ros::Subscriber position_from_gazebo = n.subscribe("/gazebo/model_states", 1, drone_pos_callback);
   // ros::Subscriber occupancy2sdf_sub =nh.subscribe("/drone7/move_base/local_costmap/costmap_updates",1,occupancy2sdfCallback);
    ros::Subscriber drone_pos_sub = n.subscribe("/hummingbird1/camera_front_depth/depth/points",1, time_stamp_callback);

    uav_trans_pub = n.advertise<geometry_msgs::TransformStamped>("uav_trans", 1,true);

    //Loop -> Ashyncronous Module
    while(ros::ok())
    {
        ros::spin();
    }

    return 1;
}
