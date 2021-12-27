#ifndef TR_PLANNER_SDF_H

#define TR_PLANNER_SDF_H

//STL
#include <ctime>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <numeric>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <iostream>
#include <string>
#include "cstdlib"
//egien
#include <Eigen/Eigen>
#include<Eigen/Core>
//opencv
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include "opencv2/core/core.hpp"

// ROS
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
//Drone module
#include "robot_process.h"
//DroneMsgsROS
#include <droneMsgsROS/AliveSignal.h>
#include <droneMsgsROS/dronePose.h>
#include <droneMsgsROS/droneSpeeds.h>
#include <droneMsgsROS/droneTrajectoryRefCommand.h>
#include <droneMsgsROS/droneRefCommand.h>
#include <droneMsgsROS/dronePositionTrajectoryRefCommand.h>
#include <droneMsgsROS/dronePositionRefCommandStamped.h>
#include <droneMsgsROS/droneCommand.h>
#include <droneMsgsROS/droneYawRefCommand.h>
//Path_Smooth
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/trajectory_sampler_node.h>
//Gazebo messages
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SetModelState.h"
//rrt_algorithm
#include "rrtstar.h"
//chomp planner
#include "chomp_opt.h"
//voxblox
#include "voxblox_ros/esdf_server.h"
//rviz_visual
#include <rviz_visual_tools/rviz_visual_tools.h>
//tf
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
//sleep_time
#include <unistd.h>


class TR_PLANNER_SDF : public RobotProcess
{
  public:
    TR_PLANNER_SDF(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);

    ~TR_PLANNER_SDF();
    Vector4d getMapDistance(const Eigen::Vector3d& position) const;

    std::ofstream f_data_recorder;//record data
    std::string file_name;
    int num_episode_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    //sub
    ros::Subscriber uav_pose_velocity_subs_;
    ros::Subscriber laser_scan_subs_;
    //pub
    ros::Publisher candidate_traj_pub;
    ros::Publisher best_traj_pub;
    ros::Publisher final_tra_;
    ros::Publisher chomp_path_pub;
    ros::Publisher tra_visual_;
    ros::Publisher traj_control_pub;
    ros::Publisher pose_reset;
    //service
    ros::ServiceClient gazebo_set_model_state_srv_;

    Vector3d uav_pos;
    nav_msgs::Path chomp_path;
    Eigen::Vector3d goal_point;

    bool msg_receive;

    bool planner_run;

    bool goal_reach;

    bool first_run;

    bool tf_msg_receive;

    bool collision_;

    bool path_feasible;

    bool reach_sub_goal;

    void PoseVelocityCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

    std::vector<std::vector<Node *> > candidate_tra_generate( Eigen::Vector3d& current_position, Eigen::Vector3d& goal_position,
                               int N_max, double s_radius,
                              double X_UP, double X_LOW, double Y_UP, double Y_LOW, double Z_UP, double Z_LOW);

    std::vector<Node *> best_candidate(std::vector<std::vector<Node *> > traj_candi_,
                                       Eigen::Vector3d& current_position, Eigen::Vector3d& goal_position);

    nav_msgs::Path Tra_Optim(std::vector<Node *> best_trajectory);


    void tra_controller(nav_msgs::Path tra_);

    void plot_candidate_tra(std::vector<std::vector<Node *> > traj_);

    void plot_uav(Eigen::Vector3d current_position);

    void plot_best_tra(std::vector<Node *> traj_);

    bool Check_Collision_Point(Eigen::Vector3d point_);

    void mav_tra_generation_(nav_msgs::Path pos, double delta_t,
                             trajectory_msgs::MultiDOFJointTrajectory &my_traj,
                             Eigen::Vector4d current_vel, double vcc_max, double ac_max);

    void GetPosfromTF();

  //goal_reached_checking
    std::vector<float> laser_ranges_;
    float min_range_reset_value_;
    void LaserScanCallback(const sensor_msgs::LaserScan &msg);

  //param_setting
    double X_MIN;
    double Y_MIN;
    double Z_MIN;
    double X_MAX;
    double Y_MAX;
    double Z_MAX;
    double goal_x;
    double goal_y;
    double goal_z;
    double goal_X;
    double goal_Y;
    double goal_Z;
    int Nmax;
    double S_Rad;
    double R_Rad;
    double acmax;
    double vemax;
    double delta_t;
    double RRT_E;
    double random_initial_x;
    double random_goal_x;
    double random_up_lim_y;
    double random_low_lim_y;
    double random_up_lim_y_initial;
    double random_low_lim_y_initial;
  //robot_process_function
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
    void init();
   private:
    // Map!
    voxblox::EsdfServer voxblox_server_;
    // For visualizing things in rviz
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    tf::TransformListener listener_;
    tf::StampedTransform transform;

};

#endif
