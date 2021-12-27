#include<sdf_planner.h>


TR_PLANNER_SDF::TR_PLANNER_SDF(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      voxblox_server_(nh_, nh_private_) {

  double robot_radius = 0.3;
  voxblox_server_.setTraversabilityRadius(robot_radius);
  voxblox_server_.publishTraversable();

//  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","/rviz_visual_markers"));

}

TR_PLANNER_SDF::~TR_PLANNER_SDF()
{



    f_data_recorder.close();


}

void TR_PLANNER_SDF::init()
{

    ros::param::get("~xmax", X_MAX);
    if(X_MAX == 0)
        X_MAX = 6;
    ros::param::get("~xmin", X_MIN);
    if(X_MIN == 0)
        X_MIN = -3;
    ros::param::get("~ymax", Y_MAX);
    if(Y_MAX == 0)
        Y_MAX = 3;
    ros::param::get("~ymin", Y_MIN);
    if(Y_MIN == 0)
        Y_MIN = -3;
    ros::param::get("~zmax", Z_MAX);
    if(Z_MAX == 0)
        Z_MAX = 1.5;
    ros::param::get("~zmin", Z_MIN);
    if(Z_MIN == 0)
        Z_MIN = 0.25;
    ros::param::get("~g_x", goal_X);
    if(goal_x == 0)
        goal_x = 5;
    ros::param::get("~g_y", goal_Y);
    if(goal_y == 0)
        goal_y = 0;
    ros::param::get("~g_z", goal_Z);
    if(goal_z == 0)
        goal_z = 0.5;
    ros::param::get("~max_n", Nmax);
    if(Nmax == 0)
        Nmax = 40;
    ros::param::get("~s_rad", S_Rad);
    if(S_Rad == 0)
        S_Rad = 3;
    ros::param::get("~r_rad", R_Rad);
    if(R_Rad == 0)
        R_Rad = 0.3;
    ros::param::get("~acmax", acmax);
    if(acmax == 0)
        acmax = 5;
    ros::param::get("~vemax", vemax);
    if(vemax == 0)
        vemax = 2;
    ros::param::get("~delta_t", delta_t);
    if(delta_t == 0)
        delta_t = 0.1;
    ros::param::get("~RRT_E", RRT_E);
    if(RRT_E == 0)
        RRT_E = 0.5;
    ros::param::get("~random_initial_x", random_initial_x);
    if(random_initial_x == 0)
        random_initial_x = 0.0;
    ros::param::get("~random_goal_x", random_goal_x);
    if(random_goal_x == 0)
        random_goal_x = 50;
    ros::param::get("~random_up_lim_y", random_up_lim_y);
    if(random_up_lim_y == 0)
        random_up_lim_y = 35;
    ros::param::get("~random_low_lim_y", random_low_lim_y);
    if(random_low_lim_y == 0)
        random_low_lim_y = 15;
    ros::param::get("~random_up_lim_y_initial", random_up_lim_y_initial);
    if(random_up_lim_y_initial == 0)
        random_up_lim_y_initial = 35;
    ros::param::get("~random_low_lim_y_initial", random_low_lim_y_initial);
    if(random_low_lim_y_initial == 0)
        random_low_lim_y_initial = 15;
    ros::param::get("~file_name", file_name);
    if(file_name == "")
        file_name = "/home/liang/planner_data_record/document.txt";
   tf_msg_receive = false;
   msg_receive = false;
   planner_run = true;
   goal_reach = false;
   first_run = true;
   collision_ = false;
   path_feasible = true;
   reach_sub_goal = false;
   num_episode_ = 1;
   f_data_recorder.open(file_name);
   f_data_recorder<<"time"<<"\t"<<"num_episode"<<"\t"<<"goal_reached"<<"\t"<<"uav_posx"<<"\t"<<"uav_posy"<<"\t"<<"uav_posz"<<"\t"<<
                        "target_posx"<<"\t"<<"target_posy"<<"\t"<<"target_posz"<<"\t"<<
                        "speed_x"<<"\t"<<"speed_y"<<"\t"<<"speed_z"<<std::endl;

}


void TR_PLANNER_SDF::ownSetUp()
{

    this ->init();
}

void TR_PLANNER_SDF::ownStop()
{

}

void TR_PLANNER_SDF::ownStart()
{


    //Subscribers
    uav_pose_velocity_subs_ = nh_private_.subscribe("/gazebo/model_states",1,&TR_PLANNER_SDF::PoseVelocityCallback, this);
    laser_scan_subs_ = nh_private_.subscribe("/drone7/scan",1, &TR_PLANNER_SDF::LaserScanCallback,this);

    //Publishers
    candidate_traj_pub = nh_private_.advertise<visualization_msgs::MarkerArray>("candidate_traj",1,true);
    tra_visual_ = nh_private_.advertise<visualization_msgs::MarkerArray>("trajectory",1);
    best_traj_pub = nh_private_.advertise<visualization_msgs::Marker>("best_traj_", 1, true);
    final_tra_ = nh_private_.advertise<nav_msgs::Path>("final_tra_", 1, true);
    traj_control_pub = nh_private_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/hummingbird1/command/trajectory", 1, true);
    pose_reset = nh_private_.advertise<geometry_msgs::PoseStamped>("/hummingbird1/command/pose", 1, true);

    //Service
    gazebo_set_model_state_srv_ = nh_private_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");


}

void TR_PLANNER_SDF::ownRun()
{

    GetPosfromTF();

    if(tf_msg_receive == true)
    {
        if(msg_receive == true)
        {
            goal_point.x() = goal_X;
            goal_point.y() = goal_Y;
            goal_point.z() = goal_Z;
            std::cout << "goal_x ==" << goal_point.x()
                      << "goal_y ==" << goal_point.y()
                      << "goal_z ==" << goal_point.z()<<std::endl;
            GetPosfromTF();

            msg_receive = false;
            sleep(2);
        }
        if(first_run == true)
        {
            goal_point.x() = goal_X;
            goal_point.y() = goal_Y;
            goal_point.z() = goal_Z;
            first_run = false;
        }

        if( planner_run == true && goal_reach == false)
        {

            chomp_path.poses.clear();
            trajectory_msgs::MultiDOFJointTrajectory my_traj;

            std::vector<std::vector<Node *> > trajectory_candidate;
            std::vector<Node *> best_trajectory;

            int N_max = Nmax;
            double radius = S_Rad;
            double x_up = X_MAX;
            double x_low = X_MIN;
            double y_up = Y_MAX;
            double y_low = Y_MIN;
            double z_up = Z_MAX;
            double z_low = Z_MIN;
            //plot_uav(uav_pos);

            trajectory_candidate = candidate_tra_generate(uav_pos, goal_point, N_max, radius, x_up, x_low,
                                                          y_up, y_low, z_up, z_low);
            std::cout << "getting candidate trajectory !" << std::endl;
            best_trajectory = best_candidate(trajectory_candidate,uav_pos,goal_point);
            std::cout << "getting best trajrctory !" << std::endl;
//            int size_traj_ = best_trajectory.size();

//            for(int j = 1; j < size_traj_-1; ++j)
//            {
//                chomp_path.header.frame_id = "world";
//                geometry_msgs::PoseStamped traj_pose;
//                traj_pose.header.frame_id = "world";
//                traj_pose.pose.position.x = best_trajectory[size_traj_-1-j]->position.x();
//                traj_pose.pose.position.y = best_trajectory[size_traj_-1-j]->position.y();
//                traj_pose.pose.position.z = best_trajectory[size_traj_-1-j]->position.z();
//                chomp_path.poses.push_back(traj_pose);

//            }
            chomp_path = Tra_Optim(best_trajectory);
            for(int i = 0; i < chomp_path.poses.size(); i++)
            {
                std::cout << "chomp_path_x ==" << chomp_path.poses[i].pose.position.x
                          << "chomp_path_y ==" << chomp_path.poses[i].pose.position.y
                          << "chomp_path_z ==" << chomp_path.poses[i].pose.position.z << std::endl;
                if(!(std::isfinite(chomp_path.poses[i].pose.position.x)&&
                     std::isfinite(chomp_path.poses[i].pose.position.y) && std::isfinite(chomp_path.poses[i].pose.position.z))){

                    std::cout << " * * * * PATH VALUE IS NAN  * * * * " << std::endl;
                    path_feasible = false;
                    break;

                }
                if((chomp_path.poses[i].pose.position.x < (X_MIN- 1) || chomp_path.poses[i].pose.position.x > (X_MAX + 1)) ||
                   (chomp_path.poses[i].pose.position.y < (Y_MIN- 1) || chomp_path.poses[i].pose.position.y > (Y_MAX + 1)) ||
                   (chomp_path.poses[i].pose.position.z < 0 || chomp_path.poses[i].pose.position.z > 3)){

                    std::cout << " * * * * PATH VALUE IS FALSE  * * * * " << std::endl;
                    path_feasible = false;
                    break;

                }
            }
            if(path_feasible == true)
            {
                Eigen::Vector4d current_vel = Eigen::Vector4d::Zero();
                mav_tra_generation_(chomp_path,delta_t,my_traj,current_vel,vemax,acmax);

                final_tra_.publish(chomp_path);
                traj_control_pub.publish(my_traj);
                planner_run = false;
            }
            path_feasible = true;
        }
        else
        {

        }

        if(goal_reach == false)
        {

            double delta_x = uav_pos.x() - goal_point.x();
            double delta_y = uav_pos.y() - goal_point.y();
            double delta_z = uav_pos.z() - goal_point.z();
            double dist_to_goal = sqrt(pow(delta_x,2) + pow(delta_y,2) + pow(delta_z,2));

            if(dist_to_goal <= 1)
            {
                std::cout << "reach the goal !!" << std::endl;

                planner_run = false;
                goal_reach = true;

            }
            else
            {
                if(chomp_path.poses.size() > 0)
                {
                    double delta_x = uav_pos.x() - chomp_path.poses[chomp_path.poses.size() - 1].pose.position.x;
                    double delta_y = uav_pos.y() - chomp_path.poses[chomp_path.poses.size() - 1].pose.position.y;
                    double delta_z = uav_pos.z() - chomp_path.poses[chomp_path.poses.size() - 1].pose.position.z;
                    double dist_to_sub_goal = sqrt(pow(delta_x,2) + pow(delta_y,2) + pow(delta_z,2));
                    if(dist_to_sub_goal <= 0.25)
                    {

                        planner_run = true;
                    }

                    for(int chomp_num = 0; chomp_num < chomp_path.poses.size(); chomp_num ++)
                    {
                        Vector3d point_;
                        point_.x() = chomp_path.poses[chomp_num].pose.position.x;
                        point_.y() = chomp_path.poses[chomp_num].pose.position.y;
                        point_.z() = chomp_path.poses[chomp_num].pose.position.z;

                        if(!Check_Collision_Point(point_))
                        {
                            planner_run = true;
                            break;
                        }
                    }
                }
                else
                {
                    std::cout << "no chomp path !!" << std::endl;
                    planner_run = true;
                }

            }
        }
//        else
//        {
//            if(chomp_path.poses.size() > 0)
//            {
//                double delta_x = uav_pos.x() - chomp_path.poses[chomp_path.poses.size() - 1].pose.position.x;
//                double delta_y = uav_pos.y() - chomp_path.poses[chomp_path.poses.size() - 1].pose.position.y;
//                double delta_z = uav_pos.z() - chomp_path.poses[chomp_path.poses.size() - 1].pose.position.z;
//                double dist_to_sub_goal = sqrt(pow(delta_x,2) + pow(delta_y,2) + pow(delta_z,2));
//                if(dist_to_sub_goal <= 0.5)
//                {
//                    std::cout << "running planner!" << std::endl;

//                    reach_sub_goal = true;
//                }
//            }

//        }
        tf_msg_receive = false;
    }

}


void TR_PLANNER_SDF::LaserScanCallback(const sensor_msgs::LaserScan &msg)
{

    laser_ranges_.clear();

    laser_ranges_.insert(laser_ranges_.end(), msg.ranges.begin(), msg.ranges.end());

    min_range_reset_value_ = R_Rad-0.1;

    if ( std::any_of(laser_ranges_.begin(), laser_ranges_.end(), [this](float i){return i<min_range_reset_value_;}) )
    {

        std::cout<<"MIN DISTANCE reached!"<<std::endl;


        collision_ = true;

    }
}

Eigen::Vector4d TR_PLANNER_SDF::getMapDistance(const Eigen::Vector3d &position) const {

  Eigen::Vector4d dist_and_grad;
  if (!voxblox_server_.getEsdfMapPtr()) {

    return dist_and_grad;
  }
  double distance = 0.0;
  Eigen::Vector3d gradient = Eigen::Vector3d::Zero();

  if (!voxblox_server_.getEsdfMapPtr()->getDistanceAndGradientAtPosition(position,&distance,&gradient)) {

    return dist_and_grad;
  }

  dist_and_grad(0) = gradient(0);
  dist_and_grad(1) = gradient(1);
  dist_and_grad(2) = gradient(2);
  dist_and_grad(3) = distance;

  return dist_and_grad;
}

void TR_PLANNER_SDF::PoseVelocityCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{

    cv::Point3f uav_position, uav_linear_vel;
    for (int i=0; i<msg->name.size(); i++)
    {
        if (msg->name[i].compare("hummingbird1") == 0)
        {

            uav_position.x = msg->pose[i].position.x;
            uav_position.y = msg->pose[i].position.y;
            uav_position.z = msg->pose[i].position.z;

            uav_linear_vel.x = msg->twist[i].linear.x;
            uav_linear_vel.y = msg->twist[i].linear.y;
            uav_linear_vel.z = msg->twist[i].linear.z;
            break;

        }
    }

    double time_now = (double) ros::Time::now().sec + ((double) ros::Time::now().nsec / (double) 1E9);
    f_data_recorder<<time_now<<"\t"<<num_episode_<<"\t"<<goal_reach<<"\t"<<
                             uav_position.x<<"\t"<<uav_position.y<<"\t"<<
                          uav_position.z<<"\t"<<goal_X<<"\t"<<
                             goal_Y<<"\t"<<goal_Z<<"\t"<<
                             uav_linear_vel.x<<"\t"<<uav_linear_vel.y<<"\t"<<uav_linear_vel.z<<std::endl;

    srand (time(NULL));

    if((/*reach_sub_goal == true &&*/ goal_reach == true) || collision_ == true)
    {

        double lower_lim_y_init = random_low_lim_y_initial;
        double upper_lim_y_init = random_up_lim_y_initial;
        double rand_pos_y_init = lower_lim_y_init +
                static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_y_init-lower_lim_y_init)));
        double lower_lim_y_goal = random_low_lim_y;
        double upper_lim_y_goal = random_up_lim_y;

        double rand_pos_y_goal = lower_lim_y_goal +
                static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_lim_y_goal-lower_lim_y_goal)));
        gazebo_msgs::SetModelState uav_pose_reset;
        uav_pose_reset.request.model_state.model_name = "hummingbird1";
        uav_pose_reset.request.model_state.pose.position.x = random_initial_x;
        uav_pose_reset.request.model_state.pose.position.y = rand_pos_y_init;
        uav_pose_reset.request.model_state.pose.position.z = 1.0;
        if (gazebo_set_model_state_srv_.call(uav_pose_reset))
        {

        }
        else{
            ROS_ERROR("ENV_INFO: Failed to call set model state");
        }


        geometry_msgs::PoseStamped pose_reset_p;
        pose_reset_p.pose.position.x = random_initial_x;
        pose_reset_p.pose.position.y = rand_pos_y_init;
        pose_reset_p.pose.position.z = 1.0;


        pose_reset.publish(pose_reset_p);

        gazebo_msgs::SetModelState goal_pose_reset;
        goal_pose_reset.request.model_state.model_name = "goal";
        goal_pose_reset.request.model_state.pose.position.x = random_goal_x;
        goal_pose_reset.request.model_state.pose.position.y = rand_pos_y_goal;
        goal_pose_reset.request.model_state.pose.position.z = 0.0;
        if (gazebo_set_model_state_srv_.call(goal_pose_reset))
        {

        }
        else{
            ROS_ERROR("ENV_INFO: Failed to call set model state");
        }

        goal_X = random_goal_x;
        goal_Y = rand_pos_y_goal;
        goal_Z = 1.0;

        std::cout << "y_goal_up =" << goal_Y << std::endl;

        chomp_path.poses.clear();
        msg_receive = true;

        if(goal_reach = true)
        {
            goal_reach = false;
           // reach_sub_goal = false;
        }

        if(collision_ = true)
        {
           collision_ = false;
        }
        num_episode_++;

    }


}

void TR_PLANNER_SDF::GetPosfromTF()
{
    try{
            listener_.lookupTransform("/world", "/hummingbird1/base_link",
                                          ros::Time(0), transform);

//            std::cout << " * * * * TF   * * * * " << std::endl;
//            std::cout << "to :" << to << std::endl;
//            std::cout << "from :" << from << std::endl;

            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();
            double z = transform.getOrigin().z();


//            std::cout << "x :" << x << std::endl;
//            std::cout << "y :" << y << std::endl;
//            std::cout << "z :" << z << std::endl;

            if(!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))){

                std::cout << " * * * * TF VALUE IS NAN  * * * * " << std::endl;

            }
            else{
                uav_pos.x() = x;
                uav_pos.y() = y;
                uav_pos.z() = z;
            }
            tf_msg_receive = true;
        }

        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
}

std::vector<std::vector<Node *> > TR_PLANNER_SDF::candidate_tra_generate(Vector3d &current_position, Vector3d &goal_position, int N_max, double s_radius,
                                            double X_UP, double X_LOW, double Y_UP, double Y_LOW, double Z_UP, double Z_LOW)
{

    RRTSTAR rrt_star_planner;

    if(current_position.x()< 0.001 && current_position.x() > -0.001)
    {
        current_position.x() = 0;
    }

    if(current_position.y()< 0.001 && current_position.y() > -0.001)
    {
        current_position.y() = 0;
    }

    if(current_position.z()< 0.001 && current_position.z() > -0.001)
    {
        current_position.z() = 0;
    }

    rrt_star_planner.setStepSize(RRT_E);
    rrt_star_planner.setSTART(current_position.x(),current_position.y(), current_position.z());
    rrt_star_planner.setEND(goal_position.x(),goal_position.y(), goal_position.z());
    srand (time(NULL));

    double XMAX = current_position.x() + s_radius;
    double XMIN = current_position.x() - s_radius;;//only can see the obstacle in the front

    double YMAX = current_position.y() + s_radius;
    double YMIN = current_position.y() - s_radius;

    double ZMAX = current_position.z() + s_radius;
    double ZMIN = current_position.z() - s_radius;

    if(XMAX > X_UP)
    {
        XMAX = X_UP;
    }
    if(XMIN < X_LOW)
    {
        XMIN = X_LOW;
    }
    if(YMAX > Y_UP)
    {
        YMAX = Y_UP;
    }
    if(YMIN < Y_LOW)
    {
        YMIN = Y_LOW;
    }
    if(ZMAX > Z_UP)
    {
        ZMAX = Z_UP;
    }
    if(ZMIN < Z_LOW)
    {
        ZMIN = Z_LOW;
    }


    int N_T = rrt_star_planner.nodes.size(); //the number of nodes in the Rapid Random Tree

    while(N_T < N_max)
    {


        Node* q;

        double x_0 = (double)rand()/(RAND_MAX + 1.0)*(XMAX-XMIN) + XMIN;
        double y_0 = (double)rand()/(RAND_MAX + 1.0)*(YMAX-YMIN) + YMIN;
        double z_0 = (double)rand()/(RAND_MAX + 1.0)*(ZMAX-ZMIN) + ZMIN;

        Vector3d point(x_0, y_0, z_0);
        double orient = rand()%1000 /1000.0 * 2 * 3.142;

        q = new Node;
        q->position = point;
        q->orientation = orient;

        Node *qNearest =  rrt_star_planner.nearest(q->position);

        if ( rrt_star_planner.distance(q->position, qNearest->position) >  rrt_star_planner.step_size)
        {

            Vector4d newConfigPosOrient;

            newConfigPosOrient = rrt_star_planner.newConfig(q, qNearest);


            Vector3d newConfigPos(newConfigPosOrient.x(), newConfigPosOrient.y(), newConfigPosOrient.z());

            if(Check_Collision_Point(newConfigPos))
            {

                Node *qNew = new Node;
                qNew->position = newConfigPos;
                qNew->orientation = 0;
                vector<Node *> Qnear;
                rrt_star_planner.near(qNew->position,  rrt_star_planner.step_size, Qnear);
                Node *qMin = qNearest;
                double cmin =  rrt_star_planner.Cost(qNearest) +  rrt_star_planner.PathCost(qNearest, qNew);

                for(int j = 0; j < Qnear.size(); j++){
                    Node *qNear = Qnear[j];
                    if(( rrt_star_planner.Cost(qNear)+rrt_star_planner.PathCost(qNear, qNew)) < cmin ){
                        qMin = qNear; cmin =  rrt_star_planner.Cost(qNear)+ rrt_star_planner.PathCost(qNear, qNew);
                    }
                }
                rrt_star_planner.add(qMin, qNew);

                for(int j = 0; j < Qnear.size(); j++){
                    Node *qNear = Qnear[j];
                    if(( rrt_star_planner.Cost(qNew)+ rrt_star_planner.PathCost(qNew, qNear)) <  rrt_star_planner.Cost(qNear) ){
                        Node *qParent = qNear->parent;
                        // Remove edge between qParent and qNear
                        qParent->children.erase(std::remove(qParent->children.begin(), qParent->children.end(), qNear), qParent->children.end());
                        // Add edge between qNew and qNear
                        qNear->cost =  rrt_star_planner.Cost(qNew) +  rrt_star_planner.PathCost(qNew, qNear);
                        qNear->parent = qNew;
                        qNew->children.push_back(qNear);

                    }

                }
                N_T ++;
            }

        }

    }


    std::vector<std::vector<Node *> > candidate_tra;


    for(int n_ = 0; n_ < rrt_star_planner.nodes.size(); n_++)
    {


        if(rrt_star_planner.nodes[n_]->children.empty())
        {
            Node *q;
            std::vector<Node *> tra_;
            q = rrt_star_planner.nodes[n_];

            while (q != NULL) {

                tra_.push_back(q);


                q = q->parent;
            }
            if(tra_.size() >= 4)
            {
            candidate_tra.push_back(tra_);
            }
            tra_.clear();

        }
    }

    plot_candidate_tra(candidate_tra);

    return candidate_tra;

}

std::vector<Node *> TR_PLANNER_SDF::best_candidate(std::vector<std::vector<Node *> > traj_candi_,
                                                   Eigen::Vector3d& current_position, Eigen::Vector3d& goal_position)
{
    CHOMP_OPT chomp_planner;
    int size_candidate = traj_candi_.size();

    double err = 100;
    int dim = 3;                                                                  // hardcoded param
    std::vector<Node *> traj_;
    int traj_num;
    cv::Point3d goal;
    goal.x = goal_position(0);
    goal.y = goal_position(1);
    goal.z = goal_position(2);
    for(int i = 0; i < size_candidate; ++i)
    {
        int size_traj_ = traj_candi_[i].size();
        Eigen::VectorXd xi(size_traj_*dim);
        Eigen::VectorXd gradient(size_traj_*dim);
        Eigen::VectorXd distance(size_traj_);
        double errd;
        for(int j = 0; j < size_traj_; ++j)
        {
            Eigen::VectorXd dxi(dim);
            Eigen::Vector4d gra_and_dist;
            Eigen::Vector3d dgrad;

            dxi << traj_candi_[i][j]->position.x(), traj_candi_[i][j]->position.y(), traj_candi_[i][j]->position.z();

            xi.block(dim * j, 0, dim, 1) = dxi;

            gra_and_dist = getMapDistance(dxi);

            dgrad << gra_and_dist(0),gra_and_dist(1),gra_and_dist(2);
            gradient.block(dim * j, 0, dim, 1) = dgrad;
            distance(j) = gra_and_dist(3);

        }

        //chomp_planner.init_chomp(current_position,goal_position,xi);


        Eigen::Vector3d qs;
        Eigen::Vector3d qe;
        qs<< xi(size_traj_*dim - 3), xi(size_traj_*dim - 2), xi(size_traj_*dim-1);
        qe<< xi(0), xi(1), xi(2);


        errd = chomp_planner.chomp_goal_bias(qs,qe,xi,goal,gradient,distance);


        if(errd < err)
        {
            err = errd;

            traj_num = i;
        }
    }


    traj_ = traj_candi_[traj_num];

    plot_best_tra(traj_);

    return traj_;

}

bool TR_PLANNER_SDF::Check_Collision_Point(Vector3d point_)
{
    Eigen::Vector4d gra_and_dist_;

    gra_and_dist_ = getMapDistance(point_);

    double dist = gra_and_dist_(3);

    if(dist >= (R_Rad + 0.05))
    {
        return true;
    }
    else
    {
        return false;
    }
}

nav_msgs::Path TR_PLANNER_SDF::Tra_Optim(std::vector<Node *> best_trajectory)
{
    nav_msgs::Path chomp_path;
    CHOMP_OPT traj_opt;
    double err_chomp = 1000;
    int dim = 3;
    int size_traj_ = best_trajectory.size();
    int mid_p_num = size_traj_-2;
    Eigen::VectorXd xi((mid_p_num)*dim);
    Eigen::VectorXd grad((mid_p_num)*dim);
    Eigen::VectorXd dist((mid_p_num));
    for(int j = 1; j < size_traj_-1; ++j)
    {
        Eigen::VectorXd dxi(dim);
        Eigen::Vector4d gra_and_dist;
        Eigen::Vector3d dgrad;

        dxi << best_trajectory[size_traj_-1-j]->position.x(), best_trajectory[size_traj_-1-j]->position.y(),
                best_trajectory[size_traj_-1-j]->position.z();

        xi.block(dim * (j-1), 0, dim, 1) = dxi;

        gra_and_dist = getMapDistance(dxi);
        dgrad << gra_and_dist(0),gra_and_dist(1),gra_and_dist(2);
        grad.block(dim * (j-1), 0, dim, 1) = dgrad;

        dist((j-1)) = gra_and_dist(3);



    }

    Eigen::Vector3d qs;
    Eigen::Vector3d qe;
    qs<<best_trajectory[0]->position.x(), best_trajectory[0]->position.y(), best_trajectory[0]->position.z();
    qe<<best_trajectory[size_traj_-1]->position.x(),
            best_trajectory[size_traj_-1]->position.y(), best_trajectory[size_traj_-1]->position.z();

    traj_opt.init_chomp(qs,qe,xi);


    for(int ii = 0; ii < 600; ii++)                         ///hardcode program
    {

        err_chomp = traj_opt.chomp_iteration(qs, qe, xi, grad, dist);
        int xi_size = xi.rows()/dim;
        for(int jj = 0; jj < xi_size; jj++)
        {
            Eigen::VectorXd dxi_(dim);
            Eigen::Vector4d gra_and_dist_;
            Eigen::Vector3d dgrad_;

            dxi_(0) = xi(jj*dim);
            dxi_(1) = xi(jj*dim+1); ///big bug need fix
            dxi_(2) = xi(jj*dim+2);
            gra_and_dist_ = getMapDistance(dxi_);

            dgrad_ << gra_and_dist_(0),gra_and_dist_(1),gra_and_dist_(2);
            grad.block(dim * jj, 0, dim, 1) = dgrad_;
            dist(jj) = gra_and_dist_(3);
            //std::cout << "xi == " << xi <<std::endl;
        }

        //std::cout << "erro_chomp =="<< err_chomp << std::endl;

        if(err_chomp < 0.0001)
        {
            break;
        }
    }

    traj_opt.generateTrajectory(qs,qe,xi,chomp_path);

    return chomp_path;
}

void TR_PLANNER_SDF::mav_tra_generation_(nav_msgs::Path pos, double delta_t,
                                                      trajectory_msgs::MultiDOFJointTrajectory &my_traj,
                                                      Eigen::Vector4d current_vel, double vcc_max, double ac_max)
{

    mav_trajectory_generation::Vertex::Vector vertices;
    const int dimension = 4;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);
    start.makeStartOrEnd(Eigen::Vector4d(pos.poses[0].pose.position.x, pos.poses[0].pose.position.y,
            pos.poses[0].pose.position.z, 0), derivative_to_optimize);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_vel);
    //start.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, current_acc);
    vertices.push_back(start);
    int pos_size = pos.poses.size() - 1;
    //int goal_size = pos.size()-1;
    if(pos_size == 1)
    {
        end.makeStartOrEnd(Eigen::Vector4d(pos.poses[pos_size].pose.position.x, pos.poses[pos_size].pose.position.y,
                pos.poses[pos_size].pose.position.z, 0), derivative_to_optimize);
        vertices.push_back(end);
    }
    else
    {
        for(int i = 1; i < pos_size; i++)
        {
            middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                                 Eigen::Vector4d(pos.poses[i].pose.position.x, pos.poses[i].pose.position.y,
                                                 pos.poses[i].pose.position.z, 0));

            vertices.push_back(middle);
        }

        end.makeStartOrEnd(Eigen::Vector4d(pos.poses[pos_size].pose.position.x, pos.poses[pos_size].pose.position.y,
                pos.poses[pos_size].pose.position.z, 0), derivative_to_optimize);
        vertices.push_back(end);
    }

    std::vector<double> segment_times;
    const double v_max = vcc_max;
    const double a_max = ac_max;

    const double magic_fabian_constant = 10; // A tuning parameter.
    segment_times = mav_trajectory_generation::estimateSegmentTimesNfabian(vertices, v_max, a_max, magic_fabian_constant);

    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.max_iterations = 1000;
    parameters.f_rel = 0.05;
    parameters.x_rel = 0.1;
    parameters.time_penalty = 500.0;
    parameters.initial_stepsize_rel = 0.1;
    parameters.inequality_constraint_tolerance = 0.1;
    mav_trajectory_generation::NonlinearOptimizationParameters::kRichterTimeAndConstraints;

    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
    //opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

    ROS_INFO("Performing optimization...");
    opt.optimize();

    mav_trajectory_generation::Segment::Vector segments;
    opt.getPolynomialOptimizationRef().getSegments(&segments);
    ROS_INFO("Done.");
    mav_trajectory_generation::Trajectory trajectory;
    opt.getTrajectory(&trajectory);

    mav_msgs::EigenTrajectoryPoint::Vector states;

    double sampling_interval = delta_t;
    bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);
    if(success)
    {
        for(int j = 0; j< states.size() ; j++)
        {
            trajectory_msgs::MultiDOFJointTrajectoryPoint my_point;
            geometry_msgs::Transform position_vector;

            position_vector.translation.x = states[j].position_W[0];
            position_vector.translation.y = states[j].position_W[1];
            position_vector.translation.z = states[j].position_W[2];


//            position_vector.rotation.x = states[j].orientation_W_B.x();
//            position_vector.rotation.y = states[j].orientation_W_B.y();
//            position_vector.rotation.z = states[j].orientation_W_B.z();
//            position_vector.rotation.w = states[j].orientation_W_B.w();

            my_point.transforms.push_back(position_vector);

            geometry_msgs::Twist vel_vector;

            vel_vector.linear.x = states[j].velocity_W[0];
            vel_vector.linear.y = states[j].velocity_W[1];
            vel_vector.linear.z = states[j].velocity_W[2];

//            vel_vector.angular.x = states[j].angular_velocity_W[0];
//            vel_vector.angular.y = states[j].angular_velocity_W[1];
//            vel_vector.angular.z = states[j].angular_velocity_W[2];

            my_point.velocities.push_back(vel_vector);

            geometry_msgs::Twist acc_vector;

            acc_vector.linear.x = states[j].acceleration_W[0];
            acc_vector.linear.y = states[j].acceleration_W[1];
            acc_vector.linear.z = states[j].acceleration_W[2];

//            acc_vector.angular.x = states[j].angular_acceleration_W[0];
//            acc_vector.angular.y = states[j].angular_acceleration_W[1];
//            acc_vector.angular.z = states[j].angular_acceleration_W[2];

            my_point.accelerations.push_back(acc_vector);

            my_traj.points.push_back(my_point);

            my_traj.header.stamp = ros::Time::now();
            my_traj.points[j].time_from_start = ros::Duration(j*delta_t);

        }

    }
    else
    {
        std::cout << "trajectory generate unsuccessfully !!"<< std::endl;
    }

    visualization_msgs::MarkerArray markers;
    double distance = 0.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";
    // From Trajectory class:

    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    tra_visual_.publish(markers);
    markers.markers.clear();

}


void TR_PLANNER_SDF::plot_candidate_tra(std::vector<std::vector<Node *> > traj_)
{
    visualization_msgs::MarkerArray marker_arrays;


    for(int i =0; i < traj_.size(); i++)
    {

        geometry_msgs::Point point;
        visualization_msgs::Marker marker_line;
        marker_line.header.stamp = ros::Time();
        marker_line.header.frame_id = "world";
        marker_line.ns = "Candidate_tra";
        marker_line.id = i;
        marker_line.action = visualization_msgs::Marker::ADD;
        marker_line.type = visualization_msgs::Marker::LINE_STRIP;
        marker_line.pose.orientation.w = 1;
        marker_line.scale.x = 0.05;

        marker_line.color.a = 1.0; // Don't forget to set the alpha!
        marker_line.color.r = 1;
        marker_line.color.g = 1;
        marker_line.color.b = 0;
        for(int j = 0; j< traj_[i].size(); j++)
        {
            point.x = traj_[i][j]->position.x();
            point.y = traj_[i][j]->position.y();
            point.z = traj_[i][j]->position.z();


            marker_line.points.push_back(point);
        }

        marker_arrays.markers.push_back(marker_line);
    }
    candidate_traj_pub.publish(marker_arrays);
    marker_arrays.markers.clear();
}

void TR_PLANNER_SDF::plot_best_tra(std::vector<Node *> traj_)
{

    visualization_msgs::Marker marker_line;
    for(int i =0; i < traj_.size(); i++)
    {

        geometry_msgs::Point point;

        marker_line.header.stamp = ros::Time();
        marker_line.header.frame_id = "world";
        marker_line.ns = "Best_tra";
        marker_line.id = 100;
        marker_line.action = visualization_msgs::Marker::ADD;
        marker_line.type = visualization_msgs::Marker::LINE_STRIP;
        marker_line.pose.orientation.w = 1;
        marker_line.scale.x = 0.05;

        marker_line.color.a = 1.0; // Don't forget to set the alpha!
        marker_line.color.r = 1;
        marker_line.color.g = 0;
        marker_line.color.b = 0;

        point.x = traj_[i]->position.x();
        point.y = traj_[i]->position.y();
        point.z = traj_[i]->position.z();

        marker_line.points.push_back(point);

    }
    best_traj_pub.publish(marker_line);

}


//void TR_PLANNER_SDF::plot_uav(Vector3d current_position)
//{
//    visualization_msgs::Marker uav_axis;
//    uav_axis.header.stamp = ros::Time();
//    uav_axis.header.frame_id = "world";
//    uav_axis.ns = "uav_pos";
//    uav_axis.id = 50;
//    uav_axis.action = visualization_msgs::Marker::ADD;
//    uav_axis.type = visualization_msgs::Marker::ARROW;
//    uav_axis.scale.x = 0.5;
//    uav_axis.scale.y = 0.1;
//    uav_axis.scale.z = 0.1;
//    uav_axis.color.a = 1.0; // Don't forget to set the alpha!
//    uav_axis.color.r = 1.0;
//    uav_axis.color.g = 0.0;
//    uav_axis.color.b = 0.0;

//    uav_axis.pose.orientation.x = 0;
//    uav_axis.pose.orientation.y = 0;
//    uav_axis.pose.orientation.z = 0;
//    uav_axis.pose.orientation.w = 1;

//    uav_axis.pose.position.x = current_position.x();
//    uav_axis.pose.position.y = current_position.y();
//    uav_axis.pose.position.z = current_position.z();

//    uav_pos_marker_pub.publish(uav_axis);
//}
