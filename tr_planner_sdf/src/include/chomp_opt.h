#ifndef CHOMP_OPT_HPP
#define CHOMP_OPT_HPP

//egien
#include <Eigen/Dense>
#include <Eigen/Eigen>
//std
#include <iostream>
#include <vector>
#include "std_msgs/String.h"
//ROS_message
#include <nav_msgs/Path.h>
//opencv
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include "opencv2/core/core.hpp"

//param chomp

static size_t const cdim(3);            // dimension of config space
static size_t const iteration_limit(400);
static double const dt(1);            // time step
static double const eta(40.0);         // >= 1, regularization factor for gradient descent
static double const lambda(2.0);        // weight of smoothness objective
static double const ebso(0.65);          //allowable threshold for obstacle function

typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::Isometry3d Transform;


class obs_chomp_{
    private:

    public:
    cv::Point3f obs_center;
    float obs_radius;

};


class CHOMP_OPT{

public:

    CHOMP_OPT();
    ~CHOMP_OPT();

    void init_chomp(Vector const &qs, Vector const &qe, Vector  &xi);

    double chomp_iteration(Vector const &qs, Vector const &qe, Vector  &xi,
                           const Vector gradient, const Vector distance);

    double chomp_goal_bias(Vector const &qs, Vector const &qe, Vector  &xi, cv::Point3d &goal,
                           const Vector gradient, const Vector distance);

//    void generatePath(Vector const &qs, Vector const &qe, Vector &xi , const Vector gradient, const Vector distance);

    void generateTrajectory(const Vector &qs, const Vector &qe, const Vector &xi, nav_msgs::Path &chomp_path);

private:

    //////////////////////////////////////////////////
    // gradient descent etc

    Eigen::MatrixXd AA;                      // metric
    Vector bb;                      // acceleration bias for start and end config
    Eigen::MatrixXd Ainv;                    // inverse of AA


};

#endif
