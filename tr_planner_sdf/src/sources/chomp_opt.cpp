#include "chomp_opt.h"

CHOMP_OPT::CHOMP_OPT()
{}

CHOMP_OPT::~CHOMP_OPT()
{}


void CHOMP_OPT::init_chomp(const Vector &qs, const Vector &qe, Vector &xi)
{

    AA = Matrix::Zero(xi.rows(), xi.rows());
    int nq = xi.rows()/cdim;
    for (size_t ii(0); ii < nq; ++ii) {
        AA.block(cdim * ii, cdim * ii, cdim, cdim) = 2.0 * Matrix::Identity(cdim, cdim);
        if (ii > 0) {
            AA.block(cdim * (ii - 1), cdim * ii, cdim, cdim) = -1.0 * Matrix::Identity(cdim, cdim);
            AA.block(cdim * ii, cdim * (ii - 1), cdim, cdim) = -1.0 * Matrix::Identity(cdim, cdim);
        }
    }
    AA /= dt * dt * (nq + 1);

    bb = Vector::Zero(xi.rows());
    bb.block(0, 0, cdim, 1) = qs;
    bb.block(xi.rows() - cdim, 0, cdim, 1) = qe;
    bb /= -dt * dt * (nq + 1);

    // not needed anyhow
    // double cc (double (qs.transpose() * qs) + double (qe.transpose() * qe));
    // cc /= dt * dt * (nq + 1);

    Ainv = AA.inverse();

}

double CHOMP_OPT::chomp_iteration(const Vector &qs, const Vector &qe, Vector &xi,
                                  const Vector gradient, const Vector distance)
{
    //////////////////////////////////////////////////
    // beginning of "the" CHOMP iteration
    Vector nabla_smooth(AA * xi + bb);
    Vector const & xidd(nabla_smooth); // indeed, it is the same in this formulation...

    Vector nabla_obs(Vector::Zero(xi.rows()));

    int nq = xi.rows()/cdim;

    for (size_t iq(0); iq < nq; ++iq) {
        Vector const qq(xi.block(iq * cdim, 0, cdim, 1));
        Vector qd;
        if (0 == iq) {
            qd = 0.5 * (xi.block((iq + 1) * cdim, 0, cdim, 1) - qs);
        } else if (iq == nq - 1) {
            qd = 0.5 * (qe - xi.block((iq - 1) * cdim, 0, cdim, 1));
        } else {
            qd = 0.5 * (xi.block((iq + 1) * cdim, 0, cdim, 1) - xi.block((iq - 1) * cdim, 0, cdim, 1));
        }

        // In this case, C and W are the same, Jacobian is identity.  We
        // still write more or less the full-fledged CHOMP expressions
        // (but  we only use one body point) to make subsequent extension
        // easier.
        //

        Vector const & xx(qq);

        Vector const & xd(qd);

        Matrix const JJ(Matrix::Identity(3, 3)); // a little silly here, as noted above.

        double const vel(xd.norm());

        if (vel < 1.0e-3)                               // avoid div by zero further down
            continue;
        Vector const xdn(xd / vel);

        Vector const xdd(JJ * xidd.block(iq * cdim, 0, cdim, 1));

        Matrix const prj(Matrix::Identity(3, 3) - xdn * xdn.transpose()); // hardcoded planar case
        Vector const kappa(prj * xdd / pow(vel, 2.0));

       static double const z_gain(1);                                                // hardcoded param
         if(xx(2) <= 0.2)
        {
           Eigen::VectorXd delta_z(3);
           delta_z << 0,0,xx(2);
           double const z_cost(pow(xx(2)-0.5, 2.0));
           delta_z *= z_gain*2*(xx(2)-0.5)/xx(2);// hardcoded param cost function for z_limit 0.5

           static double const gain(1);                                                // hardcoded param
           double cost(0);
           Eigen::VectorXd delta(3);
           delta(0) = gradient(cdim*iq);
           delta(1) = gradient(cdim*iq+1);
           delta(2) = gradient(cdim*iq+2);
           Eigen::VectorXd delta_obs(3);
           if(distance(iq) < 0)
           {
               cost= -gain*distance(iq) + gain*ebso/2;
               delta_obs = -gain*delta;
           }
           else if(distance(iq) > 1e-9 && distance(iq) < ebso)
           {
               cost= gain*pow((distance(iq)-ebso), 2)/(2*ebso);

               delta_obs = gain*delta*(distance(iq) -ebso)/ebso;

           }
           else
           {
               cost = 0;
               delta_obs(0) = 0;
               delta_obs(1) = 0;
               delta_obs(2) = 0;
           }

           nabla_obs.block(iq * cdim, 0, cdim, 1) += JJ.transpose() *
                   vel * (prj * delta_obs + prj * delta_z - cost * kappa - z_cost * kappa);

         }
         else
          {
            double const z_cost(0);
            Eigen::VectorXd delta_z(3);
            delta_z << 0,0,0;

            static double const gain(1);                                                // hardcoded param

            double cost(0);
            Eigen::VectorXd delta(3);
            delta(0) = gradient(cdim*iq);
            delta(1) = gradient(cdim*iq+1);
            delta(2) = gradient(cdim*iq+2);
            Eigen::VectorXd delta_obs(3);
            if(distance(iq) < 0 )
            {
                cost= -gain*distance(iq) + gain*ebso/2;
                delta_obs = -gain*delta;
            }
            else if(distance(iq) > 1e-9 && distance(iq) < ebso)
            {
                cost= gain*pow((distance(iq)-ebso), 2)/(2*ebso);

                delta_obs = gain*delta*(distance(iq) - ebso)/ebso;

            }
            else
            {
                cost = 0;
                delta_obs(0) = 0;
                delta_obs(1) = 0;
                delta_obs(2) = 0;
            }

            nabla_obs.block(iq * cdim, 0, cdim, 1) += JJ.transpose() *
                    vel * (prj * delta_obs + prj * delta_z - cost * kappa - z_cost * kappa);

//            std::cout << "delta_obs == " << delta_obs << std::endl;
//            std::cout << "dist == " << distance(iq) << std::endl;
            }


            }

//            std::cout << "nabla_smooth == "<< nabla_smooth << std::endl;
//            std::cout << "nabla_obs == "<< nabla_obs << std::endl;
            Vector dxi(Ainv * (nabla_obs + lambda * nabla_smooth));
//            std::cout << "dxi ==" << dxi << std::endl;
            //Vector dxi(Ainv * (lambda * nabla_smooth));
            xi -= dxi / eta;
            //return the error (in Euclidean sense ). Remeber that the difference is -dxi/eta
            return dxi.norm() / eta;

}

double CHOMP_OPT::chomp_goal_bias(const Vector &qs, const Vector &qe, Vector &xi, cv::Point3d &goal,
                                  const Vector gradient, const Vector distance)
{
    //////////////////////////////////////////////////

    // beginning of "the" CHOMP iteration

   // Vector const & xidd(nabla_smooth); // indeed, it is the same in this formulation...


    double const ebso_g(0.45);
    static double const goal_gain(0.5);                                                // hardcoded param
    //static double const goal_size(1);                                                // hardcoded param

    double delta_x = goal.x - qe(0);
    double delta_y = goal.y - qe(1);
    double delta_z = goal.z - qe(2);

    double dist_goal = sqrt(pow(delta_x,2)+pow(delta_y,2)+pow(delta_z,2));
    double const cost_goal(goal_gain*dist_goal);

//    std::cout<<"cost_goal =="<<cost_goal<<std::endl;
//    std::cout<<"goal_pos =="<<qe(0)<<"//"<<qe(1)<<"//"<<qe(2)<<std::endl;
    int nq = xi.rows()/cdim;
       double cost(0);
    for (size_t iq(0); iq < nq; ++iq) {
        Vector const qq(xi.block(iq * cdim, 0, cdim, 1));
        Vector qd;

        if (0 == iq) {
            qd = 0.5 * (xi.block((iq + 1) * cdim, 0, cdim, 1) - qs);
        } else if (iq == nq - 1) {
            qd = 0.5 * (qe - xi.block((iq - 1) * cdim, 0, cdim, 1));
        } else {
            qd = 0.5 * (xi.block((iq + 1) * cdim, 0, cdim, 1) - xi.block((iq - 1) * cdim, 0, cdim, 1));
        }

        // In this case, C and W are the same, Jacobian is identity.  We
        // still write more or less the full-fledged CHOMP expressions
        // (but  we only use one body point) to make subsequent extension
        // easier.
        //

        Vector const & xx(qq);

        Vector const & xd(qd);

        double const vel(xd.norm());

        if (vel < 1.0e-3)                               // avoid div by zero further down
            continue;


         if(xx(2) <= 0.1)
        {

           double const z_cost(pow(xx(2)-0.5, 2.0));

           static double const gain(30);                                                // hardcoded param
           //double cost(0);

           if(distance(iq) < 0)
           {
               cost= -gain*distance(iq) + gain*ebso_g/2;

           }
           else if(distance(iq) > 1e-9 && distance(iq) < ebso_g)
           {
               cost= gain*pow((distance(iq)-ebso_g), 2)/(2*ebso_g);


           }
           else
           {
               cost = 0;

           }
                cost = cost + z_cost;
         }
         else
          {
            double const z_cost(0);


            static double const gain(15);                                                // hardcoded param
            //double cost(0);
            if(distance(iq) < 0)
            {
                cost= -gain*distance(iq) + gain*ebso_g/2;
            }
            else if(distance(iq) > 1e-9 && distance(iq) < ebso_g)
            {
                cost= gain*pow((distance(iq)-ebso_g), 2)/(2*ebso_g);

            }
            else
            {
                cost = 0;
            }
            cost = cost + z_cost;
            }

            }

            //Vector dxi(Ainv * (nabla_obs + lambda * nabla_smooth));

            return (cost + cost_goal);

//    return cost_goal;

}


//void CHOMP_OPT::generatePath(const Vector &qs, const Vector &qe, Vector &xi,
//                             const Vector gradient, const Vector distance)
//{
//    double err_chomp = 1;

//    init_chomp(qs, qe, xi);
//    //cout<<"is it ok?"<<endl;
//    for (size_t ii = 0; ii < iteration_limit; ii++) {

//        err_chomp = chomp_iteration(qs, qe, xi, gradient, distance);
//        if (err_chomp < 0.00001)
//            break;
//    }
//}

void CHOMP_OPT::generateTrajectory(const Vector &qs, const Vector &qe, const Vector &xi, nav_msgs::Path &chomp_path)
{

        geometry_msgs::PoseStamped points;
        Vector xi_copy(xi.size()+qs.size()+qe.size());
        chomp_path.header.frame_id = "world";
        xi_copy<<qs,xi,qe; //copy xi and add starting point and end point
        for (size_t ii = 0; ii < xi_copy.size()/3; ii++){
            points.header.frame_id = "world";
            points.pose.position.x = xi_copy[xi_copy.size() - ii*3 - 3];
            points.pose.position.y = xi_copy[xi_copy.size() - ii*3 - 2];
            points.pose.position.z = xi_copy[xi_copy.size() - ii*3 - 1];
//            std::cout << "pos_x =="<< points.pose.position.x << std::endl;
//            std::cout << "pos_y =="<< points.pose.position.y << std::endl;
//            std::cout << "pos_z =="<< points.pose.position.z << std::endl;
            chomp_path.poses.push_back(points);
        }


}
