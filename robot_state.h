//
// Created by han on 2021/3/16.
//

#ifndef OSQP_CMPC_ROBOT_STATE_H
#define OSQP_CMPC_ROBOT_STATE_H

#include <eigen3/Eigen/Dense>

using Eigen::Matrix;
using Eigen::Quaterniond;

namespace CMPC {
    struct RobotState {
        void set(double* p, double* v, double* q, double* w, double* r, double yaw);
        //void compute_rotations();
        //void print();
        Matrix<double,3,1> p,v,w;
        Matrix<double,3,4> r_feet;
        Matrix<double,3,3> R;
        Matrix<double,3,3> R_yaw;
        Matrix<double,3,3> I_body;
        Quaterniond q;
        double yaw;
        double m = 12.454;

    };
}


#endif //OSQP_CMPC_ROBOT_STATE_H
