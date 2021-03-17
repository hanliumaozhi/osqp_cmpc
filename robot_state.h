//
// Created by han on 2021/3/16.
//

#ifndef OSQP_CMPC_ROBOT_STATE_H
#define OSQP_CMPC_ROBOT_STATE_H

#include <eigen3/Eigen/Dense>

using Eigen::Matrix;
using Eigen::Quaternionf;

namespace CMPC {
    struct RobotState {
        void set(float* p, float* v, float* q, float* w, float* r, float yaw);
        //void compute_rotations();
        //void print();
        Matrix<float,3,1> p,v,w;
        Matrix<float,3,4> r_feet;
        Matrix<float,3,3> R;
        Matrix<float,3,3> R_yaw;
        Matrix<float,3,3> I_body;
        Quaternionf q;
        float yaw;
        float m = 12.454;

    };
}


#endif //OSQP_CMPC_ROBOT_STATE_H
