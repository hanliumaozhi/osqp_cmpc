//
// Created by han on 2021/3/15.
//

#ifndef OSQP_CMPC_OSQP_SOLVER_H
#define OSQP_CMPC_OSQP_SOLVER_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <memory>
#include "iosqp.hpp"

#include "robot_state.h"

namespace CMPC {

    struct update_data_t {
        float p[3];
        float v[3];
        float q[4];
        float w[3];
        float r[12];
        float yaw;
        float weights[12];
        float traj[12 * 20];
        float alpha;
        int gait[20];
        //unsigned char hack_pad[1000];
        int max_iterations;
        float rho, sigma, solver_alpha, terminate;
        int use_jcqp;
        float x_drag;
    };

    using Eigen::Dynamic;
    using Eigen::Matrix;

    class OSQPSolver {
    public:
        OSQPSolver();
        ~OSQPSolver() = default;

        Matrix<float, Dynamic, 13> A_qp;
        Matrix<float, Dynamic, Dynamic> B_qp;
        Matrix<float, 13, 12> Bdt;
        Matrix<float, 13, 13> Adt;
        Matrix<float, 25, 25> ABc, expmm;
        Matrix<float, Dynamic, Dynamic> S;
        Matrix<float, Dynamic, 1> X_d;
        Matrix<float, Dynamic, 1> U_b;
        Matrix<float, Dynamic, 1> L_b;
        Matrix<float, Dynamic, Dynamic> fmat;
        Matrix<float, Dynamic, Dynamic> qH;
        Matrix<float, Dynamic, 1> qg;

        Matrix<float, Dynamic, Dynamic> osqp_P;
        Matrix<float, Dynamic, 1> osqp_q;
        Matrix<float, Dynamic, 1> U_b_red;
        Matrix<float, Dynamic, 1> L_b_red;
        Matrix<float, Dynamic, Dynamic> A_red;

        Matrix<float, Dynamic, Dynamic> eye_12h;

        update_data_t update_;
        //std::shared_ptr<RobotState> rs_;
        RobotState rs_;

        Matrix<float, 13, 1> x_0;
        Matrix<float, 3, 3> I_world;
        Matrix<float, 13, 13> A_ct;
        Matrix<float, 13, 12> B_ct_r;

        Matrix<float, 12, 1> q_soln;

        uint8_t var_elim[2000];
        uint8_t con_elim[2000];


        float dt_;
        int horizon_;
        float mu_;
        float f_max_;

        void set_problem(float dt, int horizon, float mu, float f_max);

        void update_problem_data(float *p, float *v, float *q, float *w, float *r, float yaw, float *weights,
                                 float *state_trajectory, float alpha, int *gait);

        void solve_mpc();

        void update_x_drag(float x_drag) {
            update_.x_drag = x_drag;
        }

        void quat_to_rpy(Quaternionf q, Matrix<float, 3, 1> &rpy);

        void ct_ss_mats(Matrix<float, 3, 3> I_world, float m, Matrix<float, 3, 4> r_feet, Matrix<float, 3, 3> R_yaw,
                        Matrix<float, 13, 13> &A, Matrix<float, 13, 12> &B, float x_drag);

        Matrix<float, 3, 3> cross_mat(Matrix<float, 3, 3> I_inv, Matrix<float, 3, 1> r);

        void c2qp(Matrix<float, 13, 13> Ac, Matrix<float, 13, 12> Bc, float dt, int horizon);

        static float sq(float x) { return x * x; }

        static bool near_zero(float a) {return (a<0.01 && a >-0.01);}

        static bool near_one(float a){ return near_zero(a-1);}

        //std::shared_ptr<IOSQP> osqp_ptr;

    };
}


#endif //OSQP_CMPC_OSQP_SOLVER_H
