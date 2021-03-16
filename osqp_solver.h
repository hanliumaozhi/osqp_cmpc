//
// Created by han on 2021/3/15.
//

#ifndef OSQP_CMPC_OSQP_SOLVER_H
#define OSQP_CMPC_OSQP_SOLVER_H

#include <eigen3/Eigen/Dense>
#include <memory>

#include "robot_state.h"

namespace CMPC {

    struct update_data_t {
        double p[3];
        double v[3];
        double q[4];
        double w[3];
        double r[12];
        double yaw;
        double weights[12];
        double traj[12 * 20];
        double alpha;
        int gait[20];
        //unsigned char hack_pad[1000];
        int max_iterations;
        double rho, sigma, solver_alpha, terminate;
        int use_jcqp;
        double x_drag;
    };

    using Eigen::Dynamic;
    using Eigen::Matrix;

    class OSQPSolver {
    public:
        OSQPSolver();

        Matrix<double, Dynamic, 13> A_qp;
        Matrix<double, Dynamic, Dynamic> B_qp;
        Matrix<double, 13, 12> Bdt;
        Matrix<double, 13, 13> Adt;
        Matrix<double, 25, 25> ABc, expmm;
        Matrix<double, Dynamic, Dynamic> S;
        Matrix<double, Dynamic, 1> X_d;
        Matrix<double, Dynamic, 1> U_b;
        Matrix<double, Dynamic, Dynamic> fmat;
        Matrix<double, Dynamic, Dynamic> qH;
        Matrix<double, Dynamic, 1> qg;

        Matrix<double, Dynamic, Dynamic> eye_12h;

        std::shared_ptr<update_data_t> update_;
        std::shared_ptr<RobotState> rs_;

        Matrix<double, 13, 1> x_0;
        Matrix<double, 3, 3> I_world;
        Matrix<double, 13, 13> A_ct;
        Matrix<double, 13, 12> B_ct_r;

        uint8_t var_elim[2000];
        uint8_t con_elim[2000];


        double dt_;
        int horizon_;
        double mu_;
        double f_max_;

        void set_problem(double dt, int horizon, double mu, double f_max);

        void update_problem_data(double *p, double *v, double *q, double *w, double *r, double yaw, double *weights,
                                 double *state_trajectory, double alpha, int *gait);

        void solve_mpc();

        void quat_to_rpy(Quaternionf q, Matrix<double, 3, 1> &rpy);

        void ct_ss_mats(Matrix<double, 3, 3> I_world, double m, Matrix<double, 3, 4> r_feet, Matrix<double, 3, 3> R_yaw,
                        Matrix<double, 13, 13> &A, Matrix<double, 13, 12> &B, double x_drag);

        Matrix<double, 3, 3> cross_mat(Matrix<double, 3, 3> I_inv, Matrix<double, 3, 1> r);

        void c2qp(Matrix<double, 13, 13> Ac, Matrix<double, 13, 12> Bc, double dt, int horizon);

        double sq(double x) { return x * x; }


    };
}


#endif //OSQP_CMPC_OSQP_SOLVER_H
