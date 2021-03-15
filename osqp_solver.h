//
// Created by han on 2021/3/15.
//

#ifndef OSQP_CMPC_OSQP_SOLVER_H
#define OSQP_CMPC_OSQP_SOLVER_H

#include <eigen3/Eigen/Dense>

namespace CMPC {
    using Eigen::Dynamic;
    using Eigen::Matrix;
    class OSQPSolver {
    public:
        Matrix<double, Dynamic, 13> A_qp;
        Matrix<double, Dynamic, Dynamic> B_qp;
        Matrix<double, 13, 12> Bdt;
        Matrix<double, 13, 13> Adt;
        Matrix<double, 25, 25> ABc, expmm;
        Matrix<double, Dynamic, Dynamic> S;
        Matrix<double, Dynamic, 1> X_d;
        Matrix<double, Dynamic, 1> U_b;
        Matrix<double, Dynamic, Dynamic> fmat;
    };
}


#endif //OSQP_CMPC_OSQP_SOLVER_H
