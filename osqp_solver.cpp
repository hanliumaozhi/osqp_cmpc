//
// Created by han on 2021/3/15.
//

#include "osqp_solver.h"

#include <eigen3/unsupported/Eigen/MatrixFunctions>

namespace CMPC {
    OSQPSolver::OSQPSolver() {
        update_ = std::make_shared<update_data_t>();
        rs_ = std::make_shared<RobotState>();
        //osqp_ptr = std::make_shared<IOSQP>();
    }

    void OSQPSolver::set_problem(double dt, int horizon, double mu, double f_max) {
        dt_ = dt;
        horizon_ = horizon;
        mu_ = mu;
        f_max_ = f_max;

        int mcount = 0;
        int h2 = horizon * horizon;

        A_qp.resize(13 * horizon, Eigen::NoChange);
        mcount += 13 * horizon * 1;

        B_qp.resize(13 * horizon, 12 * horizon);
        mcount += 13 * h2 * 12;

        S.resize(13 * horizon, 13 * horizon);
        mcount += 13 * 13 * h2;

        X_d.resize(13 * horizon, Eigen::NoChange);
        mcount += 13 * horizon;

        U_b.resize(20 * horizon, Eigen::NoChange);
        mcount += 20 * horizon;

        L_b.resize(20 * horizon, Eigen::NoChange);
        mcount += 20 * horizon;

        /*U_b_red.resize(20 * horizon, Eigen::NoChange);
        mcount += 20 * horizon;

        L_b_red.resize(20 * horizon, Eigen::NoChange);
        mcount += 20 * horizon;*/

        fmat.resize(20 * horizon, 12 * horizon);
        mcount += 20 * 12 * h2;

        //A_red.resize(20 * horizon, 12 * horizon);
        //mcount += 20 * 12 * h2;

        qH.resize(12 * horizon, 12 * horizon);
        mcount += 12 * 12 * h2;

        //osqp_P.resize(12 * horizon, 12 * horizon);
        //mcount += 12 * 12 * h2;

        qg.resize(12 * horizon, Eigen::NoChange);
        mcount += 12 * horizon;

        //osqp_q.resize(12 * horizon, Eigen::NoChange);
        //mcount += 12 * horizon;

        eye_12h.resize(12 * horizon, 12 * horizon);
        mcount += 12 * 12 * horizon;

        //printf("realloc'd %d floating point numbers.\n",mcount);
        mcount = 0;

        A_qp.setZero();
        B_qp.setZero();
        S.setZero();
        X_d.setZero();
        U_b.setZero();
        fmat.setZero();
        qH.setZero();
        eye_12h.setIdentity();
    }

    void
    OSQPSolver::update_problem_data(double *p, double *v, double *q, double *w, double *r, double yaw, double *weights,
                                    double *state_trajectory, double alpha, int *gait) {
        update_->alpha = alpha;
        update_->yaw = yaw;

        memcpy((void *) update_->gait, (void *) gait, sizeof(int) * 4 * horizon_);
        memcpy((void *) update_->p, (void *) p, sizeof(double) * 3);
        memcpy((void *) update_->v, (void *) v, sizeof(double) * 3);
        memcpy((void *) update_->q, (void *) q, sizeof(double) * 4);
        memcpy((void *) update_->w, (void *) w, sizeof(double) * 3);
        memcpy((void *) update_->r, (void *) r, sizeof(double) * 12);
        memcpy((void *) update_->weights, (void *) weights, sizeof(double) * 12);
        memcpy((void *) update_->traj, (void *) state_trajectory, sizeof(double) * 12 * horizon_);

    }

    void OSQPSolver::solve_mpc() {
        rs_->set(update_->p, update_->v, update_->q, update_->w, update_->r, update_->yaw);
        //roll pitch yaw
        Matrix<double, 3, 1> rpy;
        quat_to_rpy(rs_->q, rpy);

        x_0 << rpy(2), rpy(1), rpy(0), rs_->p, rs_->w, rs_->v, -9.8f;
        I_world = rs_->R_yaw * rs_->I_body * rs_->R_yaw.transpose();

        ct_ss_mats(I_world, rs_->m, rs_->r_feet, rs_->R_yaw, A_ct, B_ct_r, update_->x_drag);


        //QP matrices
        c2qp(A_ct, B_ct_r, dt_, horizon_);

        //weights
        Matrix<double, 13, 1> full_weight;
        for (int i = 0; i < 12; i++)
            full_weight(i) = update_->weights[i];
        full_weight(12) = 0.f;
        S.diagonal() = full_weight.replicate(horizon_, 1);

        //trajectory
        for (int i = 0; i < horizon_; i++) {
            for (int j = 0; j < 12; j++)
                X_d(13 * i + j, 0) = update_->traj[12 * i + j];
        }

        //note - I'm not doing the shifting here.
        int k = 0;
        for (int i = 0; i < horizon_; i++) {
            for (int j = 0; j < 4; j++) {
                U_b(5 * k + 0) = 5e10;
                U_b(5 * k + 1) = 5e10;
                U_b(5 * k + 2) = 5e10;
                U_b(5 * k + 3) = 5e10;
                U_b(5 * k + 4) = update_->gait[i * 4 + j] * f_max_;
                k++;
            }
        }

        double mu = 1.f / mu_;
        Matrix<double, 5, 3> f_block;

        f_block << mu, 0, 1.f,
                -mu, 0, 1.f,
                0, mu, 1.f,
                0, -mu, 1.f,
                0, 0, 1.f;

        for (int i = 0; i < horizon_ * 4; i++) {
            fmat.block(i * 5, i * 3, 5, 3) = f_block;
        }

        qH = 2 * (B_qp.transpose() * S * B_qp + update_->alpha * eye_12h);
        qg = 2 * B_qp.transpose() * S * (A_qp * x_0 - X_d);

        for (int i = 0; i < horizon_ * 20; i++) {
            L_b(i) = 0.0;
        }

        //reduce

        int num_constraints = 20 * horizon_;
        int num_variables = 12 * horizon_;

        int new_vars = num_variables;
        int new_cons = num_constraints;

        for (int i = 0; i < num_constraints; i++)
            con_elim[i] = 0;

        for (int i = 0; i < num_variables; i++)
            var_elim[i] = 0;

        for (int i = 0; i < num_constraints; i++) {
            if (not(near_zero(U_b(i)) && near_zero(L_b(i)))) continue;

            for (int j = 0; j < num_variables; ++j) {
                if (near_one(fmat(i, j))) {
                    new_vars -= 3;
                    new_cons -= 5;
                    int cs = (j * 5) / 3 - 3;
                    var_elim[j - 2] = 1;
                    var_elim[j - 1] = 1;
                    var_elim[j] = 1;
                    con_elim[cs] = 1;
                    con_elim[cs + 1] = 1;
                    con_elim[cs + 2] = 1;
                    con_elim[cs + 3] = 1;
                    con_elim[cs + 4] = 1;
                }
            }

        }

        int var_ind[new_vars];
        int con_ind[new_cons];
        int vc = 0;
        for (int i = 0; i < num_variables; i++) {
            if (!var_elim[i]) {
                if (vc >= new_vars) {
                    printf("BAD ERROR 1\n");
                }
                var_ind[vc] = i;
                vc++;
            }
        }
        vc = 0;
        for (int i = 0; i < num_constraints; i++) {
            if (!con_elim[i]) {
                if (vc >= new_cons) {
                    printf("BAD ERROR 1\n");
                }
                con_ind[vc] = i;
                vc++;
            }
        }

        osqp_P.resize(new_vars, new_vars);
        osqp_q.resize(new_vars, Eigen::NoChange);

        //reduce
        for (int i = 0; i < new_vars; i++) {
            int olda = var_ind[i];
            osqp_q[i] = qg[olda];
            for (int j = 0; j < new_vars; j++) {
                int oldb = var_ind[j];
                osqp_P(i, j) = qH(olda, oldb);
            }
        }


        A_red.resize(new_vars / 3 * 5, new_vars);


        for (int con = 0; con < new_cons; con++) {
            for (int st = 0; st < new_vars; st++) {
                float cval = fmat(con_ind[con], var_ind[st]);
                A_red(con, st) = cval;
            }
        }

        U_b_red.resize(new_vars / 3 * 5, Eigen::NoChange);
        L_b_red.resize(new_vars / 3 * 5, Eigen::NoChange);

        for (int i = 0; i < new_cons; i++) {
            int old = con_ind[i];
            U_b_red[i] = U_b[old];
            L_b_red[i] = L_b[old];
        }

        IOSQP osqp;
        Eigen::SparseMatrix<double> p_sparse = osqp_P.sparseView();
        //auto q_sparse = osqp_q.sparseView();
        Eigen::SparseMatrix<double> a_sparse = A_red.sparseView();

        osqp.setMats(p_sparse, osqp_q, a_sparse, L_b_red, U_b_red);

        osqp.solve();
        //std::cout<<osqp.getStatus()<<std::endl;

        Eigen::VectorXd q_red = osqp.getPrimalSol();
        vc = 0;
        for (int i = 0; i < 12; i++) {
            if (var_elim[i]) {
                q_soln[i] = 0.0;
            } else {
                q_soln[i] = q_red[vc];
                vc++;
            }
        }


    }

    void OSQPSolver::quat_to_rpy(Quaterniond q, Matrix<double, 3, 1> &rpy) {
        //edge case!
        double as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
        rpy(0) = atan2(2.f * (q.x() * q.y() + q.w() * q.z()), sq(q.w()) + sq(q.x()) - sq(q.y()) - sq(q.z()));
        rpy(1) = asin(as);
        rpy(2) = atan2(2.f * (q.y() * q.z() + q.w() * q.x()), sq(q.w()) - sq(q.x()) - sq(q.y()) + sq(q.z()));

    }


    Matrix<double, 3, 3> OSQPSolver::cross_mat(Matrix<double, 3, 3> I_inv, Matrix<double, 3, 1> r) {
        Matrix<double, 3, 3> cm;
        cm << 0.f, -r(2), r(1),
                r(2), 0.f, -r(0),
                -r(1), r(0), 0.f;
        return I_inv * cm;
    }

    void OSQPSolver::ct_ss_mats(Matrix<double, 3, 3> I_world, double m, Matrix<double, 3, 4> r_feet,
                                Matrix<double, 3, 3> R_yaw, Matrix<double, 13, 13> &A, Matrix<double, 13, 12> &B,
                                double x_drag) {
        A.setZero();
        A(3, 9) = 1.f;
        A(11, 9) = x_drag;
        A(4, 10) = 1.f;
        A(5, 11) = 1.f;

        A(11, 12) = 1.f;
        A.block(0, 6, 3, 3) = R_yaw.transpose();

        B.setZero();
        Matrix<double, 3, 3> I_inv = I_world.inverse();

        for (int b = 0; b < 4; b++) {
            B.block(6, b * 3, 3, 3) = cross_mat(I_inv, r_feet.col(b));
            B.block(9, b * 3, 3, 3) = Matrix<double, 3, 3>::Identity() / m;
        }
    }

    void OSQPSolver::c2qp(Matrix<double, 13, 13> Ac, Matrix<double, 13, 12> Bc, double dt, int horizon) {
        ABc.setZero();
        ABc.block(0, 0, 13, 13) = Ac;
        ABc.block(0, 13, 13, 12) = Bc;
        ABc = dt * ABc;
        expmm = ABc.exp();
        Adt = expmm.block(0, 0, 13, 13);
        Bdt = expmm.block(0, 13, 13, 12);
        if (horizon > 19) {
            throw std::runtime_error("horizon is too long!");
        }

        Matrix<double, 13, 13> powerMats[20];
        powerMats[0].setIdentity();
        for (int i = 1; i < horizon + 1; i++) {
            powerMats[i] = Adt * powerMats[i - 1];
        }

        for (int r = 0; r < horizon; r++) {
            A_qp.block(13 * r, 0, 13, 13) = powerMats[r + 1];//Adt.pow(r+1);
            for (int c = 0; c < horizon; c++) {
                if (r >= c) {
                    int a_num = r - c;
                    B_qp.block(13 * r, 12 * c, 13, 12) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt;
                }
            }
        }

    }

}
