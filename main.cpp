#include "osqp_solver.h"
#include <chrono>

int main() {
    CMPC::OSQPSolver osqp_solver;

    osqp_solver.set_problem(0.026, 10, 0.4, 120);

    double p[3] = {0, 0, 0.3};
    double v[3] = {0, 0, 0};
    double q[4] = {1, 0, 0, 0};
    double w[3] = {0, 0, 0};
    double r[12] = {0.183, 0.183, -0.183, -0.183, -0.14, 0.14, -0.14, 0.14, -0.3, -0.3, -0.3, -0.3};
    double yaw = 0;

    double trajAll[120];

    double trajInitial[12] = {0,  // 0
                             0,    // 1
                             0,    // 2
            //yawStart,    // 2
                             0,                                   // 3
                             0,                                   // 4
                             0.3,      // 5
                             0,                                        // 6
                             0,                                        // 7
                             0,  // 8
                             0,                           // 9
                             0,                           // 10
                             0};

    for(int i = 0; i < 10; i++)
    {
        for(int j = 0; j < 12; j++)
            trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this
        {
            trajAll[3] = 0;
            trajAll[4] = 0;
            trajAll[2] = 0;
        }
        else
        {
            trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3];
            trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4];
            trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2];
        }
    }

    double Q[12] = {0.25, 2, 10, 50, 50, 100, 0, 0, 0.3, 0.2, 0.2, 0.1};
    double alpha = 4e-5;

    int mpc_table[40];
    for (int i = 0; i < 10; ++i) {
        if((i*0.026) < (0.13)){
            mpc_table[0+i*4] = 0;
            mpc_table[1+i*4] = 1;
            mpc_table[2+i*4] = 1;
            mpc_table[3+i*4] = 0;
        }else{
            mpc_table[0+i*4] = 1;
            mpc_table[1+i*4] = 0;
            mpc_table[2+i*4] = 0;
            mpc_table[3+i*4] = 1;
        }
    }

    auto t0 = std::chrono::high_resolution_clock::now();
    osqp_solver.update_problem_data(p, v, q, w, r, yaw, Q, trajAll, alpha, mpc_table);
    osqp_solver.solve_mpc();
    auto t1 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<float> fs = t1 - t0;
    std::chrono::microseconds d = std::chrono::duration_cast<std::chrono::microseconds>(fs);
    std::cout << d.count() << std::endl;
    return 0;
}
