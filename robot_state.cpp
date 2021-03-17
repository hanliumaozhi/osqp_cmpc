//
// Created by han on 2021/3/16.
//

#include "robot_state.h"

namespace CMPC{
    void RobotState::set(float *p_, float *v_, float *q_, float *w_, float *r_, float yaw_) {
        for(int i = 0; i < 3; i++)
        {
            this->p(i) = p_[i];
            this->v(i) = v_[i];
            this->w(i) = w_[i];
        }
        this->q.w() = q_[0];
        this->q.x() = q_[1];
        this->q.y() = q_[2];
        this->q.z() = q_[3];
        this->yaw = yaw_;

        //for(u8 i = 0; i < 12; i++)
        //    this->r_feet(i) = r[i];
        for(int rs = 0; rs < 3; rs++)
            for(int c = 0; c < 4; c++)
                this->r_feet(rs,c) = r_[rs*4 + c];

        R = this->q.toRotationMatrix();
        float yc = cos(yaw_);
        float ys = sin(yaw_);

        R_yaw <<  yc,  -ys,   0,
                ys,  yc,   0,
                0,   0,   1;

        Matrix<float, 3, 1> Id;
        Id << 0.04264025f, 0.2571403f, 0.2800529f;
        I_body.diagonal() = Id;
    }
}