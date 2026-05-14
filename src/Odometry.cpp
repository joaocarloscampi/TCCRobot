#include "inc/Odometry.hpp"

Odometry::Odometry()
{
    v_M1_=0;
    v_M2_=0;
    v_M3_=0;
    v_M4_=0;

    vL_ = 0;
    vR_ = 0;

    vx_local_ = 0;
    w_local_ = 0;

    x_local_ = 0;
    y_local_ = 0;
    theta_local_ = 0;

    x_global_ = 0;
    y_global_ = 0;
    theta_global_ = 0;
}

void Odometry::set_velocities(float v_M1, float v_M2, float v_M3, float v_M4)
{
    v_M1_ = v_M1;
    v_M2_ = v_M2;
    v_M3_ = v_M3;
    v_M4_ = v_M4;
}


void Odometry::update(void)
{   
    vL_ = (v_M3_ + v_M4_)*0.06f/2.0f;
    vR_ = (v_M1_ + v_M2_)*0.06f/2.0f;

    vx_local_ = (vR_+vL_)/2;
    w_local_ =  (vR_-vL_)/(2*0.1);

    x_local_ = x_local_ + vx_local_*10.0f/1000;
    theta_local_ = theta_local_ + w_local_*10.0f/1000;

    theta_local_ = atan2(sin(theta_local_), cos(theta_local_));

    float dx = vx_local_*10.0f/1000 * cos(theta_local_);
    float dy = vx_local_*10.0f/1000 * sin(theta_local_);

    x_global_ += dx;
    y_global_ += dy;
    theta_global_ = theta_local_;
}

void Odometry::set_v_M1(float v_M1) { v_M1_ = v_M1; }
void Odometry::set_v_M2(float v_M2) { v_M2_ = v_M2; }
void Odometry::set_v_M3(float v_M3) { v_M3_ = v_M3; }
void Odometry::set_v_M4(float v_M4) { v_M4_ = v_M4; }

float Odometry::get_v_M1() { return v_M1_; }
float Odometry::get_v_M2() { return v_M2_; }
float Odometry::get_v_M3() { return v_M3_; }
float Odometry::get_v_M4() { return v_M4_; }
float Odometry::get_x_local() { return x_local_; }
float Odometry::get_y_local() { return y_local_; }
float Odometry::get_theta_local() { return theta_local_; }
float Odometry::get_x_global() { return x_global_; }
float Odometry::get_y_global() { return y_global_; }
float Odometry::get_theta_global() { return theta_global_; }