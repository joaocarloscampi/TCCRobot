#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <stdio.h>
#include "pico/stdlib.h"

#define TS_CONTROL 10
#define R_WHEEL 0.06
#define C_ROBOT 0.1

class Odometry{
private:
    float v_M1_;
    float v_M2_;
    float v_M3_;
    float v_M4_;

    float vL_;
    float vR_;

    float vx_local_;
    float w_local_;

    float x_local_;
    float y_local_;
    float theta_local_;

    float x_global_;
    float y_global_;
    float theta_global_;
public:
    Odometry(/* args */);

    void set_v_M1(float v_M1);
    void set_v_M2(float v_M2);
    void set_v_M3(float v_M3);
    void set_v_M4(float v_M4);
    void set_velocities(float v_M1, float v_M2, float v_M3, float v_M4);

    float get_v_M1();
    float get_v_M2();
    float get_v_M3();
    float get_v_M4();
    float get_x_local();
    float get_y_local();
    float get_theta_local();

    float get_x_global();
    float get_y_global();
    float get_theta_global();
    
    void update(void);
};

#endif


