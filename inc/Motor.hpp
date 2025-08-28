#pragma once
#include "pico/stdlib.h"
#include "PWM.hpp"

class Motor {
public:
    Motor(uint in1, uint in2, uint en);

    void init();
    void setDuty(float duty);
    void forward();
    void backward();
    void stop();
    void free();

private:
    uint in1_, in2_;
    float duty_;
    PWM pwm_;
};
