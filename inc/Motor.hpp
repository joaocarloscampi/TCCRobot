#pragma once
#include "pico/stdlib.h"
#include "PWM.hpp"
#include "inc/Encoder.hpp"

class Motor {
public:
    Motor(uint in1, uint in2, uint en, uint8_t encoder_cs_pin, spi_inst_t* spi);

    void init();
    void setDuty(float duty);
    void forward();
    void backward();
    void stop();
    void free();

    uint32_t get_encoder_pulses();
    void reset_encoder_pulses();

    PWM pwm_;

private:
    uint8_t Encoder_CS_Pin_;
    uint in1_, in2_;
    float duty_;
    

    Encoder encoder_;
    spi_inst_t* spi_;
};
