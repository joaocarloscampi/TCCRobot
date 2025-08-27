#pragma once
#include "pico/stdlib.h"
#include "hardware/pwm.h"

class Motor {
public:
    // Construtor: IN1, IN2, EN (PWM)
    Motor(uint in1, uint in2, uint en);

    // Inicializa pinos e PWM
    void init(uint32_t freq = 1000);

    // Controle de movimento
    void forward(float duty);
    void backward(float duty);
    void stop();

private:
    uint in1_, in2_, en_;
    uint slice_;
    uint32_t top_;
};
