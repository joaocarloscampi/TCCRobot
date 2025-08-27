#include "Motor.hpp"

Motor::Motor(uint in1, uint in2, uint en)
{
    in1_ = in1; 
    in2_ = in2; 
    en_  = en;
};

void Motor::init(uint32_t freq) {
    // Configura GPIOs como saída
    gpio_init(in1_); gpio_set_dir(in1_, GPIO_OUT);
    gpio_init(in2_); gpio_set_dir(in2_, GPIO_OUT);

    // Configura pino enable como PWM
    gpio_set_function(en_, GPIO_FUNC_PWM);

    // Calcula parâmetros PWM
    uint32_t f_sys = 125000000; // clock RP2040
    float div = 1.0f;
    top_ = (f_sys / (freq * div)) - 1;

    slice_ = pwm_gpio_to_slice_num(en_);
    pwm_set_clkdiv(slice_, div);
    pwm_set_wrap(slice_, top_);
    pwm_set_gpio_level(en_, 0); // começa desligado
    pwm_set_enabled(slice_, true);
}

void Motor::forward(float duty) {
    gpio_put(in1_, 1);
    gpio_put(in2_, 0);
    uint32_t duty_top =  uint32_t(duty/100.0f * top_);
    pwm_set_gpio_level(en_, duty_top);
}

void Motor::backward(float duty) {
    gpio_put(in1_, 0);
    gpio_put(in2_, 1);
    uint32_t duty_top = uint32_t(duty/100.0f * top_);
    pwm_set_gpio_level(en_, duty_top);
}

void Motor::stop() {
    gpio_put(in1_, 1);
    gpio_put(in2_, 1);
    pwm_set_gpio_level(en_, top_);
}

void Motor::free() {
    gpio_put(in1_, 0);
    gpio_put(in2_, 0);
    pwm_set_gpio_level(en_, 0);
}