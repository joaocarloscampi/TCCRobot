#include "PWM.hpp"

PWM::PWM(uint pin) : pin_(pin) {}

void PWM::init(uint32_t freq, float div) {
    gpio_set_function(pin_, GPIO_FUNC_PWM);

    uint32_t f_sys = 125000000; // clock base do RP2040
    top_ = (f_sys / (freq * div)) - 1;

    slice_ = pwm_gpio_to_slice_num(pin_);
    pwm_set_clkdiv(slice_, div);
    pwm_set_wrap(slice_, top_);
    pwm_set_gpio_level(pin_, 0);
}

void PWM::setDuty(float duty) 
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 100.0f) duty = 100.0f;
    uint32_t level = static_cast<uint32_t>((duty / 100.0f) * top_);
    pwm_set_gpio_level(pin_, level);
}

void PWM::enable() 
{
    pwm_set_enabled(slice_, true);
}

void PWM::disable() 
{
    pwm_set_enabled(slice_, false);
}
