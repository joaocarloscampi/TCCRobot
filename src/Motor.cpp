#include "Motor.hpp"

Motor::Motor(uint in1, uint in2, uint en)
    : in1_(in1), in2_(in2), pwm_(en) {}

void Motor::init() 
{
    gpio_init(in1_); gpio_set_dir(in1_, GPIO_OUT);
    gpio_init(in2_); gpio_set_dir(in2_, GPIO_OUT);

    uint32_t freq = 1000;

    pwm_.init(freq);
    pwm_.enable();
}

void Motor::setDuty(float duty)
{
    duty_ = duty;
}

void Motor::forward() 
{
    gpio_put(in1_, 1);
    gpio_put(in2_, 0);
    pwm_.setDuty(duty_);
}

void Motor::backward() 
{
    gpio_put(in1_, 0);
    gpio_put(in2_, 1);
    pwm_.setDuty(duty_);
}

void Motor::stop() 
{
    gpio_put(in1_, 1);
    gpio_put(in2_, 1);
    duty_ = 100.0f;
    pwm_.setDuty(duty_); // freio
    duty_ = 0;
}

void Motor::free() 
{
    gpio_put(in1_, 0);
    gpio_put(in2_, 0);
    duty_ = 0;
    pwm_.setDuty(duty_);   // desligado
}
