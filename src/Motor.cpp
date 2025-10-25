#include "Motor.hpp"

Motor::Motor(uint in1, uint in2, uint en, uint8_t encoder_cs_pin, spi_inst_t *spi)
    : in1_(in1), in2_(in2), pwm_(en), encoder_(spi, encoder_cs_pin) {}

void Motor::init() 
{
    gpio_init(in1_); gpio_set_dir(in1_, GPIO_OUT);
    gpio_init(in2_); gpio_set_dir(in2_, GPIO_OUT);

    // TODO: Analisar por que colocar div=1 faz o duty dobrar na prática
    uint32_t freq = 1000;
    float div = 2.0f;

    pwm_.init(freq,div);
    pwm_.enable();

    encoder_.init();
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

uint32_t Motor::get_encoder_pulses()
{
    return encoder_.getPulses();
}

void Motor::reset_encoder_pulses()
{
    encoder_.resetPulses();
}