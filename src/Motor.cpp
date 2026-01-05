#include "Motor.hpp"

Motor::Motor(uint in1, uint in2, uint en, uint8_t encoder_cs_pin, spi_inst_t *spi, bool inverted)
    : in1_(in1), in2_(in2), pwm_(en), encoder_(spi, encoder_cs_pin), is_inverted_(inverted) {}

void Motor::init() 
{
    // Define os pinos GPIO de direção IN
    gpio_init(in1_); gpio_set_dir(in1_, GPIO_OUT);
    gpio_init(in2_); gpio_set_dir(in2_, GPIO_OUT);

    // TODO: Analisar por que colocar div=1 faz o duty dobrar na prática
    uint32_t freq = 1000;
    float div = 2.0f;

    ratio_pulses_speed = 0;
    motor_speed_ = motor_pulses_ = 0;

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

int32_t Motor::get_encoder_pulses()
{
    return encoder_.getPulses();
}

void Motor::reset_encoder_pulses()
{
    encoder_.resetPulses();
}

void Motor::init_controller(float Kp, float Ki, float Kd, float Ts)
{
    pid_.setKp(Kp);
    pid_.setKi(Ki);
    pid_.setKd(Kd);
    pid_.setTs(Ts);
}

void Motor::set_control_setpoint(float setpoint)
{
    pid_.setSetpoint(setpoint);
}

void Motor::control_update(){
    float pulses_measurement = get_encoder_pulses();

    float r = 2.0*3.14159/(MOTOR_REDUCTION*ENCODER_PPR*pid_.getTs());
    float speed_measurement = pulses_measurement * r;

    if(is_inverted_)
    {
        pulses_measurement *= (-1.0f);
        speed_measurement *= (-1.0f);
    }

    set_speed(speed_measurement, pulses_measurement);

    float error = pid_.getSetpoint() - speed_measurement;
    float u = pid_.update(error);
    
    float duty = u/V_MAX*100;
    setDuty(duty);
    //setDuty(00.0f);
    forward();

    reset_encoder_pulses();
}

void Motor::set_speed(float speed_measurement, float pulses_measurement)
{
    motor_speed_ = speed_measurement;
    motor_pulses_ = pulses_measurement;
}

float Motor::get_speed()
{
    return motor_speed_;
}
float Motor::get_pulses()
{
    return motor_pulses_;
}

float Motor::get_control_action()
{
    return pid_.getLastOutput();
}