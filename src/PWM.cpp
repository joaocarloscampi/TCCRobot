#include "PWM.hpp"

PWM::PWM(uint pin) : pin_(pin) {}

void PWM::init(uint32_t freq) {

    // TODO: Essa configuração de pwm não está correta, estourando variaveis
    gpio_set_function(pin_, GPIO_FUNC_PWM); // Define pino PWM

    const float f_sys = 125000000.0f; // clock base do RP2040

    // divisor ideal assumindo TOP máximo
    float div_ideal = f_sys / (freq * 65536.0f);

    // divisor ideal assumindo TOP máximo
    // limitar ao range do hardware
    if(div_ideal < 1.0f)
        div_ideal = 1.0f;

    if(div_ideal > 255.9375f)
        div_ideal = 255.9375f;

    // quantização para resolução 1/16
    float div = (float)((int)(div_ideal * 16.0f)) / 16.0f;

    // recalcular TOP
    float top_f = (f_sys / (freq * div)) - 1.0f;

    if(top_f > 65534.0f)
        top_f = 65534.0f;

    if(top_f < 1.0f)
        top_f = 1.0f;

    top_ = (uint16_t)top_f;


    slice_ = pwm_gpio_to_slice_num(pin_);

    pwm_set_clkdiv(slice_, div);
    pwm_set_wrap(slice_, top_);

    pwm_set_gpio_level(pin_, 0);

    //pwm_set_enabled(slice_, true);
}

void PWM::setDuty(float duty) 
{
    // Retificação do valor de duty
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 100.0f) duty = 100.0f;
    
    level_ = static_cast<uint32_t>((duty / 100.0f) * top_);
    pwm_set_gpio_level(pin_, level_);
}

void PWM::enable() 
{
    pwm_set_enabled(slice_, true);
}

void PWM::disable() 
{
    pwm_set_enabled(slice_, false);
}

uint32_t PWM::getLevel()
{
    return level_;
}

uint32_t PWM::getTop()
{
    return top_;
}