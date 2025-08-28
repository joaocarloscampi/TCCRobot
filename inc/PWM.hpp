#ifndef PWM_HPP
#define PWM_HPP

#include "pico/stdlib.h"
#include "hardware/pwm.h"

class PWM {
public:
    PWM(uint pin);

    void init(uint32_t freq, float div = 1.0f);
    void setDuty(float duty);   // duty em % (0–100)
    void enable();
    void disable();

private:
    uint pin_;
    uint slice_;
    uint32_t top_;
};

#endif
