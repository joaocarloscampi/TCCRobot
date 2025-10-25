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

    uint32_t getLevel();
    uint32_t getTop();


private:
    uint pin_;
    uint slice_;
    uint32_t top_;
    uint32_t level_; 
};

#endif
