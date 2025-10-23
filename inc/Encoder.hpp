#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// IR SPI bits

#define MDR0    0b001 << 3
#define MDR1    0b010 << 3
#define DTR     0b011 << 3
#define CNTR    0b100 << 3
#define OTR     0b101 << 3
#define STR     0b110 << 3
#define NONE    0b111 << 3

#define CLR      0b00 << 6
#define RD       0b01 << 6
#define WR       0b10 << 6
#define LOAD     0b11 << 6

class Encoder {
public:
    Encoder(spi_inst_t *spi, uint8_t CS_Pin);

    void init();

    uint32_t getPulses();
    void resetPulses();

    uint32_t lastPulses;

private:
    spi_inst_t* spi_handler_;
    uint8_t CS_Pin_;

};

#endif