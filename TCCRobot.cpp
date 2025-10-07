#include <stdio.h>
#include "pico/stdlib.h"
#include "inc/Motor.hpp"
#include "hardware/spi.h"

#define LED_DELAY_MS 2000

#define IN1_R 10
#define IN2_R 11
#define IN3_R 12
#define IN4_R 13
#define ENA_R 6
#define ENB_R 7

#define IN1_L 20
#define IN2_L 21
#define IN3_L 22
#define IN4_L 26
#define ENA_L 8
#define ENB_L 9

// SPI Communication - Encoder 

#define MOSI_SPI0 19
#define SCLK_SPI0 18
#define MISO_SPI0 16

#define CS1 2
#define CS2 3
#define CS3 4
#define CS4 5

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

int pico_led_init(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}

void pico_set_led(bool led_on) {
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
}

void pico_spi_init(void)
{
    // Ports
    spi_inst_t *spi = spi0;

    // Initialize CS pins high
    gpio_init(CS1);
    gpio_init(CS2);
    gpio_init(CS3);
    gpio_init(CS4);
    gpio_set_dir(CS1, GPIO_OUT);
    gpio_set_dir(CS2, GPIO_OUT);
    gpio_set_dir(CS3, GPIO_OUT);
    gpio_set_dir(CS4, GPIO_OUT);
    gpio_put(CS1, 1);
    gpio_put(CS2, 1);
    gpio_put(CS3, 1);
    gpio_put(CS4, 1);

    // Initialize SPI port at 1 MHz
    spi_init(spi, 1000 * 1000);

    // Set SPI format
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Initialize SPI pins
    gpio_set_function(SCLK_SPI0, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_SPI0, GPIO_FUNC_SPI);
    gpio_set_function(MISO_SPI0, GPIO_FUNC_SPI);

    uint8_t msg;

    // Configura Encoder 1

    gpio_put(CS1, 0);
    sleep_us(10);

    // Define escrita no registrador MDR0
    msg = WR | MDR0;
    spi_write_blocking(spi0, &msg, 1);

    // Define escrita no registrador MDR0
    msg  =  0x00;
    msg |= (0b11 << 0);  //x4 quad count mode
    msg |= (0b00 << 2);  //free-running
    msg |= (0b00 << 4);  //disable index
    msg |= (0b0  << 6);  //Assincronous
    msg |= (0b0  << 7);  //Filter Clock 1
    spi_write_blocking(spi0, &msg, 1);

    gpio_put(CS1, 1);
    sleep_us(10);

    // Define escrita no registrador MDR1
    gpio_put(CS1, 0);
    sleep_us(10);

    msg = 0x00;
    msg = WR | MDR1;
    spi_write_blocking(spi0, &msg, 1);

    // Define escrita no registrador MDR1

    msg  =  0x00;
    msg |= (0b01 << 0); //4 bytes counter mode
    // Resto dos bits mantém em 0...
    spi_write_blocking(spi0, &msg, 1);

    gpio_put(CS1, 1);
    sleep_us(10);
}

int main()
{
    stdio_init_all();

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    pico_spi_init();

    Motor motorDireito_Frente(IN1_R, IN2_R, ENA_R);
    Motor motorDireito_Tras(IN4_R, IN3_R, ENB_R);
    Motor motorEsquerdo_Frente(IN2_L, IN1_L, ENA_L);
    Motor motorEsquerdo_Tras(IN4_L, IN3_L, ENB_L);

    motorDireito_Frente.init();
    motorDireito_Tras.init();
    motorEsquerdo_Frente.init();
    motorEsquerdo_Tras.init();

    sleep_ms(1000);

    while (true) {

        pico_set_led(true);
        printf("Frente 50\n");
        motorDireito_Frente.setDuty(50.0f);
        motorDireito_Tras.setDuty(50.0f);
        motorEsquerdo_Frente.setDuty(50.0f);
        motorEsquerdo_Tras.setDuty(50.0f);
        

        motorDireito_Frente.forward();
        motorDireito_Tras.forward();
        motorEsquerdo_Frente.forward();
        motorEsquerdo_Tras.forward();

        sleep_ms(LED_DELAY_MS);

        motorDireito_Frente.stop();
        motorDireito_Tras.stop();

        sleep_ms(LED_DELAY_MS);

        motorEsquerdo_Frente.stop();
        motorEsquerdo_Tras.stop();
        
        sleep_ms(LED_DELAY_MS);

        pico_set_led(false);
        printf("Tras 50\n");
        motorDireito_Frente.setDuty(50.0f);
        motorDireito_Tras.setDuty(50.0f);
        motorEsquerdo_Frente.setDuty(50.0f);
        motorEsquerdo_Tras.setDuty(50.0f);

        motorDireito_Frente.backward();
        motorDireito_Tras.backward();
        motorEsquerdo_Frente.backward();
        motorEsquerdo_Tras.backward();

        sleep_ms(LED_DELAY_MS);

        motorEsquerdo_Frente.free();
        motorEsquerdo_Tras.free();

        sleep_ms(LED_DELAY_MS);

        motorDireito_Frente.free();
        motorDireito_Tras.free();
        
        sleep_ms(LED_DELAY_MS);
    }
}
