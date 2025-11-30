// Raspberry native includes
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// H-Bridge L298N Connections
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

// UART Communication
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Functions
int pico_led_init(void);
void pico_set_led(bool led_on);

void pico_spi_init(spi_inst_t *spi);
void uart_init_pico(uart_inst_t *UART_ID, int BAUD_RATE);