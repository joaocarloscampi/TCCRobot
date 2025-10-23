#include <stdio.h>
#include "pico/stdlib.h"
#include "inc/Motor.hpp"
#include "hardware/spi.h"
#include "inc/Encoder.hpp"

#define LED_DELAY_MS 10

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

// UART Comm

#define UART_ID uart0
#define BAUD_RATE 115200

#define UART_TX_PIN 0
#define UART_RX_PIN 1

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

}

void uart_init_pico() {
    // Inicializa UART
    uart_init(UART_ID, BAUD_RATE);

    // Configura os pinos
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Opcional: Configura formato (8 bits, 1 stop, sem paridade)
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);

    // Opcional: Habilita FIFO
    uart_set_fifo_enabled(UART_ID, true);

    // (Opcional) Espera um pouco para estabilizar
    sleep_us(100);
}

int main()
{
    stdio_init_all();

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    pico_spi_init();

    uart_init_pico();

    Motor motorDireito_Frente(IN1_R, IN2_R, ENA_R);
    Motor motorDireito_Tras(IN4_R, IN3_R, ENB_R);
    Motor motorEsquerdo_Frente(IN2_L, IN1_L, ENA_L);
    Motor motorEsquerdo_Tras(IN4_L, IN3_L, ENB_L);

    motorDireito_Frente.init();
    motorDireito_Tras.init();
    motorEsquerdo_Frente.init();
    motorEsquerdo_Tras.init();

    Encoder encoder1(spi0, CS1);
    Encoder encoder2(spi0, CS2);
    Encoder encoder3(spi0, CS3);
    Encoder encoder4(spi0, CS4);

    encoder1.init();
    encoder2.init();
    encoder3.init();
    encoder4.init();

    sleep_ms(1000);

    // Acionamento teste dos motores
    printf("Frente 50\n");
    motorDireito_Frente.setDuty(50.0f);
    motorDireito_Tras.setDuty(50.0f);
    motorEsquerdo_Frente.setDuty(50.0f);
    motorEsquerdo_Tras.setDuty(50.0f);

    motorDireito_Frente.forward();
    motorDireito_Tras.forward();
    motorEsquerdo_Frente.forward();
    motorEsquerdo_Tras.forward();

    char msg_uart[64];
    
    while (true) {

        // Debug para verificar a leitura

        int count_values[4] = {0};

        count_values[0] = encoder1.getPulses();
        encoder1.resetPulses();
        count_values[1] = encoder2.getPulses();
        encoder2.resetPulses();
        count_values[2] = encoder3.getPulses();
        encoder3.resetPulses();
        count_values[3] = encoder4.getPulses();
        encoder4.resetPulses();

        // Envio de dados via UART

        snprintf(msg_uart, sizeof(msg_uart), "%d;%d;%d;%d\r\n", count_values[0],
                                                                count_values[1],
                                                                count_values[2],
                                                                count_values[3]);
        uart_puts(UART_ID, msg_uart);

        sleep_ms(LED_DELAY_MS);
    }
}
