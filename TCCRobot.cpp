// Raspberry native includes
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// TCCRobot includes
#include "inc/Hardware.h"
#include "inc/Motor.hpp"

// Timers defines
#define LED_DELAY_MS 10
#define STEP_TIME_DELAY_MS 1000 

// UART Comm
#define UART_ID uart0
#define BAUD_RATE 115200

// SPI Comm
spi_inst_t *spi = spi0;

// Auxiliar Variables
char msg_uart[64];          // Array for UART messages 

int64_t elapsed_us = 0;     // Timer for step input (microsseconds)
int elapsed_sec = 0;        // Timer for step input (seconds)
int elapsed_ms = 0;         // Timer for step input (milisseconds)

bool step_time = false;     // Step has started

// Checklist #3
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
    // Checklist #2
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
    // Raspberry Pi Pico innit
    stdio_init_all();

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    pico_spi_init();

    uart_init_pico();

    // Motor objects
    Motor motorDireito_Frente(IN1_R, IN2_R, ENA_R, CS1, spi0);
    Motor motorDireito_Tras(IN4_R, IN3_R, ENB_R, CS2, spi0);
    Motor motorEsquerdo_Frente(IN2_L, IN1_L, ENA_L, CS3, spi0);
    Motor motorEsquerdo_Tras(IN4_L, IN3_L, ENB_L, CS4, spi0);

    motorDireito_Frente.init();
    motorDireito_Tras.init();
    motorEsquerdo_Frente.init();
    motorEsquerdo_Tras.init();

    sleep_ms(1000);

    // Degrau iniciando em 0 - Motor não travado
    printf("Frente 0\n");
    motorDireito_Frente.setDuty(0.0f);
    motorDireito_Tras.setDuty(0.0f);
    motorEsquerdo_Frente.setDuty(0.0f);
    motorEsquerdo_Tras.setDuty(0.0f);

    motorDireito_Frente.forward();
    motorDireito_Tras.forward();
    motorEsquerdo_Frente.forward();
    motorEsquerdo_Tras.forward();

    absolute_time_t start_time = get_absolute_time();
    
    while (true) {
        // Tempo atual
        absolute_time_t now_time = get_absolute_time();

        // Calcula diferença em microssegundos
        elapsed_us = absolute_time_diff_us(start_time, now_time);

        // Conversão de escala - Step Logging
        elapsed_sec = elapsed_us / (1000*1000);
        elapsed_ms = elapsed_us / (1000) - 1000*elapsed_sec;

        if(!step_time && elapsed_us > STEP_TIME_DELAY_MS*1000)
        {
            // Acionamento teste dos motores
            float duty_cicle = 100.0f;
            printf("Frente 0\n");
            motorDireito_Frente.setDuty(duty_cicle);
            motorDireito_Tras.setDuty(duty_cicle);
            motorEsquerdo_Frente.setDuty(duty_cicle);
            motorEsquerdo_Tras.setDuty(duty_cicle);

            motorDireito_Frente.forward();
            motorDireito_Tras.forward();
            motorEsquerdo_Frente.forward();
            motorEsquerdo_Tras.forward();

            step_time = true;
        }

        // Aquisição da leitura do encoder

        int count_values[4] = {0};

        count_values[0] = motorDireito_Frente.get_encoder_pulses();
        motorDireito_Frente.reset_encoder_pulses();
        count_values[1] = motorDireito_Tras.get_encoder_pulses();
        motorDireito_Tras.reset_encoder_pulses();
        count_values[2] = motorEsquerdo_Frente.get_encoder_pulses();
        motorEsquerdo_Frente.reset_encoder_pulses();
        count_values[3] = motorEsquerdo_Tras.get_encoder_pulses();
        motorEsquerdo_Tras.reset_encoder_pulses();

        // Envio de dados via UART - Step info

        snprintf(msg_uart, sizeof(msg_uart), "%d;%d;%d;%d;%d;%d;%d\r\n",  step_time,
                                                                    count_values[0],
                                                                    count_values[1],
                                                                    count_values[2],
                                                                    count_values[3],
                                                                    elapsed_sec,
                                                                    elapsed_ms);

        uart_puts(UART_ID, msg_uart);

        // Delay fixo para o loop 
        // Checklist #4
        sleep_ms(LED_DELAY_MS);
    }
}
