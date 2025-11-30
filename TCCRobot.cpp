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

int main()
{
    // Raspberry Pi Pico innit
    stdio_init_all();

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    pico_spi_init(spi);

    uart_init_pico(UART_ID, BAUD_RATE);

    // Motor objects
    Motor motorDireito_Frente(IN1_R, IN2_R, ENA_R, CS1, spi);
    Motor motorDireito_Tras(IN4_R, IN3_R, ENB_R, CS2, spi);
    Motor motorEsquerdo_Frente(IN2_L, IN1_L, ENA_L, CS3, spi);
    Motor motorEsquerdo_Tras(IN4_L, IN3_L, ENB_L, CS4, spi);

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
            pico_set_led(true);
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
