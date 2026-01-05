// TCCRobot includes
#include "inc/Hardware.h"
#include "inc/Motor.hpp"

// Timers defines
#define TS_CONTROL 10
#define STEP_TIME_DELAY_MS 1000 

// UART Comm
#define UART_ID uart0
#define BAUD_RATE 115200

// SPI Comm
spi_inst_t *spi = spi0;

// Auxiliar Variables
char msg_uart[70];          // Array for UART messages 

int64_t elapsed_us = 0;     // Timer for step input (microsseconds)
int elapsed_sec = 0;        // Timer for step input (seconds)
int elapsed_ms = 0;         // Timer for step input (milisseconds)

volatile bool step_time = false;     // Step has started
volatile bool ts_time = false;     // Step has started

bool led_state = false;

float speed_setpoint = 5;   // rad/s

// Motor objects
Motor motorDireito_Frente(IN1_R, IN2_R, ENA_R, CS1, spi, false);
Motor motorDireito_Tras(IN4_R, IN3_R, ENB_R, CS2, spi, false);
Motor motorEsquerdo_Tras(IN4_L, IN3_L, ENB_L, CS3, spi, true);
Motor motorEsquerdo_Frente(IN2_L, IN1_L, ENA_L, CS4, spi, true);

bool repeating_timer_callback(__unused struct repeating_timer *t) {
    ts_time = true;
    return true;
}

int64_t alarm_callback(alarm_id_t id, __unused void *user_data) {
    printf("Timer %d fired!\n", (int) id);
    step_time = true;
    // Can return a value here in us to fire in the future
    return 0;
}

int main()
{
    // Raspberry Pi Pico innit
    stdio_init_all();

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    pico_spi_init(spi);

    uart_init_pico(UART_ID, BAUD_RATE);

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

    motorDireito_Frente.init_controller(10.6690, 169.3840, 0, 0.01);
    motorDireito_Tras.init_controller(10.6690, 169.3840, 0, 0.01);
    motorEsquerdo_Tras.init_controller(5, 50, 0, 0.01);
    motorEsquerdo_Frente.init_controller(10.6690, 169.3840, 0, 0.01);

    motorDireito_Frente.set_control_setpoint(speed_setpoint);
    motorDireito_Tras.set_control_setpoint(speed_setpoint);
    motorEsquerdo_Tras.set_control_setpoint(speed_setpoint);
    motorEsquerdo_Frente.set_control_setpoint(speed_setpoint);

    struct repeating_timer timer;

    // Call alarm_callback in 1 seconds
    add_alarm_in_ms(1000, alarm_callback, NULL, false);

    // Wait for alarm callback to set timer_fired
    while (!step_time) {
        tight_loop_contents();
    }

    add_repeating_timer_ms(-TS_CONTROL, repeating_timer_callback, NULL, &timer);

    absolute_time_t start_time = get_absolute_time();
    
    while (true) {
        // Tempo atual
        absolute_time_t now_time = get_absolute_time();

        // Calcula diferença em microssegundos
        elapsed_us = absolute_time_diff_us(start_time, now_time);

        // Conversão de escala - Step Logging
        elapsed_sec = elapsed_us / (1000*1000);
        elapsed_ms = elapsed_us / (1000) - 1000*elapsed_sec;

        /*
        if( (elapsed_us > STEP_TIME_DELAY_MS*1000))
        {
            pico_set_led(true);
            
            // Teste de controlador

            motorDireito_Frente.control_update();
            motorDireito_Tras.control_update();
            motorEsquerdo_Tras.control_update();
            motorEsquerdo_Frente.control_update();

            step_time = true;
        }
        */

        // Aquisição da leitura do encoder

        if(ts_time)
        {
            ts_time = false;

            motorDireito_Frente.control_update();
            motorDireito_Tras.control_update();
            motorEsquerdo_Tras.control_update();
            motorEsquerdo_Frente.control_update();

            int count_values[8] = {0};

            count_values[0] = motorDireito_Frente.get_pulses();
            count_values[2] = motorDireito_Tras.get_pulses();
            count_values[4] = motorEsquerdo_Tras.get_pulses();
            count_values[6] = motorEsquerdo_Frente.get_pulses();

            count_values[1] = motorDireito_Frente.get_control_action()*1000;
            count_values[3] = motorDireito_Tras.get_control_action()*1000;
            count_values[5] = motorEsquerdo_Tras.get_control_action()*1000;
            count_values[7] = motorEsquerdo_Frente.get_control_action()*1000;

            // Envio de dados via UART - Step info

            
            snprintf(msg_uart, sizeof(msg_uart), "%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d\r\n",  step_time,
                                                                        count_values[0],
                                                                        count_values[1],
                                                                        count_values[2],
                                                                        count_values[3],
                                                                        count_values[4],
                                                                        count_values[5],
                                                                        count_values[6],
                                                                        count_values[7],
                                                                        int(speed_setpoint),
                                                                        elapsed_sec,
                                                                        elapsed_ms);
                                                                        
            /*snprintf(msg_uart, sizeof(msg_uart), "%d;%d;%d\r\n",  step_time,
                                                                        count_values[4],
                                                                        count_values[5]);*/

            uart_puts(UART_ID, msg_uart);

            pico_set_led(led_state);
            led_state = !led_state;
        }
        
    }
}
