#include <stdio.h>
#include "pico/stdlib.h"
#include "inc/Motor.hpp"

#define LED_DELAY_MS 1000

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

int pico_led_init(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}

void pico_set_led(bool led_on) {
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
}

int main()
{
    stdio_init_all();

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

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

        printf("Frente 50\n");
        motorDireito_Frente.forward(50.0f);
        motorDireito_Tras.forward(50.0f);
        motorEsquerdo_Frente.forward(50.0f);
        motorEsquerdo_Tras.forward(50.0f);

        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);

        printf("Frente 25\n");
        motorDireito_Frente.forward(25.0f);
        motorDireito_Tras.forward(25.0f);
        motorEsquerdo_Frente.forward(25.0f);
        motorEsquerdo_Tras.forward(25.0f);
        
        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);

        //printf("Parado\n");
        //motorDireito.stop();
        //motorEsquerdo.stop();
        //sleep_ms(2000);
    }
}
