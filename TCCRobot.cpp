#include <stdio.h>
#include "pico/stdlib.h"
#include "inc/Motor.hpp"

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
