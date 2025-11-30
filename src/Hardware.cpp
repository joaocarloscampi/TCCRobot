#include "Hardware.h"

int pico_led_init(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}

void pico_set_led(bool led_on) {
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
}

void pico_spi_init(spi_inst_t *spi)
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
    spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Initialize SPI pins
    gpio_set_function(SCLK_SPI0, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_SPI0, GPIO_FUNC_SPI);
    gpio_set_function(MISO_SPI0, GPIO_FUNC_SPI);

}

void uart_init_pico(uart_inst_t *UART_ID, int BAUD_RATE) {
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