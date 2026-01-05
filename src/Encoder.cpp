#include "Encoder.hpp"

Encoder::Encoder(spi_inst_t *spi, uint8_t CS_Pin) : spi_handler_(spi),CS_Pin_(CS_Pin)
{
    lastPulses = 0;
}

void Encoder::init() {
    /*
    TODO #1: Transformar essa definição de registradores configuraveis de acordo com os 
             dados do datasheet. Atualmente ele está apenas para um tipo de leitura   
    */
    uint8_t msg;

    // ------------------
    // Registrador MDR0 
    // ------------------

    gpio_put(CS_Pin_, 0);
    sleep_us(10);

    // Define escrita no registrador MDR0
    msg = WR | MDR0;
    spi_write_blocking(spi_handler_, &msg, 1);

    // Dados para escrita no registrador MDR0
    msg  =  0x00;
    msg |= (0b11 << 0);  //x4 quad count mode
    msg |= (0b00 << 2);  //free-running
    msg |= (0b00 << 4);  //disable index
    msg |= (0b0  << 6);  //Assincronous
    msg |= (0b0  << 7);  //Filter Clock 1
    spi_write_blocking(spi_handler_, &msg, 1);

    gpio_put(CS_Pin_, 1);
    sleep_us(10);

    // ------------------
    // Registrador MDR1
    // ------------------

    // Define escrita no registrador MDR1
    gpio_put(CS_Pin_, 0);
    sleep_us(10);

    msg = 0x00;
    msg = WR | MDR1;
    spi_write_blocking(spi_handler_, &msg, 1);

    // Dados para escrita no registrador MDR1

    msg  =  0x00;
    msg |= (0b00 << 0); //4 bytes counter mode
    // Resto dos bits mantém em 0...
    spi_write_blocking(spi_handler_, &msg, 1);

    gpio_put(CS_Pin_, 1);
    sleep_us(10);

    // ------------------
    // Registrador CNTR
    // ------------------

    // Valor em DTR para limpar o registrador CNT

    gpio_put(CS_Pin_, 0);
    sleep_us(10);

    msg  =  0x00;
    msg  = WR | DTR; 
    spi_write_blocking(spi_handler_, &msg, 1);

    msg = 0x00;
    for (uint8_t i = 0; i < 4; i++)
        spi_write_blocking(spi_handler_, &msg, 1);

    gpio_put(CS_Pin_, 1);
    sleep_us(10);

    // Carregar os dados em CNTR via LOAD command

    gpio_put(CS_Pin_, 0);
    sleep_us(10);

    msg  =  0x00;
    msg  = LOAD | CNTR; 
    spi_write_blocking(spi_handler_, &msg, 1);

    gpio_put(CS_Pin_, 1);
    sleep_us(10);
}

int32_t Encoder::getPulses()
{
    // Acesso aos dados via SPI

    uint8_t msg = 0; 
    gpio_put(CS_Pin_, 0); 

    msg = RD | CNTR; 
    spi_write_blocking(spi_handler_, &msg, 1); 

    uint8_t msgRx[4]; 
    for (uint8_t i = 0; i < 4; i++)
    {
        spi_read_blocking(spi_handler_, 0, &msgRx[i], 1); 
    }

    gpio_put(CS_Pin_, 1); 
    sleep_us(10);

    // Conversão da mensagem recebida

    int32_t count_value=0;
    
    count_value = (msgRx[0] << 8) + msgRx[1];
    count_value = (count_value << 8) + msgRx[2];
    count_value = (count_value << 8) + msgRx[3];

    lastPulses = count_value;
    
    return count_value;
}

void Encoder::resetPulses()
{

    // Valor em DTR para limpar o registrador CNT
    gpio_put(CS_Pin_, 0);
    sleep_us(10);

    uint8_t msg  =  0x00;
    msg  = WR | DTR; 
    spi_write_blocking(spi0, &msg, 1);

    msg = 0x00;
    for (uint8_t i = 0; i < 4; i++)
        spi_write_blocking(spi0, &msg, 1);

    gpio_put(CS_Pin_, 1);
    sleep_us(10);

    // Carregar os dados em CNTR via LOAD command
    gpio_put(CS_Pin_, 0);
    sleep_us(10);

    msg  =  0x00;
    msg  = LOAD | CNTR; 
    spi_write_blocking(spi0, &msg, 1);

    gpio_put(CS_Pin_, 1);
    sleep_us(10);
}