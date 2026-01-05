#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// IR SPI bits (datasheet)

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

#define ENCODER_PPR 400.0f

/**
 * @brief Classe para gerenciar a comunicação cSPI om LS74366 (Leitura do Encoder)
 * 
 * Apresenta funções abstraídas que facilitam a comunicação com o chip LS74366
 * que permitem configurar e adquirir os dados de leitura dos Encoders de quadratura
 * do Robô móvel via SPI.
 * 
 * @param spi           Objeto SPI responsavel pela comunicação com a placa
 * @param CS_Pin        Pino CS responsável por habilitar a comunicação SPI
 * 
 * @author João Carlos
 */
class Encoder {
public:
    /**
     * @brief Construtor da classe Encoder
     * 
     * @param spi           Objeto SPI responsavel pela comunicação com a placa
     * @param CS_Pin        Pino CS responsável por habilitar a comunicação SPI
     * 
     */
    Encoder(spi_inst_t *spi, uint8_t CS_Pin);

    /**
     * @brief Inicializa o hardware necessário para o comunicação com o LS74366
     * 
     * @param None
     * 
     * @return None
     * 
     */
    void init();

    /**
     * @brief Obtém o valor de pulsos acumulados do encoder
     * 
     * @param None
     * 
     * @return encoder_pulses [uint32_t] Contém a quantidade de pulsos acumulados
     */
    int32_t getPulses();

    /**
     * @brief Reseta o contador de pulsos do encoder para 0
     * 
     * @param None
     * 
     * @return None
     */
    void resetPulses();

private:
    spi_inst_t* spi_handler_;
    uint8_t CS_Pin_;
    uint32_t lastPulses;

};

#endif