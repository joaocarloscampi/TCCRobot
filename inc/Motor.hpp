#pragma once
#include "pico/stdlib.h"
#include "PWM.hpp"
#include "inc/Encoder.hpp"

/**
 * @brief Classe para gerenciar os motores do Robô
 * 
 * Apresenta funções para controle de direção do motor, leitura de encoder
 * e controle de velocidade. Parametros aqui mostrados são relacionados ao
 * construtor.
 * 
 * @param in1               Pino na Raspberry Pi relacionado a entrada IN1 do Motor
 * @param in2               Pino na Raspberry Pi relacionado a entrada IN2 do Motor
 * @param en                Pino na Raspberry Pi relacionado a entrada EN do Motor
 * @param encoder_cs_pin    Pino na Raspberry Pi relacionado ao sinal CS para o Encoder SPI
 * @param spi               Objeto SPI para comunicação do Encoder
 * 
 * @author João Carlos
 */
class Motor {
public:
    /**
     * @brief Construtor da classe Motor
     * 
     * @param in1               Pino na Raspberry Pi relacionado a entrada IN1 do Motor
     * @param in2               Pino na Raspberry Pi relacionado a entrada IN2 do Motor
     * @param en                Pino na Raspberry Pi relacionado a entrada EN do Motor
     * @param encoder_cs_pin    Pino na Raspberry Pi relacionado ao sinal CS para o Encoder SPI
     * @param spi               Objeto SPI para comunicação do Encoder
     * 
     */
    Motor(uint in1, uint in2, uint en, uint8_t encoder_cs_pin, spi_inst_t* spi);

    /**
     * @brief Inicializa o hardware necessário para o Motor e o Encoder
     * 
     * @param None
     * 
     * @return None
     * 
     */
    void init();

    /**
     * @brief Define o duty-cicle aplicado no motor
     * 
     * @param duty  Valor em float do duty-cicle 
     * 
     * @return None
     * 
     * @example motor.setDuty(50.0f)
     */
    void setDuty(float duty);

    /**
     * @brief Acionamento do motor para frente
     * 
     * @param None
     * 
     * @return None
     */
    void forward();

    /**
     * @brief Acionamento do motor para trás
     * 
     * @param None
     * 
     * @return None
     */
    void backward();

    /**
     * @brief Força a parada do motor
     * 
     * @param None
     * 
     * @return None
     * 
     * @details Essa função aplica tensão máxima no motor, com o objetivo de travá-lo
     */
    void stop();

    /**
     * @brief Mantém o motor livre, sem acionamento
     * 
     * @param None
     * 
     * @return None
     * 
     * @details Essa função aplica tensão zero no motor, Deixando-o livre para funcionamento
     */
    void free();

    /**
     * @brief Obtém o valor de pulsos acumulados no encoder
     * 
     * @param None
     * 
     * @return encoder_pulses [uint32_t] Contém a quantidade de pulsos acumulados
     */
    uint32_t get_encoder_pulses();
    
    /**
     * @brief Reseta o contador de pulsos do encoder para 0
     * 
     * @param None
     * 
     * @return None
     */
    void reset_encoder_pulses();

    // Checklist #1
    PWM pwm_;

private:
    uint8_t Encoder_CS_Pin_;
    uint in1_, in2_;
    float duty_;
    
    Encoder encoder_;
    spi_inst_t* spi_;
};
