#pragma once
#include "pico/stdlib.h"
#include "PWM.hpp"
#include "inc/Encoder.hpp"
#include "inc/PIDControl.hpp"

#define MOTOR_REDUCTION 30.0f
#define V_MAX 12.0f

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
    Motor(uint in1, uint in2, uint en, uint8_t encoder_cs_pin, spi_inst_t* spi, bool inverted);

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
    int32_t get_encoder_pulses();
    
    /**
     * @brief Reseta o contador de pulsos do encoder para 0
     * 
     * @param None
     * 
     * @return None
     */
    void reset_encoder_pulses();

    // TODO -   Como lidar com inversões de direção da melhor forma?
    // TODO -   Função stop control
    // TODO -   Faz mais sentido deixar o objeto controle como publico? 
    //              Se colocar os sets de cada ganho, vai repetir código e não sei se faz sentido
    //              O TODO é referente a definição posterior dos ganhos. O Controle PID permite fazer separado, mas e o Motor?
    //              Faz sentido criar funções aqui também, sendo que o controlador pode ser de outro tipo?
    // TODO -   ENCODER_PPR precisa estar linkado com o valor de leitura do encoder, para calculo da velocidade angular.
    void init_controller(float Kp, float Ki, float Kd, float Ts);
    void set_control_setpoint(float setpoint);
    void control_update();
    void set_speed(float speed_measurement, float pulses_measurement);
    float get_speed();
    float get_pulses();
    float get_control_action();
    float get_speed_filter();
    float get_gain_Kp();
    float get_gain_Ki();
    float get_gain_Kd();
    void apply_gain_scheduling(float weight);
    

private:
    uint8_t Encoder_CS_Pin_;
    uint in1_, in2_;
    bool is_inverted_;
    float duty_;
    float ratio_pulses_speed;
    float motor_speed_;
    float motor_pulses_;
    float motor_speed_filter_;
    float alpha_;
    bool filter_speed_;
    
    Encoder encoder_;
    spi_inst_t* spi_;

    PWM pwm_;

    PIDControl pid_;
};
