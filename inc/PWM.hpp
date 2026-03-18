#ifndef PWM_HPP
#define PWM_HPP

#include "pico/stdlib.h"
#include "hardware/pwm.h"

/**
 * @brief Classe para gerenciar o periférico PWM da Raspberry Pi Pico
 * 
 * Apresenta funções para definição de duty cicle, hablitar e desabilitar.
 * 
 * @param pin       Pino na Raspberry Pi relacionado a saída PWM
 * 
 * @author João Carlos
 */
class PWM {
public:
    /**
     * @brief Construtor da classe Motor
     *  
     * @param pin       Pino na Raspberry Pi relacionado a saída PWM
     */
    PWM(uint pin);

    /**
     * @brief Inicializa o hardware necessário para o PWM
     * 
     * @param None
     * 
     * @return None
     * 
     */
    void init(uint32_t freq);
    
     /**
     * @brief Define o duty-cicle (t_on) do sinal PWM
     * 
     * @param duty  [float] Valor em float do duty-cicle 
     * 
     * @return None
     * 
     * @example pwm.setDuty(50.0f)
     * 
     * @todo Retornar erro se valor de duty-cicle for inválida
     */
    void setDuty(float duty);   // duty em % (0–100)

    /**
     * @brief Habilita a saída PWM configurada
     * 
     * @param None
     * 
     * @return None
     * 
     * @todo Retornar erro se pwm não foi configurado corretamente
     */
    void enable();

    /**
     * @brief Desabilita a saída PWM configurada
     * 
     * @param None
     * 
     * @return None
     * 
     * @todo Retornar erro se pwm não foi habilitado
     */
    void disable();

    /**
     * @brief Obtém o valor de configuração level
     * 
     * @param None
     * 
     * @return level_value [uint32_t]
     */
    uint32_t getLevel();
    
    /**
     * @brief Obtém o valor de configuração top
     * 
     * @param None
     * 
     * @return top_value [uint32_t]
     */
    uint32_t getTop();


private:
    uint pin_;
    uint slice_;
    uint32_t top_;
    uint32_t level_; 
};

#endif
