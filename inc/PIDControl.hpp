#ifndef PIDCONTROL_HPP
#define PIDCONTROL_HPP

#include <cstdint>
#include <cmath>

/*
 * PIDControl.hpp
 *
 * Classe PID genérica para uso embarcado (Raspberry Pi Pico / RP2040 ou similar).
 *
 * Principais características:
 * - PID de posição (forma mais comum).
 * - Anti-windup por clamping (travamento simples da integral).
 * - Armazena e[k], e[k-1], e[k-2] e u[k], u[k-1], u[k-2] como variáveis separadas.
 * - Getters/Setters para Kp, Ki, Kd, Ts, setpoint e limites de saída.
 * - Métodos: update(error) -> calcula ação de controle (float).
 *
 * Observação:
 * - Os ganhos são "brutos" (não fazemos discretização automática).
 * - Ts (tempo de amostragem) deve ser configurado via setTs().
 */

#ifndef PID_DEFAULT_MIN_OUTPUT
#define PID_DEFAULT_MIN_OUTPUT (-10.0f)
#endif

#ifndef PID_DEFAULT_MAX_OUTPUT
#define PID_DEFAULT_MAX_OUTPUT (10.0f)
#endif

class PIDControl {
public:
    // Construtores
    PIDControl();
    PIDControl(float kp, float ki, float kd, float ts);

    // Core
    float update(float speed_measurement); // calcula u(k) baseado no erro atual (position form)

    // Reset / inicialização
    void reset(); // zera estados internos (integral, erros e saídas)

    // Setters - preferiu métodos separados
    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    void setTs(float ts);
    void setSetpoint(float sp);
    void setOutputLimits(float min_out, float max_out);

    // Getters
    float getKp() const;
    float getKi() const;
    float getKd() const;
    float getTs() const;
    float getSetpoint() const;
    float getMinOutput() const;
    float getMaxOutput() const;

    // Estado/Histórico (getters)
    float getLastError() const;      // e[k]
    float getPrevError() const;      // e[k-1]
    float getPrevError2() const;     // e[k-2]
    float getLastOutput() const;     // u[k]
    float getPrevOutput() const;     // u[k-1]
    float getPrevOutput2() const;    // u[k-2]
    float getIntegral() const;       // valor integral atual

    // Utility: se quiser alimentar timestamps externos (por exemplo get_absolute_time())
    // e calcular Ts externamente, implemente chamada externa. Aqui mantemos Ts fixo.

private:
    // Ganhos
    float Kp_;
    float Ki_;
    float Kd_;

    // Amostragem
    float Ts_;

    // Variaveis auxiliares
    float alpha;
    float beta;

    // Setpoint
    float setpoint_;

    // Saturação (padrões via DEFINE acima)
    float min_output_;
    float max_output_;

    // Estados para histórico (separados conforme solicitado)
    float error_k_;      // e[kT]
    float error_k_1;     // e[(k-1)T]
    float error_k_2;     // e[(k-2)T]

    float output_k_;     // m[kT]
    float output_k_1;    // m[(k-1)T]
    float output_k_2;    // m[(k-2)T]

    float y_k_;     // y[kT]
    float y_k_1;    // y[(k-1)T]
    float y_k_2;    // y[(k-2)T]

    float m_;             // m[kT]

    // Integrador (valor acumulado)
    float integral_;

    // Internals: calcula componente derivativa (implementado em .cpp)

    // Função interna de saturação
    float saturate(float value) const;
};

#endif // PIDCONTROL_HPP
