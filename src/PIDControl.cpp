#include "PIDControl.hpp"

// Construtor padrão: ganhos zerados, Ts = 1.0, limites definidos pelos macros
PIDControl::PIDControl()
    : Kp_(0.0f),
      Ki_(0.0f),
      Kd_(0.0f),
      Ts_(1.0f),
      setpoint_(0.0f),
      min_output_(PID_DEFAULT_MIN_OUTPUT),
      max_output_(PID_DEFAULT_MAX_OUTPUT),
      error_k_(0.0f),
      error_k_1(0.0f),
      error_k_2(0.0f),
      output_k_(0.0f),
      output_k_1(0.0f),
      output_k_2(0.0f),
      integral_(0.0f)
{}

// Construtor com ganhos
PIDControl::PIDControl(float kp, float ki, float kd, float ts)
    : Kp_(kp),
      Ki_(ki),
      Kd_(kd),
      Ts_(ts > 0.0f ? ts : 1.0f),
      setpoint_(0.0f),
      min_output_(PID_DEFAULT_MIN_OUTPUT),
      max_output_(PID_DEFAULT_MAX_OUTPUT),
      error_k_(0.0f),
      error_k_1(0.0f),
      error_k_2(0.0f),
      output_k_(0.0f),
      output_k_1(0.0f),
      output_k_2(0.0f),
      integral_(0.0f)
{}

// Reset dos estados internos
void PIDControl::reset() {
    error_k_ = error_k_1 = error_k_2 = 0.0f;
    output_k_ = output_k_1 = output_k_2 = 0.0f;
    integral_ = 0.0f;
}

// Atualiza a ação de controle usando o erro atual (PID posição).
// Anti-windup: clamping simples: só integra quando a integração não levaria a saturação adicional.
float PIDControl::update(float error) {
    // Shift histórico de erros e saídas
    error_k_2 = error_k_1;
    error_k_1 = error_k_;
    error_k_  = error;

    output_k_2 = output_k_1;
    output_k_1 = output_k_;

    /*

    // Proporcional
    float P = Kp_ * error_k_;

    // Derivada (diferença entre erro atual e anterior) / Ts
    float D = 0.0f;
    if (Ts_ != 0.0f) {
        D = Kd_ * (error_k_ - error_k_1) / Ts_;
    }

    // Integral: decisão de clamping (anti-windup simples)
    // Calculamos um valor de prova (sem atualização da integral) e avaliamos se isto levaria à saturação.
    // Se o resultado estiver saturando e a integração estaria aumentando a saturação na mesma direção,
    // então NÃO atualizamos a integral (congele-a). Caso contrário, integramos normalmente.
    //
    // Observação: Ki_ já é ganho "bruto". A integração se faz como integral += Ki * error * Ts.

    // cálculo do termo integral proposto
    float integral_proposed = integral_ + Ki_ * error_k_ * Ts_;

    // controle "proposto" com integral_proposed
    float u_proposed = P + integral_proposed + D;

    // condição de saturação e sinal do erro:
    bool saturating_high = (u_proposed > max_output_);
    bool saturating_low  = (u_proposed < min_output_);

    // Se estivermos propondo saturação alta e a integração está aumentando a saída (erro positivo),
    // então não atualize integral (congele).
    bool integrate = true;
    if (saturating_high && (error_k_ > 0.0f)) {
        integrate = false;
    } else if (saturating_low && (error_k_ < 0.0f)) {
        integrate = false;
    }

    if (integrate) {
        integral_ = integral_proposed;
    }
    // else integral_ permanece sem alteração (clamped)

    // Monta saída final (antes de saturar)
    float u = P + integral_ + D;

    */

    alpha = Kp_ + Ki_*Ts_/2;
    beta = Ki_*Ts_/2 - Kp_;

    float u = alpha * error_k_ + beta * error_k_1 + output_k_1;

    // Saturação final (limit output)
    float u_sat = saturate(u);

    // Atualiza histórico de saídas com valor final
    output_k_ = u_sat;

    // Retorna saída saturada
    return output_k_;
}

// Função de saturação
float PIDControl::saturate(float value) const {
    if (value > max_output_) return max_output_;
    if (value < min_output_) return min_output_;
    return value;
}

// ========== Setters ==========
void PIDControl::setKp(float kp) { Kp_ = kp; }
void PIDControl::setKi(float ki) { Ki_ = ki; }
void PIDControl::setKd(float kd) { Kd_ = kd; }
void PIDControl::setTs(float ts) { if (ts > 0.0f) Ts_ = ts; }
void PIDControl::setSetpoint(float sp) { setpoint_ = sp; }
void PIDControl::setOutputLimits(float min_out, float max_out) {
    // troca só se válidos (min < max)
    if (min_out < max_out) {
        min_output_ = min_out;
        max_output_ = max_out;
        // garante que output atual esteja dentro dos novos limites
        output_k_ = saturate(output_k_);
        output_k_1 = saturate(output_k_1);
        output_k_2 = saturate(output_k_2);
    }
}

// ========== Getters ==========
float PIDControl::getKp() const { return Kp_; }
float PIDControl::getKi() const { return Ki_; }
float PIDControl::getKd() const { return Kd_; }
float PIDControl::getTs() const { return Ts_; }
float PIDControl::getSetpoint() const { return setpoint_; }
float PIDControl::getMinOutput() const { return min_output_; }
float PIDControl::getMaxOutput() const { return max_output_; }

float PIDControl::getLastError() const { return error_k_; }
float PIDControl::getPrevError() const { return error_k_1; }
float PIDControl::getPrevError2() const { return error_k_2; }
float PIDControl::getLastOutput() const { return output_k_; }
float PIDControl::getPrevOutput() const { return output_k_1; }
float PIDControl::getPrevOutput2() const { return output_k_2; }
float PIDControl::getIntegral() const { return integral_; }
