#include "PIDControl.hpp"

// Construtor padrão: ganhos zerados, Ts = 1.0, limites definidos pelos macros
PIDControl::PIDControl()
    : Kp_(15.0f),
      Ki_(15.0f/0.05),
      Kd_(0.0f),
      Ts_(0.01f),
      setpoint_(0.0f),
      min_output_(PID_DEFAULT_MIN_OUTPUT),
      max_output_(PID_DEFAULT_MAX_OUTPUT),
      error_k_(0.0f),
      error_k_1(0.0f),
      error_k_2(0.0f),
      output_k_(0.0f),
      output_k_1(0.0f),
      output_k_2(0.0f),
      y_k_(0.0f),
      y_k_1(0.0f),
      y_k_2(0.0f),
      m_(0.0f),
      integral_(0.0f)
{
    // Valores padrão para Gain Scheduling
    weights_test_[0] = 0;
    weights_test_[1] = 1;
    weights_test_[2] = 2.5;
    Kp_weights_[0] = 15;
    Kp_weights_[1] = 10;
    Kp_weights_[2] = 10;
    Ti_weights_[0] = 0.05;
    Ti_weights_[1] = 0.12;
    Ti_weights_[2] = 0.20;
}

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
float PIDControl::update(float speed_measurement) {
    // Shift histórico de erros e saídas
    y_k_2 = y_k_1;
    y_k_1 = y_k_;
    y_k_ = speed_measurement;

    error_k_2 = error_k_1;
    error_k_1 = error_k_;
    error_k_  = getSetpoint() - y_k_;

    output_k_2 = output_k_1;
    output_k_1 = output_k_;

    /*
    // Controlador antigo - PI discreto projetado
    alpha = Kp_ + Ki_*Ts_/2;
    beta = Ki_*Ts_/2 - Kp_;

    float u = alpha * error_k_ + beta * error_k_1 + output_k_1;

    */

    // Controlador atual - Incremental

    float Kc = Kp_;
    float Ti = Kp_/Ki_;
    float Td = Kc * Kd_;

    float delta_m = Kc * ( (error_k_ - error_k_1) + (1/Ti * error_k_ * Ts_) - Td/Ts_ * (y_k_ - 2*y_k_1 + y_k_2));
    
    ///*
    if(delta_m > 1*max_output_)
    {
        delta_m = 1*max_output_;
    }
    else if (delta_m < 1*min_output_)
    {
        delta_m = 1*min_output_;
    }
    else
    {
        
    }

    //*/
    m_ += delta_m;
    
    // Saturação final (limit output)
    float u_sat = saturate(m_);
    

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

void PIDControl::gainScheduling_weight(float extra_weight)
{
    // Verificação de peso adicional
    if(extra_weight < weights_test_[0] || extra_weight > weights_test_[2])
    {
        setKp(weights_test_[0]);
        setKi(weights_test_[2]);
        return;
    }
        

    uint8_t interval = 1;

    // Caracterização de intervalo da regressão
    if(extra_weight > weights_test_[1])
    {
        interval = 2;
    }

    // Calculo dos ganhos via Gain Scheduling
    float new_Kp =  Kp_weights_[interval-1] + 
                    (extra_weight - weights_test_[interval-1]) / (weights_test_[interval] - weights_test_[interval-1]) 
                    * 
                    (Kp_weights_[interval]-Kp_weights_[interval-1]);
    
    float new_Ti =  Ti_weights_[interval-1] + 
                    (extra_weight - weights_test_[interval-1]) / (weights_test_[interval]-weights_test_[interval-1]) 
                    * 
                    (Ti_weights_[interval]-Ti_weights_[interval-1]);

    setKp(new_Kp);
    setKi(new_Kp/new_Ti);
        
    return;
}
