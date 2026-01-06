# Controle Robusto de Velocidade de um Robô Móvel com Controlador H∞

## 📋 Resumo Executivo

Este repositório contém a implementação de um **controlador H-infinito (H∞)** para o controle independente de velocidade em cada uma das quatro rodas de uma plataforma móvel. O projeto integra:

- **Firmware embarcado**: Código em C++ para Raspberry Pi Pico
- **Modelagem e síntese de controladores**: Scripts em MATLAB para modelagem dinâmica dos motores e síntese de controladores robustos
- **Validação experimental**: Dados de testes e análise de desempenho

> **Trabalho de Conclusão de Curso** | Engenharia Elétrica | Universidade Federal de São Carlos | 2026

---

## 👤 Autores

- **João Carlos Tonon Campi** — Graduando em Engenharia Elétrica
- **Roberto Santos Inoue:** — Orientação acadêmica

**Contato:** [campijoao@estudante.ufscar.br]

---

## 🎯 Objetivos

- Adquirir as funções de transferência dos motores DC
- Sintetizar controladores H∞ para garantir robustez contra incertezas paramétricas e perturbações externas
- Implementar o controlador em tempo real no Raspberry Pi Pico
- Validar experimentalmente o desempenho do sistema em diferentes cenários operacionais

---

## 🏗️ Arquitetura do Sistema

### Hardware

| Componente | Especificação |
|-----------|--------------|
| **Microcontrolador** | Raspberry Pi Pico (RP2040, 125 MHz) |
| **Motores DC** | 4 unidades, [torque nominal], [rpm máximo], 30:1 |
| **Drivers de Potência** | 2x Ponte-H [modelo/especificação] |
| **Sensores** | Encoders incrementais [ppr], interface SPI |
| **Fonte de Alimentação** | Bateria Li-on 3S 12V 2200mAh |

### Arquitetura em Camadas

TO-DO

<!-- 

```
┌─────────────────────────────────────┐
│  Interface de Alto Nível (UART)     │  Comunicação e setpoint
├─────────────────────────────────────┤
│  Camada de Controle (Loop PID/H∞)   │  Processamento 50-100 Hz
├─────────────────────────────────────┤
│  Camada de I/O (PWM, Encoder)       │  Acionamento e leitura
├─────────────────────────────────────┤
│  Hardware (RP2040)                  │  Execução em tempo real
└─────────────────────────────────────┘
```

-->

---

## 📁 Estrutura do Repositório

TO-DO

<!-- 

```
repo-root/
│
├── README.md                          # Este arquivo
├── LICENSE                            # [Apache 2.0 / MIT]
│
├── firmware/                          # Código do Raspberry Pi Pico
│   ├── main.py                        # Inicialização e loop principal
│   ├── controller.py                  # Implementação do controlador H∞
│   ├── motor_driver.py                # Interface com pontes-H (PWM)
│   ├── encoder.py                     # Leitura e tratamento de encoder
│   ├── communication.py               # Interface UART
│   ├── config.py                      # Parâmetros e ganhos do controlador
│   └── requirements.txt               # Dependências (thonny, etc)
│
├── matlab/                            # Scripts de modelagem e síntese
│   ├── system_identification/
│   │   ├── motor_step_response.m      # Análise resposta ao degrau
│   │   ├── bode_plot.m                # Diagrama de Bode
│   │   └── system_model.m             # Função de transferência do motor
│   │
│   ├── controller_synthesis/
│   │   ├── hinf_controller_design.m   # Síntese do controlador H∞
│   │   ├── robustness_analysis.m      # Análise de estabilidade robusta
│   │   ├── uncertainty_model.m        # Modelo de incerteza paramétrica
│   │   └── controller_validation.m    # Validação em simulação
│   │
│   └── analysis/
│       ├── closed_loop_simulation.m   # Simulação da malha fechada
│       ├── step_response_analysis.m   # Análise de performance
│       └── generate_code.m            # Exportar ganhos para firmware
│
├── data/                              # Dados experimentais
│   ├── raw/
│   │   ├── motor_1_step_test.txt      # Dados brutos do ensaio
│   │   ├── motor_2_step_test.txt
│   │   ├── motor_3_step_test.txt
│   │   └── motor_4_step_test.txt
│   │
│   └── processed/
│       ├── motor_identification.mat   # Parâmetros identificados
│       └── controller_gains.mat       # Ganhos sintetizados
│
├── documentation/
│   ├── system_model.pdf               # Modelagem matemática
│   ├── controller_design.pdf          # Projeto do controlador
│   ├── experimental_protocol.pdf      # Protocolo de testes
│   └── results_summary.pdf            # Resumo dos resultados
│
└── .gitignore                         # Arquivos a ignorar
```

---

-->

## 🚀 Como Começar

### Pré-requisitos

TO-DO

<!-- 

**Hardware:**
- Raspberry Pi Pico
- 2x Ponte-H [especificação]
- 4x Motores DC [especificação]
- 4x Encoders incrementais
- Fonte de alimentação [especificação]
- Cabo USB-micro ou depurador SWD

**Software:**
- MATLAB R2021b+ (Control Systems Toolbox, Robust Control Toolbox)
- Thonny IDE ou rshell para programação do Pico
- Python 3.8+ (para scripts auxiliares)
- Git

-->

### Instalação do Firmware

TO-DO

<!-- 

1. **Preparar ambiente:**
```bash
git clone <repo-url>
cd repo-root/firmware
```

2. **Carregar MicroPython no Pico:**
   - Baixar MicroPython para RP2040 em [micropython.org](https://www.micropython.org)
   - Conectar Pico em modo bootloader (pressionando BOOTSEL)
   - Copiar arquivo `.uf2` para a unidade de armazenamento

3. **Transferir código:**
   - Usar Thonny IDE para copiar arquivo `main.py` e dependências para o Pico
   - Ou usar `rshell`:
```bash
rshell -p /dev/ttyACM0 cp firmware/* /pyboard/
```

4. **Configurar parâmetros:**
   - Editar `config.py` com os ganhos sintetizados em MATLAB
   - Ajustar limites de PWM, frequência de controle, etc.

-->

<!-- 


### Síntese do Controlador em MATLAB

1. **Identificação do sistema:**
```matlab
% Em MATLAB
cd matlab/system_identification
motor_step_response  % Executar ensaio simulado ou carregar dados reais
system_model         % Gerar função de transferência
```

2. **Design do controlador H∞:**
```matlab
cd ../controller_synthesis
hinf_controller_design  % Sintetizar ganhos
robustness_analysis     % Validar estabilidade e performance
```

3. **Exportar ganhos:**
```matlab
generate_code  % Gera arquivo de configuração para o firmware
```

---
-->

## 🧪 Execução

TO-DO

<!-- 

### Teste de Resposta ao Degrau

1. Conectar Pico via UART (115200 baud) ao computador
2. Usar script de teste:
```bash
python test_scripts/step_response_test.py --motor 1 --amplitude 255 --duration 10
```
3. Dados são salvos automaticamente em `data/raw/`

### Teste de Rastreamento de Setpoint

1. Enviar referência de velocidade via serial:
```
SET_SPEED 1 100  # Motor 1, 100 rpm
```
2. Monitorar resposta em tempo real
3. Salvar log para análise offline

### Validação de Robustez

Executar em MATLAB:
```matlab
cd matlab/analysis
closed_loop_simulation  % Simular com variações de parâmetros
step_response_analysis  % Analisar métricas de performance
```

---
-->

## 📊 Resultados Esperados

### Métricas de Performance

TO-DO

<!-- 

| Métrica | Especificação | Realizado |
|---------|---------------|-----------|
| Tempo de acomodação (2%) | < 500 ms | — |
| Sobressinal | < 10% | — |
| Erro em regime permanente | < 2% | — |
| Banda passante do sistema | > 5 Hz | — |
| Margem de fase | > 45° | — |

-->

### Análise Robusta


TO-DO

<!-- 

- **Margem de ganho**: Verificar estabilidade com variação ±30% nos parâmetros do motor
- **Incerteza multiplicativa**: Modelo de incerteza estruturada e não-estruturada
- **Perturbações**: Validar rejeição de perturbações de torque e atrito

---

-->

<!-- 

## 📝 Modelagem Matemática

### Função de Transferência do Motor

O motor DC é modelado como um sistema de segunda ordem:

$$G_m(s) = \frac{K}{(J s + B)(L_a s + R_a) + K^2}$$

Onde:
- **K**: Constante de torque (Nm/A)
- **J**: Inércia rotacional (kg·m²)
- **B**: Coeficiente de fricção viscosa (Nm·s/rad)
- **L_a**: Indutância da armadura (H)
- **R_a**: Resistência da armadura (Ω)

### Controlador H∞

Sintetizado via problema de otimização robusta:

$$\min_K \|T_{zw}(K)\|_\infty$$

Onde **T_zw** é a matriz de transferência entre perturbações e saídas controladas.

---

-->

## 🔗 Dependências

TODO

<!-- 

### Firmware (MicroPython)
- MicroPython 1.20+
- Bibliotecas nativas: `machine`, `time`, `gc`

### MATLAB
- Control Systems Toolbox
- Robust Control Toolbox (para síntese H∞)
- System Identification Toolbox (opcional)

### Documentação e Suporte
- [MicroPython Docs](https://docs.micropython.org)
- [Control Systems Toolbox Guide](https://www.mathworks.com/products/control.html)
- [H∞ Control Theory](https://en.wikipedia.org/wiki/H%E2%88%9E_control)

---

-->

<!-- 

## 📄 Licença

Este projeto está licenciado sob a licença [MIT / Apache 2.0 / CC-BY-4.0].  
Veja arquivo `LICENSE` para detalhes.

---

-->


## 📚 Referências Principais

TO-DO

<!-- 
[1] Zhou, K., Doyle, J. C., & Glover, K. (1996). *Robust and optimal control*. Prentice Hall.

[2] Ogata, K. (2010). *Modern control engineering* (5th ed.). Pearson.

[3] Åström, K. J., & Murray, R. M. (2010). *Feedback systems: An introduction for scientists and engineers*. Princeton University Press.

[4] [Datasheet Raspberry Pi Pico]

[5] [Datasheet Motor DC]
-->

---

## 🗂️ Histórico de Atualizações

| Versão | Data | Alterações |
|--------|------|-----------|
| 1.0 | [27/08/2025] | Versão inicial com atuação dos motores|
| 1.1 | [30/10/2025] | Leitura de Encoder |
| — | — | — |

---

**Status do projeto:** Em desenvolvimento
