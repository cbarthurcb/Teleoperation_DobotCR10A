# Teleoperação do Dobot CR10A — Demos TCC

Repositório de desenvolvimento do TCC: **Teleoperação a distância de robô colaborativo aplicado ao contexto de consultas médicas**.

> **Instituição:** [Universidade Federal de Uberlândia - Biolab]
> **Autor:** [Arthur Coelho Bastos]
> **Orientador:** [Alcimar Barbosa Soares]

---

## Visão Geral do Projeto

O projeto propõe um sistema de **teleoperação remota do robô Dobot CR10A** para aplicação em contextos de consultas médicas, com foco na simulação de **apalpação** — método de exame físico em que o profissional pressiona partes do corpo do paciente com as mãos.

A prova de conceito utiliza uma **célula de carga no efetor** do robô para medir e controlar a força aplicada sobre uma superfície, emulando o toque de uma apalpação.

### Arquitetura do Sistema

O sistema é composto por três camadas:

- **Teleoperação** — Controle remoto do robô via protocolo TCP usando o SDK oficial da Dobot, com suporte a VPN para operação fora da rede local.
- **Sensoriamento** — Célula de carga uniaxial + microcontrolador ESP32, que transmite os dados de força em tempo real via protocolo UDP.
- **Controle** — Controlador PI (Proporcional-Integral) que ajusta automaticamente o movimento do robô para manter a força aplicada estável no setpoint definido pelo operador.

---

## Estrutura do Repositório

```
dobot-cr10a-teleoperation/
│
├── demo1_interface_tcp/
│   ├── main.py                  # Interface gráfica de teleoperação (Tkinter)
│   └── README.md                # Instruções da Demo 1
│
├── demo2_controle_forca/
│   ├── python/
│   │   ├── controle_PI.py       # Controlador PI de força com interface gráfica
│   │   └── README.md            # Instruções do código Python
│   ├── esp32/
│   │   ├── leitura_celula_carga.ino   # Firmware ESP32: leitura e envio UDP
│   │   └── README.md                  # Instruções do ESP32
│   ├── calibracao/
│   │   └── calibracao_celula_carga.xlsx  # Planilha de calibração da célula de carga
│   └── web_monitor/
│       ├── index.html           # Monitor web: gráfico em tempo real dos dados UDP
│       └── README.md
│
└── README.md                    # Este arquivo
```

---

## Demo 1 — Interface de Teleoperação TCP

**Objetivo:** Controle do robo com protocolo de comunicação TCP/IP usando SDK disponibilizado pela fabricante.

### Como funciona

- Explicar funcionamento codigo (usar boas praticas de documentação de readme e codigos

### Como executar

```bash
cd demo1_interface_tcp
pip install -r requirements.txt
python main.py
```

**Pré-requisitos:** Python 3.x, rede conectada ao robô (rede `DobotCR10A-XXXX-XXXX`).

---

## Demo 2 — Controle PI de Força com Célula de Carga

**Objetivo:** Controlar a força exercida pelo efetor do robô usando feedback de uma célula de carga.

### Componentes

| Componente | Descrição |
|---|---|
| Dobot CR10A | Robô colaborativo |
| Célula de carga uniaxial | Sensor de força montado no efetor |
| ESP32 | Microcontrolador para leitura e envio UDP dos dados |
| PC do operador | Executa o controlador PI em Python |

### Fluxo de dados

```
Célula de Carga → ESP32 → UDP (Wi-Fi) → PC → Controlador PI → TCP → Robô
```

### 2a. Firmware ESP32


### 2b. Controlador PI (Python)


```bash
cd demo2_controle_forca/python
pip install -r requirements.txt
python controle_PI.py
```

### 2c. Monitor Web (teste de comunicação UDP)

Página HTML com gráfico em tempo real para validar o recebimento dos dados do ESP32 antes de rodar o controlador.

```bash
cd demo2_controle_forca/web_monitor
# Abrir index.html no navegador
```

---

## Dependências Principais

```
Python >= 3.8
tkinter       (interface gráfica — incluso no Python)
matplotlib    (gráficos)
numpy         (cálculos numéricos)
socket        (comunicação TCP/UDP — incluso no Python)
struct        (desempacotamento binário — incluso no Python)
```
---



---

## Licença

[Definir licença — sugestão: MIT ou CC BY 4.0 para TCC]
