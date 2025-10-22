# PID

# 🧠 Documentação do Sistema de Controle PID no ESP32

Este sistema utiliza **FreeRTOS** para executar tarefas e timers em paralelo, controlando plantas didáticas (de 1ª e 2ª ordem) via **ESP32**, com interface **WebSocket/WebServer** para monitoramento em tempo real.  

---

## ⚙️ Estrutura Geral do Sistema

| Componente | Função |
|-------------|--------|
| **FreeRTOS Tasks** | Executam rotinas assíncronas, como leitura serial e controle. |
| **Timers** | Gerenciam a execução periódica de funções (PID, leitura de sensor, MUX, etc.). |
| **WebSocket** | Transmite os dados (PV, MV, tempo) para o gráfico no navegador. |
| **SPIFFS** | Armazena arquivos HTML/CSS/JS do painel web. |
| **Semáforo (pvSemaforo)** | Garante acesso seguro à variável `y` entre tasks. |
| **Fila (pvFila)** | Armazena as leituras de PV (Process Variable) para o PID. |

---

## 🧩 Principais Variáveis Globais

| Variável | Tipo | Descrição |
|-----------|------|------------|
| `y` | `double` | **Variável de processo (PV)** — valor lido do sensor. |
| `sp` | `double` | **Setpoint** — valor de referência desejado. |
| `u` | `double` | **Sinal de controle (MV)** — saída do PID (tensão no DAC). |
| `kp`, `ki`, `kd` | `double` | **Ganhos do PID** proporcional, integral e derivativo. |
| `iTerm` | `double` | **Termo integral acumulado**. |
| `lastY` | `double` | Último valor de `y` (usado na derivada). |
| `lastTime` | `unsigned long` | Marca o instante da última atualização do PID. |
| `sampleTime` | `unsigned long` | Tempo de amostragem (em ms). |
| `uMin`, `uMax` | `double` | Limites mínimos e máximos da saída de controle `u`. |
| `plantaSelecionada` | `int` | Define qual planta física (1 a 6) está sendo controlada. |
| `usarPID` | `bool` | Define se o controle é **PID (true)** ou **degrau (false)**. |

---

## ⚡ Equações do PID Digital (Implementadas)

A função `computeU()` executa o **PID incremental**, conforme as seguintes relações:

### 1️⃣ Cálculo do erro
$$
\[
e(k) = SP - Y(k)
\]
$$

### 2️⃣ Termo integral (acumulado e limitado)

$$
\[
I(k) = I(k-1) + K_i \cdot e(k)
\]
$$

Com saturação entre `uMin` e `uMax`.

### 3️⃣ Termo derivativo (derivada da PV, não do erro)

$$
\[
D(k) = -K_d \cdot \frac{dY}{dt} = -K_d \cdot (Y(k) - Y(k-1))
\]
$$

> A derivada é feita sobre `y` (PV), não sobre `e`, para reduzir ruído.

### 4️⃣ Saída total

$$
\[
u(k) = K_p \cdot e(k) + I(k) + D(k)
\]
$$

e então:

$$
\[
u(k) = \text{clamp}(u(k), uMin, uMax)
\]
$$

---

## 🧮 Funções do PID

### `computeU()`
Calcula a nova saída `u` com base no erro atual.  
Executada periodicamente pelo **timer `getSensorReadingTimer`**.

**Passos:**
1. Recebe a PV da fila (`pvFila`).
2. Calcula o erro `e = sp - y`.
3. Atualiza o termo integral `iTerm`.
4. Calcula o termo derivativo com base em `Δy`.
5. Soma todos os termos para obter `u`.
6. Limita `u` dentro dos limites definidos.

---

### `setTunings(double kP, double kI, double kD)`
Ajusta os ganhos do PID levando em conta o **tempo de amostragem** (`sampleTime`).

$$
\[
K_i' = K_i \cdot T_s
\]
\[
K_d' = \frac{K_d}{T_s}
\]
$$

Isso garante consistência entre controladores com diferentes taxas de atualização.

---

### `setControlLimits(double min, double max)`
Define os **limites** para:
- saída de controle `u`
- termo integral `iTerm`

Evita **windup** (acúmulo excessivo do termo integral).

---

## ⏱️ Callbacks e Timers

| Função | Timer | Descrição |
|---------|--------|------------|
| `getSensorReadingCallback()` | `getSensorReadingTimer` | Lê o sensor, atualiza PV, executa `computeU()`, e envia dados via WebSocket. |
| `setStepInputReadingCallback()` | `stepInputStartTimer` | Aplica um **degrau (255)** na saída para testes em malha aberta. |
| `setMUXCallback()` | `muxConfigTimer` | Configura os pinos `A_PIN` e `B_PIN` do **CD4052** para selecionar a planta ativa. |

---

## 🌐 Conexão e Interface Web

| Função | Descrição |
|--------|------------|
| `initWiFi()` | Conecta o ESP32 à rede Wi-Fi definida. |
| `initWebSocket()` | Inicializa o servidor WebSocket na porta 8080. |
| `initSPIFFS()` | Monta o sistema de arquivos SPIFFS (HTML, CSS, JS). |
| `initWebServer()` | Configura as rotas para os arquivos da interface. |
| `onWebSocketEvent()` | Recebe comandos da interface Web (`start`, `stop`, `setKp`, `setKi`, `setKd`, `setSp`, `setPlanta`). |

---

## 🧵 Tasks Principais (FreeRTOS)

| Task | Função | Prioridade |
|------|---------|-------------|
| `serialTask()` | Lê comandos via Serial (`start` / `stop`) e aciona timers. | 1 |
| `iniciarTimersSeNecessario()` | Cria e inicia timers de controle conforme modo (aberto/fechado). | 1 |

---

## 🧰 Funções de Configuração

| Função | Descrição |
|--------|------------|
| `configurarPinos()` | Define os modos de entrada/saída para sensores e DACs. |
| `configurarMUX()` | Inicia o timer de seleção da planta (CD4052). |
| `configurarMalhaFechada()` | Inicia o controle PID e define ganhos padrões se o usuário não os enviou. |
| `configurarMalhaAberta()` | Prepara o sistema para teste em degrau (sem PID). |
| `pararTimers()` | Para e deleta todos os timers e reseta o estado do sistema. |

---

## 📊 Sinal Enviado ao Navegador (via WebSocket)

```json
{
  "time": "12.3",
  "MV": "1.85",
  "PV": "2.10"
}
```
- time: tempo decorrido em segundos.

- MV: valor da saída (em volts).

- PV: valor da variável de processo (em volts).

---
## 🧾 Resumo do Controle PID Implementado
$$
u(k) = K_p \cdot e(k) + I(k-1) + K_i \cdot e(k) - K_d \cdot (Y(k) - Y(k-1))
$$

Com

$$
I(k) = \text{clamp}\big(I(k-1) + K_i \cdot e(k),\, u_{\min},\, u_{\max}\big)
$$

--- 

## 🧭 Resumo Final

Este sistema implementa um controlador PID digital completo em ambiente FreeRTOS, com:

- Controle em malha aberta e malha fechada;

- Comunicação Wi-Fi e WebSocket;

- Multiplexação de plantas via CD4052;

- Timers e Tasks independentes para leitura, controle e interface.

Com isso, é possível estudar e aplicar técnicas reais de controle digital em hardware embarcado de forma prática e didática.
