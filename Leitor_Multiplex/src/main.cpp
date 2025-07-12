#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// === CONFIGURAÇÕES ===
static const TickType_t samplingInterval = 100 / portTICK_PERIOD_MS;
static const TickType_t timeToStartInterval = 5000 / portTICK_PERIOD_MS;
static const uint16_t RESOLUTION = 4096;
// true para (PID) ou false (Degrau)
// true malha fechada e false para malha aberta
static const bool usarPID = true;

// Pinos do MUX (CD4052)
static const uint8_t A_PIN = 14;
static const uint8_t B_PIN = 27;

// Entradas analógicas
static const uint8_t SENSOR_X = 35; // plantas 1 a 4
static const uint8_t SENSOR_Y = 34; // plantas 5 e 6

// Saídas para atuar a planta
static const uint8_t MV_PIN_1 = 25;
static const uint8_t MV_PIN_2 = 26;

// === VARIÁVEIS GLOBAIS ===
volatile float VCC = 3.3;
volatile uint16_t sensorReadingInt;
volatile float sensorReadingVoltage;
// variáveis de controle
bool iniciarExperimento = false;
bool experimentoIniciado = false;

// Task
TaskHandle_t serialTaskHandle = NULL;
TaskHandle_t iniciarTimersTaskHandle = NULL;

// Seletor de planta (1 a 6)
// 1° ordem [1 ao 4]
// 2° ordem [5 ao 6]
int plantaSelecionada = 4;

// PID
unsigned long lastTime = 0;
double u = 0, y = 0;
double sp = 0.75 * RESOLUTION; //  0.75 * 4096 = 3072
double iTerm = 0, lastY = 0;
double kp = 0.0, ki = 0.0, kd = 0.0; // inicialização
double uMin = 0, uMax = 255;
unsigned long sampleTime = samplingInterval * portTICK_PERIOD_MS;

// === TIMERS ===
static TimerHandle_t muxConfigTimer = NULL;
static TimerHandle_t getSensorReadingTimer = NULL;
static TimerHandle_t stepInputStartTimer = NULL;

// === FUNÇÕES PID ===
void computeU();
void setTunings(double kP, double kI, double kD);
void setControlLimits(double min, double max);

// === CALLBACKS ===
void getSensorReadingCallback(TimerHandle_t xTimer);
void setStepInputReadingCallback(TimerHandle_t xTimer);
void setMUXCallback(TimerHandle_t xTimer);

void configurarPinos();
void configurarMUX();
void configurarMalhaFechada();
void configurarMalhaAberta();

// parar todos os timers e resetar sistema:
void pararTimers();
// Task para ler a serial:
void serialTask(void *pvParameters);
// Função para iniciar os timers quando autorizado:
void iniciarTimersSeNecessario(void *pvParameters);

void setup()
{
  configurarPinos();
  Serial.begin(115200);
  Serial.println("Digite 'start' para iniciar o experimento.");

  // Criação das tasks
  xTaskCreate(serialTask, "serialTask", 4096, NULL, 1, &serialTaskHandle);

  // iniciarTimerSeNecessario precisa ser NULL na serialTask
  // xTaskCreate(iniciarTimersSeNecessario, "iniciarTimersTask", 4096, NULL, 1, &iniciarTimersTaskHandle);
}

void loop()
{
  vTaskSuspend(NULL); // Tudo controlado por timers
}

// === FUNÇÕES PID ===
void computeU()
{
  unsigned long now = millis();
  int timeChange = (now - lastTime);
  if (timeChange >= sampleTime)
  {
    double e = sp - y;
    iTerm += (ki * e);
    if (iTerm > uMax)
      iTerm = uMax;
    else if (iTerm < uMin)
      iTerm = uMin;
    double dY = (y - lastY);
    u = kp * e + iTerm - kd * dY;
    if (u > uMax)
      u = uMax;
    else if (u < uMin)
      u = uMin;
    lastY = y;
    lastTime = now;
  }
}

void setTunings(double kP, double kI, double kD)
{
  double sampleTimeInSec = ((double)sampleTime / 1000);
  kp = kP;
  ki = kI * sampleTimeInSec;
  kd = kD / sampleTimeInSec;
}

void setControlLimits(double min, double max)
{
  if (min > max)
    return;
  uMin = min;
  uMax = max;
  if (u > uMax)
    u = uMax;
  else if (u < uMin)
    u = uMin;
  if (iTerm > uMax)
    iTerm = uMax;
  else if (iTerm < uMin)
    iTerm = uMin;
}

// === CALLBACKS ===
void getSensorReadingCallback(TimerHandle_t xTimer)
{
  if (plantaSelecionada >= 1 && plantaSelecionada <= 4)
    sensorReadingInt = analogRead(SENSOR_X);
  else if (plantaSelecionada == 5 || plantaSelecionada == 6)
    sensorReadingInt = analogRead(SENSOR_Y);

  y = sensorReadingInt;
  sensorReadingVoltage = (VCC * y) / RESOLUTION;

  if (usarPID)
  {
    computeU();
    if (plantaSelecionada >= 1 && plantaSelecionada <= 4)
    {
      // valor digital máximo aceito pelo DAC é de 2^8 – 1 = 255
      // significa que haverá 255 níveis entre o 0V e a tensão de alimentação na saída do DAC
      // resolução de 8 bits
      dacWrite(MV_PIN_1, (int)u);
    }
    else if (plantaSelecionada == 5 || plantaSelecionada == 6)
    {
      dacWrite(MV_PIN_2, (int)u);
    }
  }

  Serial.print(xTaskGetTickCount() / 1000.0, 1);
  Serial.print(",");
  Serial.print("P");
  Serial.print(plantaSelecionada);
  Serial.print(",");
  Serial.print(sp * VCC / RESOLUTION);
  Serial.print(",");
  Serial.println(sensorReadingVoltage, 3);
}

void setStepInputReadingCallback(TimerHandle_t xTimer)
{
  if (!usarPID)
  {
    if (plantaSelecionada >= 1 && plantaSelecionada <= 4)
      dacWrite(MV_PIN_1, 255);
    else if (plantaSelecionada >= 5 && plantaSelecionada <= 6)
      dacWrite(MV_PIN_2, 255);
  }
}

void setMUXCallback(TimerHandle_t xTimer)
{
  switch (plantaSelecionada)
  {
  case 1: // Planta 1 -> X0
    // A=0 B=0
    digitalWrite(A_PIN, LOW);
    digitalWrite(B_PIN, LOW);
    break;
  case 2: // Planta 2 -> X1
    // A=1 B=0
    digitalWrite(A_PIN, HIGH);
    digitalWrite(B_PIN, LOW);
    break;
  case 3: // Planta 3 -> X2
    // A=0 B=1
    digitalWrite(A_PIN, LOW);
    digitalWrite(B_PIN, HIGH);
    break;
  case 4: // Planta 4 -> X3
    // A=1 B=1
    digitalWrite(A_PIN, HIGH);
    digitalWrite(B_PIN, HIGH);
    break;
  case 5: // Planta 5 -> Y0
    // A=0 B=0
    digitalWrite(A_PIN, LOW);
    digitalWrite(B_PIN, LOW);
    break;
  case 6: // Planta 6 -> Y1
    // A=1 B=0
    digitalWrite(A_PIN, HIGH);
    digitalWrite(B_PIN, LOW);
    break;
  default:
    Serial.println("Planta inválida.");
    return;
  }
  Serial.print("MUX configurado para Planta ");
  Serial.println(plantaSelecionada);
}

void configurarPinos()
{
  Serial.begin(115200);
  pinMode(MV_PIN_1, OUTPUT);
  pinMode(MV_PIN_2, OUTPUT);
  pinMode(SENSOR_X, INPUT);
  pinMode(SENSOR_Y, INPUT);
  pinMode(A_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);
}

void configurarMUX()
{
  // Timer do MUX (necessário em MA e MF)
  muxConfigTimer = xTimerCreate("muxConfigTimer", pdMS_TO_TICKS(100), pdFALSE, NULL, setMUXCallback);
  xTimerStart(muxConfigTimer, 0);
}

void configurarMalhaFechada()
{
  setControlLimits(0, 255);

  switch (plantaSelecionada)
  {
  case 1:
    setTunings(1.922, 0.602, 0.0);
    break;
  case 2:
    setTunings(2.033, 0.525, 0.0);
    break;
  case 3:
    setTunings(2.054, 0.299, 0.0);
    break;
  case 4:
    setTunings(1.961, 0.241, 0.0);
    break;
  }

  dacWrite(MV_PIN_1, 0);
  dacWrite(MV_PIN_2, 0);
  delay(8000); // Esperar descarga do capacitor

  // ativando time de malha fechada (com PID ativo)
  getSensorReadingTimer = xTimerCreate("getSensorReadingTimer", samplingInterval, pdTRUE, NULL, getSensorReadingCallback);
  xTimerStart(getSensorReadingTimer, 0);
}

void configurarMalhaAberta()
{
  // ativando time de  malha aberta (sem PID)
  stepInputStartTimer = xTimerCreate("stepInputStartTimer", timeToStartInterval, pdFALSE, NULL, setStepInputReadingCallback);
  xTimerStart(stepInputStartTimer, 0);
}

// Função para iniciar os timers quando autorizado:
void iniciarTimersSeNecessario(void *pvParameters)
{
  while (true)
  {
    if (iniciarExperimento && !experimentoIniciado)
    {
      configurarMUX();

      if (usarPID && plantaSelecionada >= 1 && plantaSelecionada <= 4)
        configurarMalhaFechada();

      else if (!usarPID && plantaSelecionada >= 5 && plantaSelecionada <= 6)
        configurarMalhaAberta();

      experimentoIniciado = true;
      Serial.println("Timers iniciados.");
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// parar todos os timers e resetar sistema:
void pararTimers()
{
  // Parando (xTimerStop) e deletando (xTimerDelete) os timers após o STOP
  if (getSensorReadingTimer != NULL)
  {
    xTimerStop(getSensorReadingTimer, 0);
    xTimerDelete(getSensorReadingTimer, 0);
    getSensorReadingTimer = NULL;
  }

  if (stepInputStartTimer != NULL)
  {
    xTimerStop(stepInputStartTimer, 0);
    xTimerDelete(stepInputStartTimer, 0);
    stepInputStartTimer = NULL;
  }

  if (muxConfigTimer != NULL)
  {
    xTimerStop(muxConfigTimer, 0);
    xTimerDelete(muxConfigTimer, 0);
    muxConfigTimer = NULL;
  }
  dacWrite(MV_PIN_1, 0);
  dacWrite(MV_PIN_2, 0);

  iniciarExperimento = false;
  experimentoIniciado = false;

  if (iniciarTimersTaskHandle != NULL)
  {
    vTaskDelete(iniciarTimersTaskHandle);
    iniciarTimersTaskHandle = NULL;
  }
  Serial.println("Experimento parado. Aguardando novo 'start'.");
}

void serialTask(void *pvParameters)
{
  while (true)
  {
    if (Serial.available())
    {
      String comando = Serial.readStringUntil('\n');
      comando.trim();
      comando.toLowerCase();

      if (comando == "start" && !experimentoIniciado && iniciarTimersTaskHandle == NULL)
      {
        Serial.println("Comando recebido: Iniciando experimento...");
        iniciarExperimento = true;
        xTaskCreate(iniciarTimersSeNecessario, "iniciarTimersTask", 4096, NULL, 1, &iniciarTimersTaskHandle);
      }
      else if (comando == "stop")
      {
        Serial.println("Comando recebido: Parando experimento...");
        pararTimers(); // isso agora também precisa deletar a task
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Task para ler a serial:
// void serialTask(void *pvParameters)
//{
//  while (true)
//  {
//    if (Serial.available())
//    {
//      String comando = Serial.readStringUntil('\n');
//      comando.trim();
//      comando.toLowerCase();
//
//      if (comando == "start" && !experimentoIniciado)
//      {
//        Serial.println("Comando recebido: Iniciando experimento...");
//        iniciarExperimento = true;
//      }
//      else if (comando == "stop" && experimentoIniciado)
//      {
//        Serial.println("Comando recebido: Parando experimento...");
//        pararTimers();
//      }
//    }
//
//    vTaskDelay(pdMS_TO_TICKS(50));
//  }
//}