#include "controle.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <WebSocketsServer.h>
#include "webserver.h"
#include "timers.h"
#include "pid.h"

#define MV_PIN_1 25
#define MV_PIN_2 26
#define SENSOR_X 35
#define SENSOR_Y 34
#define A_PIN 14
#define B_PIN 27

QueueHandle_t pvFila;
SemaphoreHandle_t pvSemaforo;
TaskHandle_t serialTaskHandle = NULL;
TaskHandle_t iniciarTimersTaskHandle = NULL;

extern bool iniciarExperimento;
extern bool experimentoIniciado;
extern int plantaSelecionada;
extern bool ganhosFornecidosViaWeb;

void configurarPinos() {
  pinMode(MV_PIN_1, OUTPUT);
  pinMode(MV_PIN_2, OUTPUT);
  pinMode(SENSOR_X, INPUT);
  pinMode(SENSOR_Y, INPUT);
  pinMode(A_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);

  pvFila = xQueueCreate(10, sizeof(float));
  pvSemaforo = xSemaphoreCreateMutex();
}

void iniciarSerialTask() {
  xTaskCreate([](void *) {
    while (true) {
      if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toLowerCase();

        if (cmd == "start" && !experimentoIniciado && iniciarTimersTaskHandle == NULL) {
          iniciarExperimento = true;
          xTaskCreate(iniciarTimersSeNecessario, "iniciarTimersTask", 4096, NULL, 1, &iniciarTimersTaskHandle);
        } else if (cmd == "stop") {
          pararTimers();
        }
      }
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }, "serialTask", 4096, NULL, 1, &serialTaskHandle);
}
