#include "pid.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern QueueHandle_t pvFila;
extern SemaphoreHandle_t pvSemaforo;

static double iTerm = 0, lastY = 0;
double u = 0, y = 0;
double sp = 3072;
static double kp = 0.0, ki = 0.0, kd = 0.0;
static double uMin = 0, uMax = 255;
static unsigned long lastTime = 0;
static const unsigned long sampleTime = 100;

void computeU() {
  unsigned long now = millis();
  int deltaT = now - lastTime;
  float receberPV;

  if (xQueueReceive(pvFila, &receberPV, 0) == pdTRUE) {
    if (xSemaphoreTake(pvSemaforo, portMAX_DELAY)) {
      y = receberPV;
      xSemaphoreGive(pvSemaforo);
    }
  }

  if (deltaT >= sampleTime) {
    double e, dY;
    if (xSemaphoreTake(pvSemaforo, portMAX_DELAY)) {
      e = sp - y;
      dY = y - lastY;
      lastY = y;
      xSemaphoreGive(pvSemaforo);
    }

    iTerm += ki * e;
    iTerm = constrain(iTerm, uMin, uMax);
    u = kp * e + iTerm - kd * dY;
    u = constrain(u, uMin, uMax);
    lastTime = now;
  }
}

void setTunings(double kP, double kI, double kD) {
  double Ts = sampleTime / 1000.0;
  kp = kP;
  ki = kI * Ts;
  kd = kD / Ts;
}

void setControlLimits(double min, double max) {
  uMin = min;
  uMax = max;
  u = constrain(u, uMin, uMax);
  iTerm = constrain(iTerm, uMin, uMax);
}
