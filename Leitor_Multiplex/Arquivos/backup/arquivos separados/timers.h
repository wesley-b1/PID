#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

extern TimerHandle_t muxConfigTimer;
extern TimerHandle_t getSensorReadingTimer;
extern TimerHandle_t stepInputStartTimer;

extern bool iniciarExperimento;
extern bool experimentoIniciado;
extern bool ganhosFornecidosViaWeb;
extern int plantaSelecionada;
extern const bool usarPID;

void configurarMUX();
void configurarMalhaFechada();
void configurarMalhaAberta();
void iniciarTimersSeNecessario(void *pvParameters);
void pararTimers();
