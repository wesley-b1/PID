#pragma once
#include <cstddef>
#include <stdint.h>
#include <WebSocketsServer.h>

void configurarPinos();
void iniciarSerialTask();
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
