#pragma once
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

extern WebSocketsServer websocketserver;
extern WebServer webserver;

void initWiFi();
void initSPIFFS();
void initWebServer();
void initWebSocket();