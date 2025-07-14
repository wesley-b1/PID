// #include "webserver.h"
#include <WebServer.h>
#include "controle.h"
#include "SPIFFS.h"

const char *ssid = "brisa-1401761";
const char *password = "8hwzipmn";

WebSocketsServer websocketserver = WebSocketsServer(8080);
WebServer webserver(80);

void initWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("Conectando ao WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();
  Serial.println("WiFi conectado. IP: ");
  Serial.println(WiFi.localIP());
}

void initSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("Erro ao montar SPIFFS");
  }
  else
  {
    Serial.println("SPIFFS montado com sucesso");
  }
}

void initWebSocket()
{
  websocketserver.begin();
  websocketserver.onEvent(onWebSocketEvent);
}

void initWebServer()
{
  webserver.serveStatic("/", SPIFFS, "/index.html");
  webserver.serveStatic("/style.css", SPIFFS, "/style.css");
  webserver.serveStatic("/script.js", SPIFFS, "/script.js");
  webserver.begin();
}
