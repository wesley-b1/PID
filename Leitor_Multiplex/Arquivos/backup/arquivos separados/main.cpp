#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);

void setup()
{
  Serial.begin(115200);
  WiFi.begin("SSID", "PASSWORD");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi conectado!");

  server.on("/", HTTP_GET, []()
            { server.send(200, "text/plain", "OK"); });
  server.begin();
}

void loop()
{
  server.handleClient();
}
