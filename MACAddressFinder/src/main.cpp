#include <Arduino.h>
#include "WiFi.h"

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println("==================================");
  Serial.print("Dia chi MAC cua ESP32 nay la: ");
  Serial.println(WiFi.macAddress());
  Serial.println("==================================");
  Serial.print("WiFi channel: "); Serial.println(WiFi.channel());
}

void loop() {
}