#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// ========== THAY ĐỔI THÔNG SỐ ==========
// TODO: Dinh nghia cac chan GPIO thiet bi ngoai vi
#define LED_PIN     25
#define BUZZER_PIN  26
#define RELAY_PIN   27

// Dinh nghia cau truc data nhan duoc
typedef struct struct_command {
    bool led_state;
    bool buzzer_state;
    bool relay_state;
} struct_command;

// Tao mot bien struct
struct_command commandData;

// Callback function khi nhan duoc data
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&commandData, incomingData, sizeof(commandData));
  Serial.println("Da nhan duoc lenh tu Coordinator!");

  // Thuc thi lenh
  Serial.printf("LED: %s | Coi: %s | Relay: %s\n",
                commandData.led_state ? "BAT" : "TAT",
                commandData.buzzer_state ? "BAT" : "TAT",
                commandData.relay_state ? "BAT" : "TAT");
  
  digitalWrite(LED_PIN, commandData.led_state ? HIGH : LOW);
  digitalWrite(BUZZER_PIN, commandData.buzzer_state ? HIGH : LOW);
  digitalWrite(RELAY_PIN, commandData.relay_state ? HIGH : LOW);
  // Luu y: Mot so module relay kich hoat o muc LOW
  // Neu vay, hay dao nguoc logic: commandData.relay_state ? LOW : HIGH
}
 
void setup() {
  Serial.begin(115200);
  
  // Dinh nghia chan OUTPUT
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  // Dat trang thai ban dau la TAT
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  
  // Khoi tao ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Loi khoi tao ESP-NOW");
    return;
  }
  
  // Dang ky callback khi nhan
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Node Actuator san sang nhan lenh...");
}
 
void loop() {
  // De trong, vi node nay chi phan ung khi co su kien (event-driven)
  delay(1000);
}