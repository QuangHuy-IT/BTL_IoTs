
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <string.h>

// ========= CONFIG =========
static const uint8_t ESPNOW_CHANNEL = 10;   

// ========= PIN ============
#define LED_PIN     4
#define BUZZER_PIN  18

// ==== PWM cho BUZZER ====
#define BUZZER_CH   0
#define BUZZER_FREQ 2000   // 2kHz: thường hợp với buzzer 
#define BUZZER_RES  8      // 8-bit duty
inline void buzzerOn(uint8_t duty = 128) { ledcWrite(BUZZER_CH, duty); } // ~50%
inline void buzzerOff() { ledcWrite(BUZZER_CH, 0); }

// ========= STRUCT =========
typedef struct __attribute__((packed)) {
  uint8_t  level;   // 0 = OFF, 1 = Cảnh báo, 2 = Khẩn cấp
  uint16_t ms_on;
  uint16_t ms_off;
  uint16_t cycles;
} ActCmd;

// ========= STATE ==========
volatile bool cmdReady = false; 
ActCmd g_cmd;                   

// In MAC dạng chuỗi
String macToString(const uint8_t* mac) {
  char buf[18];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

// Nhận lệnh
void onRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(ActCmd)) {
    // copy nhanh vào g_cmd rồi bật cờ
    memcpy((void*)&g_cmd, incomingData, sizeof(g_cmd));
    cmdReady = true;
    Serial.printf("[Actuator] RX from %s | level=%u ms_on=%u ms_off=%u cycles=%u\n",
                  macToString(mac).c_str(),
                  g_cmd.level, g_cmd.ms_on, g_cmd.ms_off, g_cmd.cycles);
  } else {
    Serial.printf("[Actuator] RX unknown size=%d\n", len);
  }
}

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.printf("[Actuator] TX->%s: %s\n",
                macToString(mac).c_str(),
                status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void doPattern(uint16_t onMs, uint16_t offMs, uint16_t cycles, bool buzzer)
{
  for (uint16_t i = 0; i < cycles; i++) {
    digitalWrite(LED_PIN, HIGH);
    if (buzzer) buzzerOn();          // PWM kêu to, rõ
    delay(onMs);

    digitalWrite(LED_PIN, LOW);
    if (buzzer) buzzerOff();         // tắt giữa các nhịp
    delay(offMs);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  
  ledcSetup(BUZZER_CH, BUZZER_FREQ, BUZZER_RES);
  ledcAttachPin(BUZZER_PIN, BUZZER_CH);
  buzzerOff();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Ép kênh
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.printf("[Actuator] MAC=%s | CH=%d\n",
                WiFi.macAddress().c_str(), WiFi.channel());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[Actuator] ESP-NOW init failed!");
    while (1) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  Serial.println("[Actuator] Ready!");
}

void loop() {
  if (cmdReady) {
    noInterrupts();
    ActCmd cmd = g_cmd;   
    cmdReady = false;
    interrupts();

    Serial.printf("[Actuator] Execute level=%u\n", cmd.level);

    if (cmd.level == 0) {
      digitalWrite(LED_PIN, LOW);
      buzzerOff();                                  // đảm bảo tắt hẳn
    } else if (cmd.level == 1) {
      // Cảnh báo nhẹ: LED nháy theo nhịp, CÒI KÊU THEO NHỊP
      doPattern(cmd.ms_on, cmd.ms_off, cmd.cycles, true);
    } else if (cmd.level == 2) {
      // Khẩn cấp: LED nháy theo nhịp, CÒI KÊU LIÊN TỤC (PWM)
      buzzerOn();                                  // bật liên tục
      doPattern(cmd.ms_on, cmd.ms_off, cmd.cycles, false);  // chỉ nháy LED
      // KHÔNG buzzerOff() ở đây -> sẽ kêu liên tục đến khi nhận level=0
    }
  }
  delay(5);
}
