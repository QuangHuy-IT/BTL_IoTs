#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <string.h>

// ==== Cấu hình kênh & radio ====
static const uint8_t ESPNOW_CHANNEL = 10;    
static const int8_t  TX_POWER = 78;         

// ==== PINS ====
#define MQ4_PIN   35   //ADC
#define MP2_PIN   32   //ADC
#define FLAME_PIN 34   //ADC

// ==== MAC của Node 3 (STA MAC) ====
uint8_t coordinatorMac[] = { 0x88, 0x57, 0x21, 0xE0, 0x77, 0x18 }; 

// ==== Giao thức ứng dụng ====
// Gói sensor (có seq)
typedef struct __attribute__((packed)) {
  uint32_t seq;
  uint16_t mq4;
  uint16_t mp2;
  uint16_t flame;
  uint32_t ms;
} SensorPayload;

// Gói ACK từ Node 3 trả về
typedef struct __attribute__((packed)) {
  uint32_t seq;
  uint8_t  ok;
} AckPayload;

// ==== Tham số retry/timeout ====
static const uint8_t  MAX_RETRY       = 5;
static const uint32_t ACK_TIMEOUT_MS  = 120;   // chờ ACK mỗi lần gửi
static const uint32_t SEND_PERIOD_MS  = 1000;  // tần suất gửi dữ liệu

// ==== Biến toàn cục ====
esp_now_peer_info_t peer{};
volatile bool ackReceived = false;
volatile uint32_t ackSeq  = 0;
uint32_t seqCounter = 0;
uint32_t lastSendMs = 0;

 void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // chỉ log trạng thái TX của lớp MAC (không phải ACK ứng dụng)
  // Serial.printf("[Sensor] TX status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void onRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(AckPayload)) {
    AckPayload ack;
    memcpy(&ack, incomingData, sizeof(ack));
    if (ack.ok == 1) {
      ackSeq = ack.seq;
      ackReceived = true;
    }
  }
}

bool sendWithAck(const SensorPayload &p) {
  // Gửi gói và chờ ACK đúng seq trong thời gian ACK_TIMEOUT_MS, lặp lại tối đa MAX_RETRY
  for (uint8_t attempt = 0; attempt <= MAX_RETRY; ++attempt) {
    ackReceived = false;

    esp_err_t err = esp_now_send(coordinatorMac, (uint8_t*)&p, sizeof(p));
    if (err != ESP_OK) {
      Serial.printf("[Sensor] esp_now_send err=%d (attempt %u)\n", err, attempt);
    }

    uint32_t t0 = millis();
    while (millis() - t0 < ACK_TIMEOUT_MS) {
      if (ackReceived && ackSeq == p.seq) {
        // Nhận đúng ACK cho seq này
        return true;
      }
      delay(1); 
    }
    // Hết timeout -> retry
  }
  return false;
}

void addOrUpdatePeer() {
  esp_now_del_peer(coordinatorMac);
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, coordinatorMac, 6);
  peer.channel = ESPNOW_CHANNEL;         
  peer.encrypt = false;
  peer.ifidx   = WIFI_IF_STA;            // rõ ràng interface STA
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[Sensor] Failed to add peer!");
    while (1) delay(1000);
  }
}

void setup() {
  pinMode(FLAME_PIN, INPUT);
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(TX_POWER);   // tăng công suất phát
  

  // Ép STA về kênh khởi tạo
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.printf("[Sensor] MAC=%s | Forced CH=%d\n",
                WiFi.macAddress().c_str(), ESPNOW_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[Sensor] ESP-NOW init failed!");
    while (1) delay(1000);
  }

  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv); // để nhận ACK

  addOrUpdatePeer();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
}

void loop() {
  uint32_t now = millis();
  if (now - lastSendMs < SEND_PERIOD_MS) {
    delay(1);
    return;
  }
  lastSendMs = now;

  SensorPayload p;
  p.seq   = ++seqCounter;
  p.mq4   = analogRead(MQ4_PIN);
  p.mp2   = analogRead(MP2_PIN);
  p.flame = (digitalRead(FLAME_PIN) == LOW);
  p.ms    = now;

  bool ok = sendWithAck(p);
  Serial.printf("[Sensor] seq=%lu | MQ4=%u | MP2=%u | FLAME=%u | send=%s\n",
                (unsigned long)p.seq, p.mq4, p.mp2, p.flame ? 1 : 0, ok ? "OK" : "RETRY_FAIL");
}
