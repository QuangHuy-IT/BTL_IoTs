#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <string.h>

// ==== Cấu hình ESPNOW/Wi-Fi ====
static const uint8_t ESPNOW_CHANNEL = 10;   // đồng bộ với AP của bạn
static const int8_t  TX_POWER = 78;         // ~19.5 dBm

// ==== WiFi & MQTT (ThingsBoard) ====
const char* WIFI_SSID = "401 new";
const char* WIFI_PASS = "88969696";
const char* TB_HOST   = "demo.thingsboard.io"; 
const uint16_t TB_PORT = 1883;
const char* TB_TOKEN = "Fm9NAZ9CwuMvB4CvsOiS";

WiFiClient   net;
PubSubClient mqtt(net);

// ==== MAC peers ====
// MAC Node 1: 44:1D:64:F7:4A:D8
uint8_t sensorMac[]   = { 0x44, 0x1D, 0x64, 0xF7, 0x4A, 0xD8 };
// MAC Node 2: 00:4B:12:EE:D5:30
uint8_t actuatorMac[] = { 0x00, 0x4B, 0x12, 0xEE, 0xD5, 0x30 };

// ==== Payloads ====
typedef struct __attribute__((packed)) {
  uint32_t seq;
  uint16_t mq4;
  uint16_t mp2;
  uint16_t flame;
  uint32_t ms;
} SensorPayload;

typedef struct __attribute__((packed)) {
  uint32_t seq;
  uint8_t  ok;  // 1 = nhận OK
} AckPayload;

typedef struct __attribute__((packed)) {
  uint8_t  level;    // 0=OFF, 1=Alert, 2=Emergency
  uint16_t ms_on;
  uint16_t ms_off;
  uint16_t cycles;
} ActCmd;

// ==== Ngưỡng ====
const uint16_t TH_MQ4_ALERT       = 1900;
const uint16_t TH_MQ4_EMERGENCY   = 2100;
const uint16_t TH_MP2_ALERT       = 2500;
const uint16_t TH_MP2_EMERGENCY   = 3800;
//const uint16_t TH_FLAME_ALERT     = 1000;
const uint16_t TH_FLAME_EMERGENCY = 1;

// ==== State ====
volatile bool   g_newData = false;
SensorPayload   g_last{};
uint32_t        g_recvCount = 0;
uint32_t        g_lastRecvMs = 0;

// ==== Peers ====
esp_now_peer_info_t peerSensor{};
esp_now_peer_info_t peerActuator{};

// ==== Utils ====
String macToString(const uint8_t* mac) {
  char buf[18];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

void printSensorPayload(const SensorPayload& p, const char* prefix = "") {
  Serial.printf("%sseq=%lu | MQ4=%u | MP2=%u | FLAME=%u | ms=%lu\n",
                prefix, (unsigned long)p.seq, p.mq4, p.mp2, p.flame, (unsigned long)p.ms);
}

// ==== MQTT ====
void ensureMqtt() {
  if (mqtt.connected()) return;
  while (!mqtt.connected()) {
    Serial.print("[MQTT] Connecting...");
    if (mqtt.connect("esp32-coordinator", TB_TOKEN, nullptr)) {
      Serial.println("connected.");
    } else {
      Serial.printf("failed rc=%d, retry in 2s\n", mqtt.state());
      delay(2000);
    }
  }
}

void publishTelemetry(const SensorPayload& p, const char* statusStr, uint8_t level) {
  ensureMqtt();
  String payload = "{";
  payload += "\"seq\":"   + String(p.seq)   + ",";
  payload += "\"mq4\":"   + String(p.mq4)   + ",";
  payload += "\"mp2\":"   + String(p.mp2)   + ",";
  payload += "\"flame\":" + String(p.flame) + ",";
  payload += "\"status\":\"" + String(statusStr) + "\",";
  payload += "\"level\":" + String(level);
  payload += "}";
  mqtt.publish("v1/devices/me/telemetry", payload.c_str());
}

// ==== Quyết định cấp độ ====
uint8_t decideLevel(const SensorPayload& p) {
  bool anyEmergency = (p.mq4 >= TH_MQ4_EMERGENCY) ||
                      (p.mp2 >= TH_MP2_EMERGENCY) ||
                      (p.flame >= TH_FLAME_EMERGENCY);
  bool anyAlert = (p.mq4 >= TH_MQ4_ALERT) ||
                  (p.mp2 >= TH_MP2_ALERT);
                  
  if (anyEmergency) return 2;
  if (anyAlert)     return 1;
  return 0;
}

// ==== Gửi lệnh Actuator ====
void sendActuatorCmd(uint8_t level) {
  ActCmd cmd{};
  cmd.level = level;
  if (level == 0) {
    cmd.ms_on = cmd.ms_off = cmd.cycles = 0;
  } else if (level == 1) {
    cmd.ms_on = 200; cmd.ms_off = 800; cmd.cycles = 5;
  } else {
    cmd.ms_on = 200; cmd.ms_off = 200; cmd.cycles = 20;
  }
  esp_now_send(actuatorMac, (uint8_t*)&cmd, sizeof(cmd));
}

// ==== Trả ACK về Node 1 ====
void sendAckToSensor(uint32_t seq) {
  AckPayload ack{seq, 1};
  esp_now_send(sensorMac, (uint8_t*)&ack, sizeof(ack));
}

// ==== Callbacks ====
void onRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  // Chỉ xử lý gói từ sensor
  if (memcmp(mac, sensorMac, 6) == 0 && len == sizeof(SensorPayload)) {
    memcpy(&g_last, incomingData, sizeof(SensorPayload));
    g_newData = true;
    g_recvCount++;
    g_lastRecvMs = millis();

    // In ngay khi nhận + trả ACK
    Serial.printf("\n[RX] from %s | len=%d | t=%lu ms\n",
                  macToString(mac).c_str(), len, (unsigned long)g_lastRecvMs);
    printSensorPayload(g_last, "  ");
    sendAckToSensor(g_last.seq); // trả ACK ngay trong callback (nhanh, ngắn)
  }
}

void onSent(const uint8_t* mac, esp_now_send_status_t status) {
  // Log TX đến actuator hoặc khi gửi ACK
  Serial.printf("[TX->%s] %s\n",
                macToString(mac).c_str(),
                status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ==== ESP-NOW (sau khi Wi-Fi đã kết nối) ====
void setupEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed!");
    while (1) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  esp_now_del_peer(sensorMac);
  memset(&peerSensor, 0, sizeof(peerSensor));
  memcpy(peerSensor.peer_addr, sensorMac, 6);
  peerSensor.channel = ESPNOW_CHANNEL;
  peerSensor.encrypt = false;
  peerSensor.ifidx   = WIFI_IF_STA;
  esp_now_add_peer(&peerSensor);

  esp_now_del_peer(actuatorMac);
  memset(&peerActuator, 0, sizeof(peerActuator));
  memcpy(peerActuator.peer_addr, actuatorMac, 6);
  peerActuator.channel = ESPNOW_CHANNEL;
  peerActuator.encrypt = false;
  peerActuator.ifidx   = WIFI_IF_STA;
  esp_now_add_peer(&peerActuator);

  Serial.printf("[ESP-NOW] Peers added on CH=%u\n", ESPNOW_CHANNEL);
}

// ==== Wi-Fi + MQTT (trước ESP-NOW) ====
void setupWiFiMqtt() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(TX_POWER);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(400);
  }
  Serial.printf("\n[WiFi] Connected %s | IP=%s | CH=%d | MAC=%s\n",
                WIFI_SSID, WiFi.localIP().toString().c_str(),
                WiFi.channel(), WiFi.macAddress().c_str());
  if (WiFi.channel() != ESPNOW_CHANNEL) {
    Serial.printf("[WARN] AP CH=%d khác ESPNOW_CH=%u. Hãy để AP ở kênh 10.\n",
                  WiFi.channel(), ESPNOW_CHANNEL);
  }
  mqtt.setServer(TB_HOST, TB_PORT);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // 1) Wi-Fi 
  setupWiFiMqtt();
  setupEspNow();

  Serial.printf("[Coordinator] Ready. WiFiCH=%d | ESPNOW_CH=%u\n",
                WiFi.channel(), ESPNOW_CHANNEL);
}

void loop() {
  if (g_newData) {
    noInterrupts();
    SensorPayload p = g_last;
    g_newData = false;
    interrupts();

    uint8_t level = decideLevel(p);

    // Xác định nguyên nhân để đặt status cụ thể
    bool gasAlert    = (p.mq4 >= TH_MQ4_ALERT);
    bool gasEmer     = (p.mq4 >= TH_MQ4_EMERGENCY);
    bool smokeAlert  = (p.mp2 >= TH_MP2_ALERT);
    bool smokeEmer   = (p.mp2 >= TH_MP2_EMERGENCY);
    bool flameEmer   = (p.flame >= TH_FLAME_EMERGENCY);

    const char* statusStr = "BinhThuong";
    if (flameEmer) {
      statusStr = "BaoChayKhanCap"; // lửa ưu tiên cao nhất
    } else if (gasEmer || gasAlert) {
      statusStr = (level == 1) ? "CanhBaoGas" : "KhanCapGas";
    } else if (smokeEmer || smokeAlert) {
      statusStr = (level == 1) ? "CanhBaoKhoi" : "KhanCapKhoi";
    }

    Serial.printf("[PROC] seq=%lu -> level=%u (%s)\n",
                  (unsigned long)p.seq, level, statusStr);

    sendActuatorCmd(level);
    publishTelemetry(p, statusStr, level);
  }

  if (WiFi.status() == WL_CONNECTED) {
    ensureMqtt();
    mqtt.loop();
  }
  delay(5);
}
