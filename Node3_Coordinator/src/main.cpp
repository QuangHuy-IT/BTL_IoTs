#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> // Thu vien de tao JSON

// ========== 1. THAY ĐỔI THÔNG SỐ ESP-NOW ==========
// TODO: Thay dia chi MAC cua Node 2 (Actuator): 00:4B:12:EE:D5:30
uint8_t actuatorAddress[] = {0x00, 0x4B, 0x12, 0xEE, 0xD5, 0x30};

// ========== 2. THAY ĐỔI THÔNG SỐ WI-FI ==========
// TODO: Thay thong tin Wi-Fi cua ban
const char* WIFI_SSID = "401 new";
const char* WIFI_PASSWORD = "88969696";

// ========== 3. THAY ĐỔI THÔNG SỐ THINGSBOARD (MQTT) ==========
// TODO: Thay thong tin server ThingBoard
const char* TB_SERVER = "demo.thingsboard.io"; // Hoac IP/domain cua ban
int TB_PORT = 1883;
// TODO: Thay ACCESS TOKEN cua Device tren ThingBoard
const char* TB_ACCESS_TOKEN = "Fm9NAZ9CwuMvB4CvsOiS";

// ========== 4. THAY ĐỔI NGƯỠNG CẢNH BÁO ==========
// TODO: Hieu chuan va dat nguong cho cam bien
// Gia tri ADC tho tu 0-4095
#define MQ4_THRESHOLD_GAS   2250  // Nguong bao dong GAS
#define MP2_THRESHOLD_SMOKE 1200  // Nguong bao dong KHOI

// =========================================================

// Dinh nghia cau truc data NHAN (tu Node 1)
typedef struct struct_message_sensor {
    int mq4_value;
    int mp2_value;
    bool flame_detected;
} struct_message_sensor;
struct_message_sensor sensorData;

// Dinh nghia cau truc data GUI (den Node 2)
typedef struct struct_command {
    bool led_state;
    bool buzzer_state;
    bool relay_state;
} struct_command;
struct_command commandData;

// Bien co (flag) de bao hieu loop() co du lieu moi
volatile bool newDataAvailable = false;

// Khoi tao MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Peer info (cho Node 2)
esp_now_peer_info_t peerInfo;

// Callback khi NHAN data (tu Node 1)
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&sensorData, incomingData, sizeof(sensorData));
  newDataAvailable = true; // Dat co bao hieu co du lieu moi
  Serial.println("Nhan du lieu tu Node Cam Bien!");
}

// Callback khi GUI data (den Node 2)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Trang thai gui lenh den Node Actuator: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Thanh Cong" : "That Bai");
}

// Ham ket noi Wi-Fi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Dang ket noi Wi-Fi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nDa ket noi Wi-Fi!");
  Serial.print("Dia chi IP: ");
  Serial.println(WiFi.localIP());
}

// Ham ket noi lai MQTT
void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Dang ket noi MQTT (ThingBoard)...");
    // Ket noi voi username la Access Token
    if (client.connect("ESP32_Coordinator_Gateway", TB_ACCESS_TOKEN, NULL)) {
      Serial.println("Da ket noi!");
      // Ban co the subscribe de nhan lenh tu ThingBoard o day (neu can)
      // client.subscribe("v1/devices/me/rpc/request/+");
    } else {
      Serial.print("That bai, rc=");
      Serial.print(client.state());
      Serial.println(" Thu lai sau 5 giay");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // 0) Đặt mode STA (rõ ràng) TRƯỚC khi làm gì thêm
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  // 1) Kết nối Wi-Fi
  setup_wifi();

  // In thông tin để đối chiếu
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());     // dùng MAC này cho Node1
  Serial.print("WiFi Channel: "); Serial.println(WiFi.channel());   // dùng kênh này cho Node1

  // 2) MQTT
  client.setServer(TB_SERVER, TB_PORT);
  client.setBufferSize(1024);

  // 3) ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Loi khoi tao ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  // Peer tới Node2 (giữ nguyên)
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, actuatorAddress, 6);
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.channel = 0;      // giữ 0 cũng được vì gửi từ “cùng kênh” STA
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Loi them peer (Node 2)");
    return;
  }

  Serial.println("Node Coordinator san sang hoat dong.");
}

// Ham xu ly logic chinh
void processSensorData() {
  Serial.println("Xu ly du lieu cam bien...");
  
  // Reset trang thai lenh
  commandData.led_state = LOW;
  commandData.buzzer_state = LOW;
  commandData.relay_state = LOW; // Mac dinh la TAT

  bool alert = false;
  String alert_status = "BinhThuong";

  // Logic canh bao
  // Muc 3: KHAN CAP (Phat hien lua)
  if (sensorData.flame_detected) {
    Serial.println("!!! CANH BAO KHAN CAP: PHAT HIEN LUA !!!");
    commandData.led_state = HIGH;
    commandData.buzzer_state = HIGH;
    commandData.relay_state = HIGH; // Kich hoat relay (ngat gas,...)
    alert = true;
    alert_status = "CHAY_KHANCAP";
  }
  // Muc 2: Canh bao Gas hoac Khoi
  else if (sensorData.mq4_value > MQ4_THRESHOLD_GAS || sensorData.mp2_value > MP2_THRESHOLD_SMOKE) {
    Serial.println("!! CANH BAO MUC 2: Phat hien KHOI hoac GAS !!");
    commandData.led_state = HIGH;
    commandData.buzzer_state = HIGH; // Co the cho coi keu ngat quang
    commandData.relay_state = LOW;  // Chua can kich hoat relay
    alert = true;
    alert_status = "CANHBAO_KHOI_GAS";
  }
  // Muc 1: Binh thuong
  else {
    Serial.println("Trang thai: Binh thuong.");
    // Tat ca da duoc dat ve LOW o tren
  }

  // 1. Gui lenh den Node 2 (Actuator) qua ESP-NOW
  esp_now_send(actuatorAddress, (uint8_t *) &commandData, sizeof(commandData));

  // 2. Chuan bi JSON va Gui du lieu len ThingBoard qua MQTT
  // Dam bao client van dang ket noi
  if (!client.connected()) {
    reconnect_mqtt();
  }
  
  // Tao payload JSON
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["mq4"] = sensorData.mq4_value;
  jsonDoc["mp2"] = sensorData.mp2_value;
  jsonDoc["flame"] = sensorData.flame_detected;
  jsonDoc["status"] = alert_status;

  char jsonBuffer[512];
  serializeJson(jsonDoc, jsonBuffer);

  bool ok = client.publish("v1/devices/me/telemetry", jsonBuffer);
  Serial.print("Da gui len ThingBoard: "); Serial.println(jsonBuffer);
  Serial.print("Publish result: "); Serial.println(ok ? "OK" : "FAIL");
  Serial.print("MQTT state: "); Serial.println(client.state());

}

void loop() {
  // Luon duy tri ket noi MQTT
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnect_mqtt();
    }
    client.loop(); // Rat quan trong, de duy tri ket noi MQTT
  } else {
    Serial.println("Mat ket noi Wi-Fi, dang thu ket noi lai...");
    setup_wifi();
  }

  // Kiem tra co (flag) xem co du lieu moi tu Node 1 khong
  if (newDataAvailable) {
    newDataAvailable = false; // Reset co
    processSensorData();      // Xu ly du lieu
  }

  // Khong nen delay() qua lau trong loop() nay
  delay(100); 
}