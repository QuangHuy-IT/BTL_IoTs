#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ========== THAY ĐỔI THÔNG SỐ ==========
// TODO: Thay dia chi MAC cua Node 3 (Coordinator) :88:57:21:E0:77:18
uint8_t coordinatorAddress[] = {0x88, 0x57, 0x21, 0xE0, 0x77, 0x18};

// TODO: Dinh nghia cac chan GPIO cam bien
#define MQ4_PIN      35  // Chan Analog
#define MP2_PIN      32  // Chan Analog
#define FLAME_PIN    34  // Chan Digital

// Dinh nghia cau truc data de gui di
typedef struct struct_message {
    int mq4_value;
    int mp2_value;
    bool flame_detected;
} struct_message;

// Tao mot bien struct
struct_message sensorData;

// Callback function khi data duoc gui
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nTrang thai gui goi tin cuoi: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Thanh Cong" : "That Bai");
}

// --- Khai báo hằng kênh của Node3
#define ESPNOW_CHANNEL  10 

// Peer info
esp_now_peer_info_t peerInfo = {};  // KHỞI TẠO ZERO

void setup() {
  Serial.begin(115200);

  pinMode(MQ4_PIN, INPUT);
  pinMode(MP2_PIN, INPUT);
  pinMode(FLAME_PIN, INPUT);

  // 1) Đặt STA mode
  WiFi.mode(WIFI_STA);

  // 2) ***ÉP KÊNH CHO RADIO*** để trùng kênh Node3
  esp_wifi_set_promiscuous(true); // bắt buộc trước khi set channel
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // 3) ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Loi khoi tao ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // 4) Thêm peer là Node3 (MAC dạng STA MAC in từ Node3)
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, coordinatorAddress, 6);
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.channel = ESPNOW_CHANNEL; // đặt đúng kênh
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Loi them peer (Node 3)");
    return;
  }
}

// Ham doc cam bien (Ban nen thuc hien hieu chuan o day)
void readSensors() {
  // Doc gia tri tho tu ADC (0-4095)
  sensorData.mq4_value = analogRead(MQ4_PIN);
  sensorData.mp2_value = analogRead(MP2_PIN);

  // Cam bien lua thuong la digital (0 = Phat hien lua, 1 = Binh thuong)
  // TODO: Kiem tra lai logic cua cam bien lua ban dang dung
  sensorData.flame_detected = (digitalRead(FLAME_PIN) == LOW); 

  Serial.printf("Dang doc: MQ4 = %d | MP2 = %d | Lua = %s\n", 
                sensorData.mq4_value, 
                sensorData.mp2_value, 
                sensorData.flame_detected ? "PHAT HIEN" : "Binh thuong");
}

void loop() { 
  // 1. Doc du lieu tu cam bien
  readSensors();

  // 2. Gui du lieu den Node 3 (Coordinator)
  esp_err_t result = esp_now_send(coordinatorAddress, (uint8_t *) &sensorData, sizeof(sensorData));
  
  if (result == ESP_OK) {
    Serial.println("Da gui du lieu den Coordinator");
  } else {
    Serial.println("Loi khi gui du lieu");
  }

  delay(1000); // Gui du lieu moi 1 giay
}