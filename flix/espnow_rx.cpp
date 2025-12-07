#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 
#include "espnow_rx.h"

#define ESPNOW_CHANNEL 6

static volatile bool g_newPacket = false;
static volatile uint32_t g_packetCount = 0;

static uint16_t g_channels[ESPNOW_RC_MAX_CHANNELS] = {};
static uint32_t g_lastRecvMs = 0;


static  uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

// --- ESP-NOW 接收回调 ---
#if ESP_IDF_VERSION_MAJOR >= 5
static void EspNow_OnReceive(const esp_now_recv_info *info, const uint8_t *data, int len) {
  const uint8_t *mac = info ? info->src_addr : nullptr;
#else
static void EspNow_OnReceive(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  (void)mac;
  if (len != sizeof(Packet)) {
    Serial.printf("[WARN] unexpected size: %d bytes\n", len);
    return;
  }

  Packet pkt;
  memcpy(&pkt, data, len);

  uint16_t calc = crc16((uint8_t*)&pkt, sizeof(pkt) - sizeof(pkt.crc));

  if (pkt.header != 0xAA) {
    Serial.printf("[ERR]header mismatch:%d,expected:0xAA",pkt.header);
    return;
  }

  if (calc != pkt.crc) {
    Serial.println("[ERR] CRC mismatch");
    return;
  }

  memcpy(g_channels, pkt.channels, sizeof(pkt.channels));
  g_packetCount++;
  g_lastRecvMs = millis();
  g_newPacket = true;


  // static uint32_t lastPrint = 0;

  //    // 每 5 秒打印一次直方图
  // if (millis() - lastPrint > 1000) {
  //     // 打印通道值
  //   Serial.print("PPM Channels: ");
  //   for (int i = 0; i < ESPNOW_RC_MAX_CHANNELS; i++) {
  //     Serial.printf("%d ", pkt.channels[i]);
  //   }
  //   Serial.println();
  //   lastPrint = millis();
  // }
}

void EspNow_Init(){
  WiFi.mode(WIFI_STA);

  // 2. 启用 LR（Low Rate）协议，增强距离
  // 必须在 WiFi 初始化后调用
  #if ESP_IDF_VERSION_MAJOR >= 4
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);   // 新版 IDF
  #else
    esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR); // 旧版 IDF
  #endif

  // 3. 固定到指定信道（避免因扫描导致通信失败）
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    ESP.restart();
  }
  
  esp_now_register_recv_cb(EspNow_OnReceive);
  Serial.println("ESP-NOW Receiver Ready.");
}

void EspNow_Stop() {
  esp_now_deinit();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
  Serial.println("ESPNOW stopped.");
}

bool EspNow_HasNewPacket() { return g_newPacket; }
void EspNow_ClearNewPacket() { g_newPacket = false; }

uint16_t* EspNow_GetChannels() { return g_channels; }
int EspNow_GetPacketCount() { return g_packetCount; }

uint32_t EspNow_GetLastRecvTime() {
  return g_lastRecvMs;
}


