#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 

#include "tx_rx_packet.h"
#include "LQCALC.h"
#include "espnow_rx.h"


static volatile bool g_newPacket = false;
static volatile uint32_t g_packetCount = 0;

static uint16_t g_channels[RC_MAX_CHANNELS] = {};
static uint32_t g_lastRecvMs = 0;

static LQCALC<100> g_lqCalc;
static rc_data_recv_cb_t OnNewRcData = nullptr;


// --- ESP-NOW 接收回调 ---
#if ESP_IDF_VERSION_MAJOR >= 5
static void EspNow_OnReceive(const esp_now_recv_info *info, const uint8_t *data, int len) {
  const uint8_t *mac = info ? info->src_addr : nullptr;
#else
static void EspNow_OnReceive(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  (void)mac;
  using RcPacket = PACKET_FRAME_T(packet_channel_t);
  if (len != sizeof(RcPacket)) {
    Serial.printf("[WARN] unexpected size: %d bytes\n", len);
    return;
  }

  const uint8_t crc = getPacketCrc8().calc(data, static_cast<uint16_t>(len - 1), 0);
  if (crc != data[len - 1]){
    Serial.println("[ERR] battery CRC mismatch");
    return;
  }

  const auto *pkt = reinterpret_cast<const RcPacket *>(data);

  if (pkt->h.header != PACKET_HEAD_CHANNEL) {
    Serial.printf("[ERR]header mismatch:%d,expected:0xAA",pkt->h.header);
    return;
  }

  memcpy(g_channels, pkt->p.channels, sizeof(pkt->p.channels));
  g_packetCount++;
  g_lastRecvMs = millis();
  g_newPacket = true;
  g_lqCalc.add();
  if (OnNewRcData != nullptr) {
    OnNewRcData(pkt->p.channels);
  }


  static uint32_t lastPrint = 0;

     // 每 5 秒打印一次直方图
  if (millis() - lastPrint > 1000) {
      // 打印通道值
    Serial.print("PPM Channels: ");
    for (int i = 0; i < RC_MAX_CHANNELS; i++) {
      Serial.printf("%d ", pkt->p.channels[i]);
    }
    Serial.println();
    lastPrint = millis();
  }
}

void EspNow_Init(rc_data_recv_cb_t OnRcData){
  OnNewRcData = OnRcData;
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

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, ESPNOW_BROADCAST_ADDRESS, sizeof(ESPNOW_BROADCAST_ADDRESS));
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ESP-NOW add peer failed!");
    ESP.restart();
  }
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
uint8_t EspNow_GetLinkQuality() { return g_lqCalc.getLQ(); }

static void LinkQualityTick(uint32_t now) {
  static uint32_t lastMs = 0;
  if (now - lastMs >= TX_TO_RX_RCDATA_PERIOD_MS) {
    lastMs = now;
    g_lqCalc.inc();
  }
}

static void SendLinkStatistics(uint32_t now) {
  static uint32_t lastSendTime = 0;
  if ((now - lastSendTime) > RX_TO_TX_LINKSTATS_PERIOD_MS) {
    lastSendTime = now;
    PACKET_FRAME_T(packet_linkstatistics_t) packet = {0};
    packet.p.uplink_Link_quality = g_lqCalc.getLQ();
    setHeaderAndCrc(&packet, PACKET_HEAD_LINK_STATISTICS);
    EspNow_Send(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
  }
}

void EspNow_LinkStatisticsTick(uint32_t now) {
  LinkQualityTick(now);
  SendLinkStatistics(now);
}

uint32_t EspNow_GetLastRecvTime() {
  return g_lastRecvMs;
}

bool EspNow_Send(const uint8_t *data, size_t len) {
  esp_err_t err = esp_now_send(ESPNOW_BROADCAST_ADDRESS, data, len);
  return err == ESP_OK;
}


