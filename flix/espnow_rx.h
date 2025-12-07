#pragma once
#include <Arduino.h>

#define ESPNOW_RC_MAX_CHANNELS 8

#pragma pack(push, 1)
struct Packet{
  uint8_t header=0xAA;
  uint16_t channels[ESPNOW_RC_MAX_CHANNELS];
  uint16_t crc;
};
#pragma pack(pop)

void EspNow_Init();
void EspNow_Stop();

bool EspNow_HasNewPacket();
void EspNow_ClearNewPacket();

uint16_t* EspNow_GetChannels();
int EspNow_GetPacketCount();

uint32_t EspNow_GetLastRecvTime();