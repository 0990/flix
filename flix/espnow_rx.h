#pragma once
#include <Arduino.h>

typedef void (*rc_data_recv_cb_t)(const uint16_t *channels);

void EspNow_Init(rc_data_recv_cb_t OnRcData = nullptr);
void EspNow_Stop();

bool EspNow_HasNewPacket();
void EspNow_ClearNewPacket();

uint16_t* EspNow_GetChannels();
int EspNow_GetPacketCount();
uint8_t EspNow_GetLinkQuality();

void EspNow_LinkStatisticsTick(uint32_t now);

uint32_t EspNow_GetLastRecvTime();
bool EspNow_Send(const uint8_t *data, size_t len);
