#ifndef LORA_User_H
#define LORA_User_H

#include <Arduino.h>
#include <LoRa.h>

#define LORA_FREQ 433E6
#define LORA_TX_POWER 17
#define LORA_BANDWIDTH 62.5E3
#define LORA_CODING_RATE 5
#define LORA_SPREADING_FACTOR 11

#define MAX_LORA_PACKET_SIZE 100
#define LORA_RX_TIMEOUT 10000

extern uint8_t loraRxBuffer[MAX_LORA_PACKET_SIZE];
extern uint8_t loraRxBytes;
extern int availableLoraBytes;
extern uint32_t lastLoraReceived;
extern uint8_t loraConnected;

void LORA_setup();

#endif /* LORA_User_H */