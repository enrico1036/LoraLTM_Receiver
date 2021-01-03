#include "lora.hpp"

uint8_t loraRxBuffer[MAX_LORA_PACKET_SIZE];
uint8_t loraRxBytes;
int availableLoraBytes = 0;
uint32_t lastLoraReceived = 0;
uint8_t loraConnected = false;

void onLoraReceive(int bytes);

void LORA_setup()
{
	LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
	LoRa.begin(LORA_FREQ);
	LoRa.sleep();
	LoRa.enableCrc();
	LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN);
	LoRa.setSignalBandwidth(LORA_BANDWIDTH);
	LoRa.setCodingRate4(LORA_CODING_RATE);
	LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
	LoRa.idle();
	LoRa.onReceive(onLoraReceive);
	LoRa.receive();
}

void onLoraReceive(int bytes)
{
	availableLoraBytes = bytes;
	lastLoraReceived = millis();
	loraConnected = true;
}