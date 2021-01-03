#include <Arduino.h>
#include <LightTelemetry.h>
#include <LoRa.h>
#include "ble/ble.hpp"
#include "ota/ota.hpp"

const char *ssid = "TP-LINK";
const char *password = "pass2016";
const char *hostname = "LoraLTM_Receiver";

#define UPDATE_PERIOD 1000

#define LORA_FREQ 868E6
#define LORA_TX_POWER 20
#define LORA_BANDWIDTH 62.5E3
#define LORA_CODING_RATE 5
#define LORA_SPREADING_FACTOR 12

double latitude = 43.885699;
double longitude = 13.353804;
uint8_t sats = 5;

#define MAX_LORA_PACKET_SIZE 100
#define LORA_RX_TIMEOUT 10000
uint8_t loraRxBuffer[MAX_LORA_PACKET_SIZE];
uint8_t loraRxBytes;
int availableLoraBytes = 0;
uint32_t lastLoraReceived = 0;
uint8_t loraConnected = false;

uint32_t timer = 0;
uint8_t start_retry_count = 0;

void onLoraReceive(int bytes);

void setup()
{
	Serial.begin(115200);

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	Serial.println("Booting");
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	while (WiFi.waitForConnectResult() != WL_CONNECTED)
	{
		Serial.println("Connection Failed! Retry...");
		delay(1000);
		if(++start_retry_count >= 5)
		{
			Serial.println("Connection Failed! Give up");
			break;
		}
	}

	OTA_setup(hostname);

	Serial.println("Ready");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	BLE_setup(hostname);

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

	delay(1000);
}

void loop()
{
	ArduinoOTA.handle();

	if (millis() - timer > UPDATE_PERIOD)
	{
		timer = millis();
		pConnCharacteristic->setValue(&loraConnected, 1);
		pConnCharacteristic->notify();
	}

	if (availableLoraBytes > 0)
	{
		loraRxBytes = LoRa.readBytes(loraRxBuffer, availableLoraBytes <= MAX_LORA_PACKET_SIZE ? availableLoraBytes : MAX_LORA_PACKET_SIZE);
		// Serial.print("RSSI: ");
		// Serial.print(157 - constrain(LoRa.packetRssi() * -1, 0, 157));
		// Serial.print(" SNR: ");
		// Serial.println(LoRa.packetSnr());
		Serial.write(loraRxBuffer, loraRxBytes);
		LTM_reader::read(loraRxBuffer, loraRxBytes);
		pLatCharacteristic->setValue(LTM_reader::uav_lat);
		pLonCharacteristic->setValue(LTM_reader::uav_lon);
		pSatsCharacteristic->setValue(&LTM_reader::uav_satellites_visible, 1);
		pLatCharacteristic->notify();
		pLonCharacteristic->notify();
		pSatsCharacteristic->notify();
		// Serial.println();
		availableLoraBytes = 0;
	}

	if ((millis() - lastLoraReceived) > LORA_RX_TIMEOUT)
	{
		//timeout
		loraConnected = false;
	}

	digitalWrite(LED_BUILTIN, loraConnected);
}

void onLoraReceive(int bytes)
{
	availableLoraBytes = bytes;
	lastLoraReceived = millis();
	loraConnected = true;
}