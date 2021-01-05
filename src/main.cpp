#include <Arduino.h>
#include <LightTelemetry.h>
#include "Lora/lora.hpp"
#include "ble/ble.hpp"
#include "ota/ota.hpp"

const char *ssid = "TP-LINK";
const char *password = "pass2016";
const char *hostname = "LoraLTM_Receiver";

#define UPDATE_PERIOD 1000

double latitude = 43.885699;
double longitude = 13.353804;
uint8_t sats = 5;

uint32_t timer = 0;
uint8_t start_retry_count = 0;

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

	LORA_setup();

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
		Serial.write(loraRxBuffer, loraRxBytes);
		LTM_reader::read(loraRxBuffer, loraRxBytes);
		pLatCharacteristic->setValue(LTM_reader::uav_lat);
		pLonCharacteristic->setValue(LTM_reader::uav_lon);
		pSatsCharacteristic->setValue(&LTM_reader::uav_satellites_visible, 1);
		pRssiCharacteristic->setValue(LoRa.packetRssi());
		pLatCharacteristic->notify();
		pLonCharacteristic->notify();
		pSatsCharacteristic->notify();
		pRssiCharacteristic->notify();
		availableLoraBytes = 0;
	}

	if ((millis() - lastLoraReceived) > LORA_RX_TIMEOUT)
	{
		//timeout
		loraConnected = false;
	}

	digitalWrite(LED_BUILTIN, loraConnected);
}

