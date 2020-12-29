#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <LightTelemetry.h>
#include <LoRa.h>


#define UPDATE_PERIOD 1000

#define LORA_FREQ 868E6
#define LORA_TX_POWER 20
#define LORA_BANDWIDTH 31.25E3
#define LORA_CODING_RATE 6

BLEServer *pServer = NULL;
BLECharacteristic *pLatCharacteristic, *pLonCharacteristic, *pSatsCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

double latitude = 43.885699;
double longitude = 13.353804;
uint8_t sats = 5;

#define MAX_LORA_PACKET_SIZE 100
#define LORA_RX_TIMEOUT 2000
uint8_t loraRxBuffer[MAX_LORA_PACKET_SIZE];
uint8_t loraRxBytes;
int availableLoraBytes = 0;
uint32_t lastLoraReceived = 0;

uint32_t timer = 0;

void onLoraReceive(int bytes);

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "87127120-6346-402b-a352-cf32bc95f3e8" // Custom GPS service UUID
#define CHARACTERISTIC_UUID_RX "87127121-6346-402b-a352-cf32bc95f3e8"
#define CHARACTERISTIC_UUID_LAT "87127122-6346-402b-a352-cf32bc95f3e8"
#define CHARACTERISTIC_UUID_LON "87127123-6346-402b-a352-cf32bc95f3e8"
#define CHARACTERISTIC_UUID_SATS "87127124-6346-402b-a352-cf32bc95f3e8"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};


void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
	LoRa.begin(LORA_FREQ);
	LoRa.sleep();
	LoRa.enableCrc();
	LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN);
	LoRa.setSignalBandwidth(LORA_BANDWIDTH);
	LoRa.setCodingRate4(LORA_CODING_RATE);
	LoRa.idle();
  LoRa.onReceive(onLoraReceive);
  LoRa.receive();

  // Create the BLE Device
  BLEDevice::init("LoraLTM_receiver");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pLatCharacteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_LAT,
										BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
									);
                      
  // pLatCharacteristic->addDescriptor(new BLE2902());

  pLonCharacteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_LON,
										BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
									);
                      
  // pLonCharacteristic->addDescriptor(new BLE2902());

  pSatsCharacteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_SATS,
										BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
									);
                      
  // pSatsCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {

  if (millis() - timer > UPDATE_PERIOD)
  {
    timer = millis();
    
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
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
    Serial.print("ok: ");
    Serial.print(LTM_reader::LTM_pkt_ok, 10);
    Serial.print(" ko: ");
    Serial.println(LTM_reader::LTM_pkt_ko, 10);
    Serial.print("lat: ");
    Serial.print(LTM_reader::uav_lat, 10);
    Serial.print(" lon: ");
    Serial.print(LTM_reader::uav_lon, 10);
    Serial.print(" sat: ");
    Serial.println(LTM_reader::uav_satellites_visible, 10);
    if (deviceConnected)
    {
      pLatCharacteristic->setValue(LTM_reader::uav_lat);
      pLatCharacteristic->notify();
      pLonCharacteristic->setValue(LTM_reader::uav_lon);
      pLonCharacteristic->notify();
      pSatsCharacteristic->setValue(&LTM_reader::uav_satellites_visible, 1);
      pSatsCharacteristic->notify();
    }
    // Serial.println();
    availableLoraBytes = 0;
  }

  if ((lastLoraReceived + LORA_RX_TIMEOUT) > millis())
	{
		//timeout
		digitalWrite(LED_BUILTIN, HIGH);
	}
	else
	{
		//correct receiving
		digitalWrite(LED_BUILTIN, LOW);
	}
}

void onLoraReceive(int bytes)
{
  availableLoraBytes = bytes;
  lastLoraReceived = millis();
}