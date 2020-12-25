#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic *pLatCharacteristic, *pLonCharacteristic, *pSatsCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

double latitude = 43.885699;
double longitude = 13.353804;
uint8_t sats = 5;

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

    if (deviceConnected) {
        pLatCharacteristic->setValue(latitude);
        pLatCharacteristic->notify();
        pLonCharacteristic->setValue(longitude);
        pLonCharacteristic->notify();
        pSatsCharacteristic->setValue(&sats, 1);
        pSatsCharacteristic->notify();
		delay(1000); // bluetooth stack will go into congestion, if too many packets are sent
	}

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}