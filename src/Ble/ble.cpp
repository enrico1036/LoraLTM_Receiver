#include "ble.hpp"


NimBLEServer *pServer = NULL;
NimBLECharacteristic *pLatCharacteristic, *pLonCharacteristic, *pSatsCharacteristic, *pConnCharacteristic, *pRssiCharacteristic;
bool deviceConnected = false;


class MyServerCallbacks : public NimBLEServerCallbacks
{
	void onConnect(BLEServer *pServer)
	{
		deviceConnected = true;
		Serial.println("Connected");
	};

	void onDisconnect(BLEServer *pServer)
	{
		deviceConnected = false;
		Serial.println("Disconnected");
	}
};

class MyCallbacks : public NimBLECharacteristicCallbacks
{
	void onWrite(BLECharacteristic *pCharacteristic)
	{
		std::string rxValue = pCharacteristic->getValue();

		if (rxValue.length() > 0)
		{
			Serial.println("*********");
			Serial.print("Received Value: ");
			for (int i = 0; i < rxValue.length(); i++)
				Serial.print(rxValue[i]);

			Serial.println();
			Serial.println("*********");
		}
	}
};

void BLE_setup(const std::string &deviceName)
{

	NimBLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_DEFAULT);
	
	// Create the BLE Device
	NimBLEDevice::init(deviceName);

	// Create the BLE Server
	pServer = BLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());

	// Create the BLE Service
	NimBLEService *pService = pServer->createService(SERVICE_UUID);

	// Create a BLE Characteristic
	pLatCharacteristic = pService->createCharacteristic(
		CHARACTERISTIC_UUID_LAT,
		NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);

	// pLatCharacteristic->addDescriptor(new BLE2902());

	pLonCharacteristic = pService->createCharacteristic(
		CHARACTERISTIC_UUID_LON,
		NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);

	// pLonCharacteristic->addDescriptor(new BLE2902());

	pSatsCharacteristic = pService->createCharacteristic(
		CHARACTERISTIC_UUID_SATS,
		NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);

	// pSatsCharacteristic->addDescriptor(new BLE2902());

	pConnCharacteristic = pService->createCharacteristic(
		CHARACTERISTIC_UUID_CONN,
		NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);

	// pConnCharacteristic->addDescriptor(new BLE2902());

	pRssiCharacteristic = pService->createCharacteristic(
		CHARACTERISTIC_UUID_RSSI,
		NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);

	// pRssiCharacteristic->addDescriptor(new BLE2902());

	BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
		CHARACTERISTIC_UUID_RX,
		NIMBLE_PROPERTY::WRITE);

	pRxCharacteristic->setCallbacks(new MyCallbacks());

	// Start the service
	pService->start();

	// Start advertising
	NimBLEAdvertising *pAdvertising = pServer->getAdvertising();
	pAdvertising->addServiceUUID(SERVICE_UUID);
	pAdvertising->start();
	Serial.println("Waiting a client connection to notify...");
}