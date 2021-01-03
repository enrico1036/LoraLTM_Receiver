#ifndef BLE_User_H
#define BLE_User_H

#include <Arduino.h>
#include <NimBLEDevice.h>

extern NimBLEServer *pServer;
extern NimBLECharacteristic *pLatCharacteristic, *pLonCharacteristic, *pSatsCharacteristic, *pConnCharacteristic, *pRssiCharacteristic;
extern bool deviceConnected;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "87127120-6346-402b-a352-cf32bc95f3e8" // Custom GPS service UUID
#define CHARACTERISTIC_UUID_RX "87127121-6346-402b-a352-cf32bc95f3e8"
#define CHARACTERISTIC_UUID_LAT "87127122-6346-402b-a352-cf32bc95f3e8"
#define CHARACTERISTIC_UUID_LON "87127123-6346-402b-a352-cf32bc95f3e8"
#define CHARACTERISTIC_UUID_SATS "87127124-6346-402b-a352-cf32bc95f3e8"
#define CHARACTERISTIC_UUID_CONN "87127125-6346-402b-a352-cf32bc95f3e8"
#define CHARACTERISTIC_UUID_RSSI "87127126-6346-402b-a352-cf32bc95f3e8"


void BLE_setup(const std::string &deviceName);

#endif /* BLE_User_H */