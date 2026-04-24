#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "virtual_machine/virtual_machine.h"
#include "blue_methods/blue_methods.h"
#include "platform/platform_hal.h"
#include "nairda.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

extern VolatileMemory volatileMemory;

BLECharacteristic *pCharacteristic;
uint8_t bleBuffer[255];
uint8_t bleIndex = 0;

#define SERVICE_UUID "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        clearVolatileMemory(&volatileMemory, true);
    };
    void onDisconnect(BLEServer *pServer)
    {
        BLEDevice::startAdvertising();
    };
};

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        String rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0)
        {
            for (int i = rxValue.length() - 1; i >= 0; i--)
            {
                if (bleIndex < 255) {
                    bleBuffer[bleIndex] = (uint8_t)rxValue[i];
                    bleIndex++;
                }
            }
        }
    }
};

bool bleAvailable()
{
    return bleIndex > 0;
}

uint8_t bleRead()
{
    if (bleAvailable())
    {
        bleIndex--;
        return bleBuffer[bleIndex];
    }
    return 0;
}

void bleWrite(uint8_t byte)
{
    std::string myStringForUnit8((char *)&byte, 1);
    String arduinoStr = String(myStringForUnit8.c_str());
    pCharacteristic->setValue(arduinoStr);
    pCharacteristic->notify();
}

void bleInit(const char *deviceName)
{
    BLEDevice::init(deviceName);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_WRITE_NR |
            BLECharacteristic::PROPERTY_READ);
    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setCallbacks(new MyCallbacks());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}

bool nextBlueByte(uint8_t *blueByte)
{
    int serialAvailable = Serial.available();
    int serial1Available = Serial1.available();
    if (serialAvailable > 0 || serial1Available > 0)
    {
        if (serialAvailable > 0)
        {
            blueByte[0] = Serial.read();
            return true;
        }
        else if (serial1Available > 0)
        {
            blueByte[0] = Serial1.read();
            return true;
        }
    }
    else if (bleAvailable())
    {
        blueByte[0] = bleRead();
        return true;
    }
    return false;
}

void hal_sendByte(uint8_t byte)
{
    bleWrite(byte);
    Serial.write(byte);
}

#endif
