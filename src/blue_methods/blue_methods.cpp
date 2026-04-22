#include "virtual_machine/virtual_machine.h"
#include <stdint.h>
#include "value_conversion/value_conversion.h"
#include <Arduino.h>
#include "nairda.h"

extern VolatileMemory volatileMemory;
extern uint8_t currentKit;

#if defined(ARDUINO_ARCH_ESP32)
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "nairda_debug/nairda_debug.h"

BLECharacteristic *pCharacteristic;
uint8_t bleBuffer[255];
uint8_t bleIndex = 0;

#define SERVICE_UUID "0000ffe0-0000-1000-8000-00805f9b34fb" // UART service UUID
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
            // Solo poner en buffer — nairdaLoop (Core 1) los procesa.
            // No llamar nairdaDebug aquí (Core 0/BLE task) porque
            // operaciones de flash desde el BLE task bloquean el IWDT.
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
    if (bleIndex > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t bleRead()
{
    if (bleAvailable())
    {
        bleIndex--;
        return bleBuffer[bleIndex];
    }
    else
    {
        return 0;
    }
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
    BLEDevice::init(deviceName); // Give it a name
    // Create the BLE Server
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_WRITE_NR |
            BLECharacteristic::PROPERTY_READ);
    pCharacteristic->addDescriptor(new BLE2902());

    pCharacteristic->setCallbacks(new MyCallbacks());
    // Start the service
    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}
#endif

bool nextBlueByte(uint8_t *blueByte)
{

#if defined(ARDUINO_ARCH_ESP32)

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
#else

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
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

#else

    if (Serial.available())
    {
        blueByte[0] = Serial.read();
        return true;

#endif

#endif
    }
    return false;
}
