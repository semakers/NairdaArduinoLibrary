#include "virtual_machine/virtual_machine.h"
#include <stdint.h>
#include "value_conversion/value_conversion.h"
#include <Arduino.h>
#include "nairda.h"

extern VolatileMemory volatileMemory;
extern bool running;
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

        restartRunFromEeprom();
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
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0)
        {
            for (int i = 0; i < rxValue.length(); i++)
            {

                if (!running)
                {

                    nairdaDebug(rxValue[i], &volatileMemory);
                }
                else
                {

                    bleBuffer[bleIndex] = (uint8_t)rxValue[(rxValue.length() - 1) - i];
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
    pCharacteristic->setValue(myStringForUnit8);
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

void sendMemorySize(uint32_t memorySize)
{
#if defined(ARDUINO_ARCH_ESP32)
    spi_flash_erase_range(0x200000, 4096 * 128);
    char cleanBuffer[22];
    memset(cleanBuffer, 0, 22);
    pCharacteristic->setValue(cleanBuffer);
    char myString[4];
    myString[0] = (char)firstValue(memorySize);
    myString[1] = (char)secondValue(memorySize);
    myString[2] = (char)thirdValue(memorySize);
    myString[3] = 0;
    pCharacteristic->setValue(myString);
    pCharacteristic->notify();

#else

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    Serial1.write((char)firstValue(memorySize));
    Serial1.write((char)secondValue(memorySize));
    Serial1.write((char)thirdValue(memorySize));
#endif
    Serial.write((char)firstValue(memorySize));
    Serial.write((char)secondValue(memorySize));
    Serial.write((char)thirdValue(memorySize));
#endif
}

bool nextBlueByte(uint8_t *blueByte)
{

#if defined(ARDUINO_ARCH_ESP32)

    if (bleAvailable())
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