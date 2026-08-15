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

#include "nairda_log.h"

extern VolatileMemory volatileMemory;

BLECharacteristic *pCharacteristic;

// Anillo FIFO de recepcion BLE. Sustituye a la pila LIFO original, que
// entregaba los paquetes EN ORDEN INVERSO cuando llegaba un segundo write
// antes de drenar el primero (el main task consume ~1 byte/ms: un chunk de
// config de 20 bytes tarda ~20 ms en drenarse, y la app los manda cada
// 20 ms — justo en el filo). Con 256 posiciones y indices uint8_t, el
// desbordamiento del indice ES el modulo: head==tail significa vacio.
// Un solo productor (tarea BLE, onWrite) y un solo consumidor (main task),
// cada indice lo escribe solo su lado: no hace falta mutex, si `volatile`.
uint8_t bleBuffer[256];
static volatile uint8_t bleHead = 0; // proximo hueco a escribir (productor)
static volatile uint8_t bleTail = 0; // proximo byte a leer (consumidor)
static volatile uint16_t bleDropped = 0; // bytes tirados por buffer lleno

// Cross-thread flag: set from the BLE task (inside onDisconnect callback) and
// consumed from the main task (via blePollActions, called from nairdaLoop).
// Calling BLEDevice::startAdvertising() directly inside onDisconnect is
// unreliable on ESP-IDF — the BLE stack is in the middle of its own event
// processing and either ignores the call or hangs. Deferring it to the main
// task fixes that.
static volatile bool bleNeedsRestartAdvertising = false;

#define SERVICE_UUID "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        NRD_LOG("[NRD/BLE] onConnect — clearVolatileMemory\n");
        clearVolatileMemory(&volatileMemory, true);
    };
    void onDisconnect(BLEServer *pServer)
    {
        NRD_LOG("[NRD/BLE] onDisconnect — se re-anunciara desde el main task\n");
        // Don't call startAdvertising() here — defer to main task.
        bleNeedsRestartAdvertising = true;
    };
};

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        String rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0)
        {
            NRD_LOG("[NRD/BLE] RX %dB:", (int)rxValue.length());
            for (int i = 0; i < (int)rxValue.length(); i++)
            {
                // FIFO: en orden de llegada. Lleno cuando avanzar head lo
                // pondria sobre tail (dejamos 1 hueco de separacion).
                uint8_t next = (uint8_t)(bleHead + 1);
                if (next != bleTail)
                {
                    bleBuffer[bleHead] = (uint8_t)rxValue[i];
                    bleHead = next;
                    NRD_LOG(" %02X", (uint8_t)rxValue[i]);
                }
                else
                {
                    bleDropped++;
                    NRD_LOG(" !DROP(%u)", (unsigned)bleDropped);
                }
            }
            NRD_LOG("\n");
        }
    }
};

bool bleAvailable()
{
    return bleHead != bleTail;
}

uint8_t bleRead()
{
    if (bleAvailable())
    {
        uint8_t b = bleBuffer[bleTail];
        bleTail = (uint8_t)(bleTail + 1);
        return b;
    }
    return 0;
}

void bleWrite(uint8_t byte)
{
    // setValue con puntero+longitud: la version anterior pasaba por c_str(),
    // que corta en el primer NUL — un hipotetico byte 0x00 dejaba la
    // caracteristica VACIA en vez de valer un byte a cero. Hoy el protocolo
    // +1 nunca emite 0, pero el transporte no debe depender de eso.
    pCharacteristic->setValue(&byte, 1);
    pCharacteristic->notify();
    NRD_LOG("[NRD/BLE] TX %02X (notify)\n", byte);
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

// Polled from the main task (nairdaLoop) every iteration. Consumes any
// pending deferred BLE actions — currently just the "restart advertising
// after disconnect" request from onDisconnect.
void blePollActions()
{
    if (bleNeedsRestartAdvertising) {
        bleNeedsRestartAdvertising = false;
        BLEDevice::startAdvertising();
    }
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
