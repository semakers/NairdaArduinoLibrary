#if !defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "virtual_machine/virtual_machine.h"
#include "extern_libraries/new_ping/new_ping.h"

void ultrasonicCreate(uint16_t *args, component_t *component)
{
    // Guardar el pin de trigger tambien en pins[0]: el mock del banco lo usa
    // como valor de referencia, y tenerlo registrado es inocuo en el camino
    // real (en AVR solo el sonar lo usaba).
    component->pins[0] = args[1];
    component->sonar = new NewPing(args[1], args[2], 100);
}

// Mock del banco de protocolo (misma bandera que avr_sensors.cpp): devuelve
// siempre el pin de trigger. Si la app enseña otro valor, la clave viajo
// cruzada. Nota: en el mock no usamos component->sonar, pero se crea igual.
#define NAIRDA_MOCK_SENSORS 0

void ultrasonicSenseImpl(component_t *component, uint8_t *tempRead)
{
    NewPing *sonar = (NewPing *)component->sonar;
#if NAIRDA_MOCK_SENSORS
    (void)sonar;
    tempRead[0] = component->pins[0] <= 100 ? component->pins[0] : 100;
    return;
#endif
    tempRead[0] = sonar->ping_cm();
    static int lastValue;
    static int zeroCounter = 0;

    if (zeroCounter == 3)
    {
        if (tempRead[0] == 0)
            tempRead[0] = 99;
        else
            zeroCounter = 0;
    }
    else
    {
        if (tempRead[0] == 0)
            zeroCounter++;
        lastValue = (tempRead[0] == 0) ? lastValue : tempRead[0];
        tempRead[0] = (tempRead[0] == 0) ? lastValue : tempRead[0];
    }
}

void ultrasonicOffImpl(component_t *component)
{
    NewPing *sonar = (NewPing *)component->sonar;
    free(sonar);
}

#endif
