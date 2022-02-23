#include "analogic_component.h"
#include "load_from_eeprom.h"
#include "components/inputs/digital_in/digital_in_component.h"
#include "components/component.h"

#include <Arduino.h>


extern LinkedList<component_t *> listAnalogics;
extern bool loadedAnalogics;
extern uint16_t descArgsBuffer[5];
extern uint32_t execBuffer[6];


void analogicCreate(uint16_t *args,uint8_t * pins){
    pins[0] = args[1];
}

void analogicSense(uint8_t * pins,uint8_t* tempRead){
        #if defined(ARDUINO_ARCH_STM32)
                  tempRead[0] = map(analogRead(pins[0]), 0, 1023, 0, 100);
#elif defined(ARDUINO_ARCH_ESP32)
                  tempRead[0] = map(analogRead(pins[0]), 0, 4095, 0, 100);
#else
                  tempRead[0] = map(analogRead(pins[0]), 0, 1023, 0, 100);
#endif
}

void analogicOff(){

}

void analogicEepromLoad(){
     uint8_t currentByte;
    while (!loadedAnalogics)
    {
        currentByte = nextByte();
        if (currentByte == endAnalogics)
        {
            loadedAnalogics = true;
        }
        else
        {
            descArgsBuffer[0] = ANALOGIC;
            descArgsBuffer[1] = getMapedPin(currentByte);
            component_t *tempAnalogic = newComponent(descArgsBuffer);
            listAnalogics.add(tempAnalogic);
        }
    }
    digitalInEepromLoad();
}

int32_t analogicEepromRead(){
     return getSensVal(ANALOGIC,listAnalogics.get(nextByte()));
}

