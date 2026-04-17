#include "virtual_machine/virtual_machine.h"
#include "blue_methods/blue_methods.h"
#include "kits/v1.h"

uint8_t getMapedPin(uint8_t pin)
{
    return (pin >= 70) ? (A0 + (pin - 70)) : pin;
}
