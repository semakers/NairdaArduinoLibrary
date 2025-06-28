#include "virtual_machine/virtual_machine.h"
#include "dynamixel_component.h"
#include "extern_libraries/linked_list/linked_list.h"

#include <Arduino.h>
#include <DynamixelSDK.h>

extern dynamixel::PortHandler *portHandler;
extern dynamixel::PacketHandler *packetHandler;

void dynamixelColorExec(uint32_t *execArgs) {}

void dynamixelWriteExec(uint32_t *execArgs) {}

void dynamixelReadSense(uint32_t *execArgs)
{
    // Serial3.write((char)getSensVal(type, component));
}
