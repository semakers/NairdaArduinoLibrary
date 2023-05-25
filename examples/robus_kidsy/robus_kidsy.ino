#include <nairda.h>

void setup()
{
    setKit(ROBUS_KIDSY_KIT);
    nairdaBegin("Robus Kidsy BT");
}

void loop()
{
    nairdaLoop();
}
