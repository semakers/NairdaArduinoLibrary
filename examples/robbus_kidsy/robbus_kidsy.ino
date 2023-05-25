#include <nairda.h>

void setup()
{
    setKit(ROBBUS_KIDSY_KIT);
    nairdaBegin("Robbus Kidsy BT");
}

void loop()
{
    nairdaLoop();
}
