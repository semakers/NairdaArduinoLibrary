#include <nairda.h>

void setup()
{
    setKit(ROBBUS_KIDSY_KIT);
    nairdaBegin("Robbus Kidsy BT", 9600);
}

void loop()
{
    nairdaLoop();
}
