#include <nairda.h>

void setup()
{
    setKit(ROBBUS_ZEEGO_KIT);
    nairdaBegin(" ZEEGO BT");
}

void loop()
{
    nairdaLoop();
}
