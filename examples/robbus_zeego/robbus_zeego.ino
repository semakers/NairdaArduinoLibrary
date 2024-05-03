#include <nairda.h>

void setup()
{
    setKit(ROBBUS_ZEEGO_KIT);
    nairdaBegin(" ZEEGO BT", 9600);
}

void loop()
{
    nairdaLoop();
}
