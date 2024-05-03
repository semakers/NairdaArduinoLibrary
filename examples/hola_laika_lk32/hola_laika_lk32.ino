#include <nairda.h>

void setup()
{
    setKit(LK32_KIT);
    nairdaBegin("LK32 BT", 9600);
}

void loop()
{
    nairdaLoop();
}
