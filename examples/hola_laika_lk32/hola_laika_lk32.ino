#include <nairda.h>

void setup()
{
    setKit(LK32_KIT);
    nairdaBegin("LK32 BT");
}

void loop()
{
    nairdaLoop();
}
