#include <nairda.h>
//recuerda que puedes usar código aparte del necesario para usar nairda

void setup() {
    nairdaBegin(9600,&Serial);
    //velocidad en baudios de tu bluetooth (HM10, hc-08)
}

void loop() {
    nairdaLoop();
}