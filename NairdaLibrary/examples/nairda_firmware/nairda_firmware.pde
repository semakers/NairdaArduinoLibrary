#include <nairda.h>
//recuerda que puedes usar código aparte del necesario para usar nairda

void setup() {
    nairdaBegin(9600);//Aquí la velocidad en baudios de tu bluetooth (HM10, hc-08, hc-06 o hc-05)
}

void loop() {
    nairdaLoop();
}