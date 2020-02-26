#include <nairda.h>
//recuerda que puedes usar código aparte del necesario para usar nairda

void setup() {
    nairdaBegin(9600,&Serial);
    //velocidad en baudios de tu bluetooth (HM10, hc-08, hc-06 o hc-05)
    //puerto de serie que se usará en caso de Aruino MEGA se puede usar Serial1, Serial2 y Serial3
}

void loop() {
    nairdaLoop();
}