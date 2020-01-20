/*
Libejemplo.cpp -Descripción cpp
Creada por Nombre Autor, Fecha
Lanzado bajo licencia---
*/

//#include "arduino.h"
#include "nairda.h"
#include "linked/LinkedList.h"
#include "ping/NewPing.h"
#include "softpwm/SoftPWM.h"
#if defined(ARDUINO_ARCH_AVR)
#include "avr/Servo.h"
#elif defined(ARDUINO_ARCH_SAM)
#include "sam/Servo.h"
#else
#error "This library only supports boards with an AVR or SAM processor."
#endif

HardwareSerial *serial;
short int projectInit=100;
short int endServos=101;
short int endDC=102;
short int endLeds=103; 
short int endAnalogics=104; 
short int endDigitals=105;
short int endUltrasonics=106;
bool declaratedDescriptor=false;
bool declaratedServos=false;
bool declaratedDC=false;
bool declaratedLeds=false;
bool declaratedAnalogics=false;
bool declaratedDigitals=false;
bool declaratedUltrasonics=false;
bool executeServo=false;
bool executeDC=false;
bool executeLed=false;
int i;
int tempValue;

bool executeDCBoolean[3];
short int executeDCBuffer[3];

bool servoBoolean[7];
bool dcBoolean[3];
bool ultraosnicBoolean;

short int servoBuffer[7];
short int dcBuffer[3];


class analogic{
  public:
  int pin;

  analogic(int cpin){
    pin=cpin;
  }

 void sendValue(){ 
   int tempRead=map(analogRead(pin),0,1023,0,100);
   serial->write((char)tempRead);
   delay(5);
   serial->write((char)tempRead);
 }

};

class ultrasonic{
  public:
  NewPing* sonar;

  ultrasonic(int trigger,int echo){
    sonar =new NewPing(trigger,echo,100);
  }

  void sendValue(){ //funcion para medir el tiempo y guardarla en la variable "tiempo"
    long int tempRead=  sonar->ping_cm();
    if(tempRead>100)tempRead=100;
    serial->write((char)tempRead);
    delay(5);
    serial->write((char)tempRead);
  }

};

class digital{
  public:
  int pin;

  digital(int cpin){
    pin=cpin;
    pinMode(pin, INPUT);
  }

  void sendValue(){
    int tempRead=digitalRead(pin);
    serial->write((char)tempRead);
    delay(5);
    serial->write((char)tempRead);
  }
};

class servo {
  public:
    Servo cservo;
    
    servo(int cpin,int cpmin, int cpmax,int cdpos){
      SoftPWMEnd(cpin);
      cservo.attach(cpin, cpmin, cpmax);
      cservo.write(cdpos);
    }
    
    void setPos(int pos){
      cservo.write(map(pos, 0, 99, 0, 180));
    }

    void off(){
      cservo.detach();
    }

};

class dc {
  public:
    int a,b,pwm;
    int vel;
    
    dc(int ca,int cb,int cpwm){
      pinMode(ca,OUTPUT);
      pinMode(cb,OUTPUT);
      SoftPWMSetFadeTime(cpwm, 0, 0);
     a=ca;
     b=cb;
     pwm=cpwm;
    }

    void setVel(int cvel){
      vel=cvel;  
    }

    void setMove (int mode){
      switch(mode){
        case 0:
          digitalWrite(a,HIGH);
          digitalWrite(b,LOW);
          SoftPWMSet(pwm,map(vel,0,99,0,255));
          //serial->print("izquierda");

        break;
        case 1:
          digitalWrite(a,LOW);
          digitalWrite(b,LOW);
          SoftPWMSet(pwm,0);
          //serial->print("detener");
        break;
        case 2:
          digitalWrite(a,LOW);
          digitalWrite(b,HIGH);
          SoftPWMSet(pwm,map(vel,0,99,0,255));
          //serial->print("izquierda");
        break;
      }
    }
    
};

class led {
  public:
    int pin;
    led(int cpin){
      pinMode(cpin, OUTPUT);
      SoftPWMSetFadeTime(cpin, 0, 0);
      pin=cpin;
    }

    void setPWM(int pwm){
      SoftPWMSet(pin,map(pwm,0,99,0,255));
    }

};

void cleanServoBoolean(){
  for(int j=0;j<7;j++){
    servoBoolean[j]=false;
  }
}

void cleanDCBoolean(){
  for(int j=0;j<3;j++){
    dcBoolean[j]=false;
  }
}

void cleanExecuteDCBoolean(){
  for(int j=0;j<3;j++){
    executeDCBoolean[j]=false;
  }
}

LinkedList<servo*> listServos = LinkedList<servo*>();
LinkedList<dc*> listDC = LinkedList<dc*>();
LinkedList<led*> listLeds = LinkedList<led*>();
LinkedList<analogic*> listAnalogics = LinkedList<analogic*>();
LinkedList<digital*> listDigitals = LinkedList<digital*>();
LinkedList<ultrasonic*> listUltrasonics = LinkedList<ultrasonic*>();




void nairdaBegin(long int bauds,HardwareSerial *refSer){
  serial=refSer;
  serial->begin(bauds);
  SoftPWMBegin();
}



void nairdaLoop(){
  if(serial->available()) {
	tempValue=serial->read();
    if(tempValue==projectInit){
      asm volatile ( "jmp 0");  
    }
    if(tempValue==endServos){
      
      declaratedServos=true;
      
      //serial->println("Se han agregado todos los servos");
    }
    else if(tempValue==endDC){
      declaratedDC=true;
      
      //serial->println("Se han agregado todos los motores DC");
    }
    else if(tempValue==endLeds){
      declaratedLeds=true;
      //serial->println("Se han agregado todos los leds");
    }
    else if(tempValue==endAnalogics){
      declaratedAnalogics=true;
    }
    else if(tempValue==endDigitals){
      declaratedDigitals=true;
    }
    else if(tempValue==endUltrasonics){
      declaratedUltrasonics=true;
      declaratedDescriptor=true;
    }

    if(declaratedDescriptor==false && tempValue<100){
      if(declaratedServos==false){
        //pasos para declarar un servo
        if(!servoBoolean[0]){
          servoBoolean[0]=true;
          servoBuffer[0]=tempValue;
        }
        else if(!servoBoolean[1]){
          servoBoolean[1]=true;
          servoBuffer[1]=tempValue;
        }
        else if(!servoBoolean[2]){
          servoBoolean[2]=true;
          servoBuffer[2]=tempValue;
        }
        else if(!servoBoolean[3]){
          servoBoolean[3]=true;
          servoBuffer[3]=tempValue;
        }
        else if(!servoBoolean[4]){
          servoBoolean[4]=true;
          servoBuffer[4]=tempValue;
        }
        else if(!servoBoolean[5]){
          servoBoolean[5]=true;
          servoBuffer[5]=tempValue;
        }
        else if(!servoBoolean[6]){
          servoBoolean[6]=true;
          servoBuffer[6]=tempValue;
          servo* tempServo=new servo(servoBuffer[0],(servoBuffer[1]*100)+servoBuffer[2],(servoBuffer[3]*100)+servoBuffer[4],(servoBuffer[5]*100)+servoBuffer[6]); 
          listServos.add(tempServo);
          cleanServoBoolean();
          
          /*serial->print(tempServo->pin);
          serial->print(" : ");
          serial->print(tempServo->pmin);
          serial->print(" : ");
          serial->print(tempServo->pmax);
          serial->print(" : ");
          serial->println(tempServo->dpos);*/
          
        }
      }
      else if(declaratedDC==false && tempValue<100){
        //pasos para declarar un motor DC
        if(!dcBoolean[0]){
          dcBoolean[0]=true;
          dcBuffer[0]=tempValue;
        }
        else if(!dcBoolean[1]){
          dcBoolean[1]=true;
          dcBuffer[1]=tempValue;
        }
        else if(!dcBoolean[2]){
          dcBoolean[2]=true;
          dcBuffer[2]=tempValue;
          dc* tempDC=new dc(dcBuffer[0],dcBuffer[1],dcBuffer[2]);
          listDC.add(tempDC);
          cleanDCBoolean();
          
          /*serial->print("se agrego el motor DC ");
          serial->print(tempDC->a);
          serial->print(" : ");
          serial->print(tempDC->b);
          serial->print(" : ");
          serial->println(tempDC->pwm);*/
          
        }

      }
      else if(declaratedLeds==false && tempValue<100){
        led* tempLed =new led(tempValue);
        listLeds.add(tempLed);
        //serial->print("se agrego el led ");
        //serial->println(tempLed->pin);
      }
      else if(declaratedAnalogics==false && tempValue<100){
        analogic* tempAnalogic=new analogic(tempValue);
        listAnalogics.add(tempAnalogic);
      }
      else if(declaratedDigitals==false && tempValue<100){
        digital* tempDigital=new digital(tempValue);
        listDigitals.add(tempDigital);
      }
      else if(declaratedUltrasonics==false && tempValue<100){
        if(!ultraosnicBoolean){
          ultraosnicBoolean=true;
          i=tempValue;
        }
        else {
          ultrasonic* tempUltrasonic=new ultrasonic(i,tempValue);
          listUltrasonics.add(tempUltrasonic);
          ultraosnicBoolean=false;
        }
      }
    }
    else{
      if(executeServo==false &&executeDC==false && executeLed==false){
        int indexServos=listServos.size();
        int indexMotors=indexServos+listDC.size();
        int indexLeds=indexMotors+listLeds.size();
        int indexAnalogics=indexLeds+listAnalogics.size();
        int indexDigitals=indexAnalogics+listDigitals.size();
        int indexUltraosnics=indexDigitals+listUltrasonics.size();

        if(tempValue>=0 && tempValue<indexServos && listServos.size()>0){
          i=tempValue;
          executeServo=true;
        }
        else if(tempValue>=indexServos && tempValue<indexMotors && listDC.size()>0){
          executeDCBuffer[0]=tempValue-indexServos;
          executeDC=true;
        }
        else if(tempValue>=indexMotors && tempValue<indexLeds && listLeds.size()>0){
          i=tempValue-indexMotors;
          executeLed=true;
        }
       else if(tempValue>=indexLeds && tempValue<indexAnalogics && listAnalogics.size()>0){
          int tempPin=tempValue-indexLeds;
          listAnalogics.get(tempPin)->sendValue();
        }
        else if(tempValue>=indexAnalogics && tempValue<indexDigitals && listDigitals.size()>0){
          int tempPin=tempValue-indexAnalogics;
          listDigitals.get(tempPin)->sendValue();
        }
        else if(tempValue>=indexDigitals && tempValue<indexUltraosnics && listUltrasonics.size()>0){
          int tempPin=tempValue-indexDigitals;
          listUltrasonics.get(tempPin)->sendValue();
        }
      }
      else{
        if(executeServo==true){
          //adquirir valores para la ejecucion del servo
            listServos.get(i)->setPos(tempValue);
            executeServo=false;
          }
        else if(executeDC==true){
          //adquirir valores para la ejecucion del motor
          if(!executeDCBoolean[0]){
            executeDCBoolean[0]=true;
            executeDCBuffer[1]=tempValue;
          }
          else if(!executeDCBoolean[1]){
            executeDCBoolean[1]=true;
            executeDCBuffer[2]=tempValue;
            listDC.get(executeDCBuffer[0])->setVel(executeDCBuffer[1]);
            listDC.get(executeDCBuffer[0])->setMove(executeDCBuffer[2]);
            /*serial->print(" se ejecuto el motor");
            serial->print(" : ");
            serial->print(executeDCBuffer[1]);
            serial->print(" : ");
            serial->println(executeDCBuffer[2]); */
            cleanExecuteDCBoolean();
            executeDC=false;
          }
          
          
        }
        else if(executeLed==true){
            listLeds.get(i)->setPWM(tempValue);
            //serial->print((char)okResponse);
            //serial->print(" se ejecuto");
          executeLed=false;
        }
        
      }
    }
  }
}