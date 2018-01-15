/*
Libejemplo.cpp -Descripción cpp
Creada por Nombre Autor, Fecha
Lanzado bajo licencia---
*/

#include "arduino.h"
#include "nairda_serial1.h"
#include "linked/LinkedList.h"
#include "softpwm/SoftPWM.h"
#if defined(ARDUINO_ARCH_AVR)
#include "avr/Servo.h"
#elif defined(ARDUINO_ARCH_SAM)
#include "sam/Servo.h"
#else
#error "This library only supports boards with an AVR or SAM processor."
#endif

int projectInit=100;
int endServos=101;
int endDC=102;
int endLeds=103;
bool declaratedDescriptor=false;
bool declaratedServos=false;
bool declaratedDC=false;
bool declaratedLeds=false;
bool executeServo=false;
bool executeDC=false;
bool executeLed=false;
int i;

bool executeServoBoolean[2];
bool executeDCBoolean[3];

int executeServoBuffer[3];
int executeDCBuffer[3];

bool servoBoolean[7];
bool dcBoolean[3];

int servoBuffer[7];
int dcBuffer[3];

class servo {
  public:
    int pin;
    int pmin;
    int pmax;
    int dpos;
    Servo cservo;
    
    servo(int cpin,int cpmin, int cpmax,int cdpos){
      pin=cpin;
      pmin=cpmin;
      pmax=cpmax;
      dpos=cdpos;
      SoftPWMEnd(pin);
      cservo.attach(cpin, cpmin, cpmax);
      cservo.write(cdpos);
    }
    
    void setPos(int pos){
      cservo.write(map(pos, 0, 99, 0, 180));
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
          Serial1.print("izquierda");

        break;
        case 1:
          digitalWrite(a,LOW);
          digitalWrite(b,LOW);
          SoftPWMSet(pwm,0);
          Serial1.print("detener");
        break;
        case 2:
          digitalWrite(a,LOW);
          digitalWrite(b,HIGH);
          SoftPWMSet(pwm,map(vel,0,99,0,255));
          Serial1.print("izquierda");
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

void cleanExecuteServoBoolean(){
  for(int j=0;j<2;j++){
    executeServoBoolean[j]=false;
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
int tempValue;



void nairdaBegin(long int baudRate){
  SoftPWMBegin();
  cleanServoBoolean();
  cleanDCBoolean();
  cleanExecuteServoBoolean();
  cleanExecuteDCBoolean();
  Serial1.begin(baudRate);
}



void nairdaLoop(){
  if(Serial1.available()){
	tempValue=Serial1.read();
    if(tempValue==projectInit){
      asm volatile ( "jmp 0");  
      //Serial.println("Se limpriaron las listas");
    }
    else if(tempValue==endServos){
      declaratedServos=true;
      //Serial.println("Se han agregado todos los servos");
    }
    else if(tempValue==endDC){
      declaratedDC=true;
      //Serial.println("Se han agregado todos los motores DC");
    }
    else if(tempValue==endLeds){
      declaratedLeds=true;
      //Serial.println("Se han agregado todos los leds");
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
          /*Serial.print("se agrego el servo ");
          Serial.print(tempServo->pin);
          Serial.print(" : ");
          Serial.print(tempServo->pmin);
          Serial.print(" : ");
          Serial.print(tempServo->pmax);
          Serial.print(" : ");
          Serial.println(tempServo->dpos);*/
          cleanServoBoolean();
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
          /*Serial.print("se agrego el motor DC ");
          Serial.print(tempDC->a);
          Serial.print(" : ");
          Serial.print(tempDC->b);
          Serial.print(" : ");
          Serial.println(tempDC->pwm);*/
          cleanDCBoolean();
        }

      }
      else if(declaratedLeds==false && tempValue<100){
        led* tempLed =new led(tempValue);
        listLeds.add(tempLed);
        //Serial.print("se agrego el led ");
        //Serial.println(tempLed->pin);
      }
    }
    else{
      if(executeServo==false &&executeDC==false && executeLed==false){
        if(tempValue>=0 && tempValue<listServos.size() && listServos.size()>0){
          //ejecutar servo
          //Serial.print("Preparado para ejecutar el servo ");
          //Serial.println(listServos.get(tempValue)->pin);
          executeServoBuffer[0]=tempValue;
          executeServo=true;
        }
        else if(tempValue>=listServos.size() && tempValue<(listServos.size()+listDC.size()) && listDC.size()>0){
          //ejecutar motor DC
          executeDCBuffer[0]=tempValue-listServos.size();
          executeDC=true;
        }
        else if(tempValue>=(listServos.size()+listDC.size()) && tempValue<(listServos.size()+listDC.size()+listLeds.size()) && listLeds.size()>0){
          //ejecutar led
          //Serial.print("preparado para ejecutar el led ");
          //Serial.println(listLeds.get(tempValue-(listServos.size()+listDC.size()))->pin);
          i=tempValue-(listServos.size()+listDC.size());
          executeLed=true;
        }
      }
      else{
        if(executeServo==true){
          //adquirir valores para la ejecucion del servo
          if(!executeServoBoolean[0]){
            executeServoBoolean[0]=true;
            listServos.get(executeServoBuffer[0])->setPos(tempValue);
            cleanExecuteServoBoolean();
            executeServo=false;
          }
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
            /*Serial.print(" se ejecuto el motor");
            Serial.print(" : ");
            Serial.print(executeDCBuffer[1]);
            Serial.print(" : ");
            Serial.println(executeDCBuffer[2]); */
            cleanExecuteDCBoolean();
            executeDC=false;
          }
          
          
        }
        else if(executeLed==true){
            listLeds.get(i)->setPWM(tempValue);
            //Serial.print(" se ejecuto");
          executeLed=false;
        }
      }
    }
  }
}
