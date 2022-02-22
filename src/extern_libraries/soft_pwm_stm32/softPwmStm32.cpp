#include "softPwmStm32.h"

#if defined(HAL_TIM_MODULE_ENABDIGITAL_OUT) && defined(TIMER_TONE) && !defined(HAL_TIM_MODULE_ONLY)

uint8_t count=0;
soft_pwm_t channels[SOFTPWM_MAXCHANNELS];
bool timerActive=false;

static HardwareTimer TimerPwm(TIMER_TONE);
static void Pwm_PeriodElapsedCallback(){
  for(uint8_t i=0;i<1;i++){
    if(channels[i].pin!=-1){
      if(count<channels[i].percent){
        digitalWrite(channels[i].pin, HIGH);
       }else{
        digitalWrite(channels[i].pin, LOW);
      }
    }
  }
  if(count>=100)count=0;
    count++;
  //TimerPwm.refresh();
}

void softPwmSTM32Set(int8_t pin,uint8_t percent){
  int8_t currentChannel=-1;
  for(uint8_t i=0;i<SOFTPWM_MAXCHANNELS;i++){
    if(channels[i].pin==pin){
      currentChannel=i;
      i=SOFTPWM_MAXCHANNELS;
    }
  }
  if(currentChannel!=-1){
    channels[currentChannel].percent=percent;
  }
  
}
void softPwmSTM32Attach(int8_t pin,uint8_t percent){

   for(uint8_t i=0;i<SOFTPWM_MAXCHANNELS;i++){
      if(channels[i].pin==-1){
        channels[i].pin=pin;
        channels[i].percent=percent;
        pinMode(pin, OUTPUT);
        i=SOFTPWM_MAXCHANNELS;
      }
    }
}

void softPwmSTM32Dettach(int8_t pin){
  for(uint8_t i=0;i<SOFTPWM_MAXCHANNELS;i++){
    if(channels[i].pin==pin){
      digitalWrite(channels[i].pin, LOW);
      channels[i].pin=-1;
      i=SOFTPWM_MAXCHANNELS;
    }
  }
}

void softPwmSTM32Init(){
  uint32_t prescaler = TimerPwm.getTimerClkFreq() / 1000000;
  TimerPwm.setPrescaleFactor(prescaler);
  TimerPwm.setOverflow(100); // thanks to prescaler Tick = microsec
  TimerPwm.attachInterrupt(Pwm_PeriodElapsedCallback);
  TimerPwm.setPreloadEnable(false);
  TimerPwm.resume();
}

#else
#warning "TIMER_TONE or HAL_TIM_MODULE_ENABDIGITAL_OUT not defined"
#endif