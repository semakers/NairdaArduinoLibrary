 #include <Arduino.h>
 #include <stdint.h>

#define SOFTPWM_MAXCHANNELS 20
typedef struct{
  int8_t pin =-1;
  uint8_t percent =0;
}soft_pwm_t;

void softPwmSTM32Set(int8_t pin,uint8_t percent);
void softPwmSTM32Attach(int8_t pin,uint8_t percent);
void softPwmSTM32Dettach(int8_t pin);
void softPwmSTM32Init();
