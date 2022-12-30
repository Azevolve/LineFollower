#include <Arduino.h>
#include "Motor.h"

const Motor_Pins Pins  = {21, 0, 22,  17,  25,  26};
Motor leftmotor(Pins);

void ISR_A() {leftmotor.encFaseA.interrupt();} 
void ISR_B() {leftmotor.encFaseB.interrupt();}

double read_ (int PIN){
  double sum = 0;
  for (int i = 0; i < 10; i++){
    sum += analogRead(PIN);
  }
  sum /= 10.0;
  sum /= 4095.0;
  return sum;
}

void loop() {
  leftmotor.pid.print();
  delay(50);
}

void loop2(void* Param) {
  while (true) {  
    double Kc = read_(36);
    double Ti = read_(37);
    if (Ti <= 0.1) Ti = 0.1;
    double Td = read_(38);
    leftmotor.pid.set_params(Kc, Ti, Td);
    double sp = 1400 * (int(millis()/5000)%2);
    leftmotor.set_speed(sp);
    leftmotor.update();
    delay(10);
  }
}

void setup() {
  Serial.begin(115200);
  leftmotor.begin(ISR_A, ISR_B);
  leftmotor.pid.set_params(0.4, 0.4, 0);
  xTaskCreatePinnedToCore( loop2, "loop2", 10000, NULL, 1, NULL, 0);

  pinMode(36, INPUT);
  pinMode(37, INPUT);
  pinMode(38, INPUT);

  delay(2000);
}



