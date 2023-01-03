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
  sum /= 40950.0;
  return sum;
}

void loop() {
  //double sp = read_(36) * 2800 - 1400;
  //leftmotor.set_speed(sp);
  //leftmotor.pid.print();
  //delay(50);
  // double Kc = read_(36) * 0.5;
  // double Ti = read_(37) * 0.2;
  // if (Ti <= 0.01) Ti = 0.01;
  // double Td = read_(38) * 0.5;
  // leftmotor.pid.set_params(Kc, Ti, Td);
  double sp = 1400 * (int(millis()/5000)%2);
  leftmotor.set_speed(sp);
  leftmotor.pid.print();
  delay(50);
}

void loop2(void* Param) {
  while (true) {  
    leftmotor.update();
    delay(1);
  }
}

void setup() {
  Serial.begin(115200);
  leftmotor.begin(ISR_A, ISR_B);
  xTaskCreatePinnedToCore( loop2, "loop2", 10000, NULL, 1, NULL, 0);

  pinMode(36, INPUT);
  pinMode(37, INPUT);
  pinMode(38, INPUT);

  delay(2000);
}



