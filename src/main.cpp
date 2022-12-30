#include <Arduino.h>
#include "Motor.h"

const Motor_Pins Pins  = {21, 0, 22,  17,  25,  26};
Motor leftmotor(Pins);

void ISR_A() {leftmotor.encFaseA.interrupt();} 
void ISR_B() {leftmotor.encFaseB.interrupt();}

void loop() {
  leftmotor.pid.print();
  delay(50);
}

void loop2(void* Param) {
  while (true) {  
    double sp = 1000 * (int(millis()/10000)%2);
    leftmotor.set_speed(sp);
    leftmotor.update();
    delay(10);
  }
}

void setup() {
  Serial.begin(115200);
  leftmotor.begin(ISR_A, ISR_B);
  leftmotor.pid.set_params(0.15, 0.1, 0);
  xTaskCreatePinnedToCore( loop2, "loop2", 10000, NULL, 1, NULL, 0);
  delay(2000);
}



