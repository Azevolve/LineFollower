#include <Arduino.h>
#include "Pinout.h"
#include "LineSensors.h"
#include "Motor.h"

const Motor_Pins Pins  = {21, 0, 22,  17,  25,  26};
HBridgeChannel hbridge (Pins);
EncoderFase encoder(Pins, 0);
EncoderFase encoder1(Pins, 1);

PID pid;

void interrupt(){
  encoder.interrupt();
}
void interrupt1(){
  encoder1.interrupt();
}

void loop() {
  pid.print();
  delay(50);
}

void loop2(void* Param) {
  while (true) {  
    encoder.update();
    encoder1.update();

    double y = (encoder.get_speed() - encoder1.get_speed())/2.0;
    
    double sp = 1000 * (int(millis()/10000)%2);
    double u = pid.compute(y, sp);
    hbridge.set_duty_cycle(u);
    delay(10);
  }
}

void setup() {
  Serial.begin(115200);
  pid.set_params(0.15, 0, 0);

  xTaskCreatePinnedToCore(
    loop2,
    "loop2",
    10000,
    NULL,
    1,
    NULL,
    0
  );

  hbridge.begin();
  encoder.begin(interrupt);
  encoder1.begin(interrupt1);
  delay(2000);
}



