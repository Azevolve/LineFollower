#include <Arduino.h>
#include "Motor.h"
#include "Pinout.h"

//const Motor_Pins motor_pins = {D19, 0, D27, D26, D15, D14};

Motor motor (RightMotorPins);

void enc_fase_a(){motor.encFaseA.interrupt();}
void enc_fase_b(){motor.encFaseB.interrupt();}

void loop2(void *pV){
    while (true) {
        motor.update();
        delay(1);   
    }
}
void loop3(void *pV){
    while (true) {
        motor.set_speed(500);
        delay(5000);
        motor.set_speed(1200);
        delay(5000);
    }
};

void setup(){
    Serial.begin(115200);
    motor.begin(enc_fase_a, enc_fase_b);
    motor.pid.set_params(0.3, 0.140, 0.038);
    
    xTaskCreatePinnedToCore(
        loop2,
        "loop2",
        10000,
        NULL,
        1,
        NULL,
        0
    );
    xTaskCreatePinnedToCore(
        loop3,
        "loop3",
        10000,
        NULL,
        2,
        NULL,
        1
    );
    
}

void loop(){
    motor.pid.print();
    delay(20);
}