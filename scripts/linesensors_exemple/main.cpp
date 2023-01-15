#include "Arduino.h"
#include "LineSensors.h"

IR_Pins ir_pins = {VP, VN, D34, D35, D32, D33, D25, D22};
LineSensors ir (ir_pins);

void setup(){
    Serial.begin(115200);
    ir.begin();
}
void loop(){
    byte buffer[8];
    ir.get(buffer, 8);
    Serial.println();
    for (int i = 0; i < 8; i++){
        Serial.printf("%d \t", buffer[i]);
    }
    delay(100);
}