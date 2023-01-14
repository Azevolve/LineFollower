#include "EspNow.h"

#define ISITCTRL true

const uint8_t ctrl_mac[] = {0x3C, 0x61, 0x05, 0x0C, 0x41, 0x68}; //pinos curtos
const uint8_t modu_mac[] = {0x3C, 0x61, 0x05, 0x0D, 0x46, 0x80}; //Pinos longos

uint8_t broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

EspNow espnow(ISITCTRL, ctrl_mac, modu_mac);

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  espnow.receive(incomingData, len);
  #if ISITCTRL == true
      Serial.printf("\n %f \t %f \t %d", espnow.get_l_speed(), espnow.get_r_speed(), espnow.get_ir());
  #else 
      int time = esp_timer_get_time()/1e6;   
      espnow.set(time*1, time*2, time*3);
      esp_err_t result = espnow.send();
  #endif
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  espnow.cb_sent(status);
}

void setup(){
  Serial.begin(115200);
  espnow.begin();
  espnow.set_cb_sent(OnDataSent);
  espnow.set_cb_receive(OnDataRecv);
}
 
int counter = 0;
void loop(){
  #if ISITCTRL == true
    espnow.set(millis()/1000, millis()/2000);
    esp_err_t result = espnow.send();
    
    if (result == ESP_OK) {
      //Serial.println("Sent with success");
    }
    else {
      counter++;
      Serial.println(counter);
    }
    delay(5);
  #endif
}