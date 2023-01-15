#include "EspNow.h"
#include "Motor.h"
#include "LineSensors.h"

EspNow espnow(false, ctrl_mac, modu_mac);
Motor leftmotor(LeftMotorPins);
Motor rightmotor(RightMotorPins);
LineSensors ir(IR_pins);

void lenc_fase_a(){leftmotor.encFaseA.interrupt();}
void lenc_fase_b(){leftmotor.encFaseB.interrupt();}

void renc_fase_a(){rightmotor.encFaseA.interrupt();}
void renc_fase_b(){rightmotor.encFaseB.interrupt();}

int64_t past_time_receive = esp_timer_get_time();
bool timeout_flag = false;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  espnow.receive(incomingData, len);
  past_time_receive = esp_timer_get_time();
  timeout_flag = false;
  leftmotor.set_speed(espnow.get_l_sp());
  rightmotor.set_speed(espnow.get_r_sp());
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  espnow.cb_sent(status);
}

void loop(){
  espnow.set(leftmotor.get_speed(), rightmotor.get_speed(), ir.get());
  espnow.send();
  delay(5);
}

void loop2(void *Pv){
  delay(200);
  leftmotor.begin(lenc_fase_a, lenc_fase_b);
  leftmotor.pid.set_params(0.3, 0.140, 0.038);
  leftmotor.set_speed(0);

  rightmotor.begin(renc_fase_a, renc_fase_b);
  rightmotor.pid.set_params(0.3, 0.140, 0.02);
  rightmotor.set_speed(0);

  while (true) {
    if((esp_timer_get_time() > (past_time_receive + 500000)) && (!timeout_flag)){
      timeout_flag = true;
      leftmotor.set_speed(0);
      rightmotor.set_speed(0);
    }

    leftmotor.update();
    rightmotor.update();
    delay(1);
  }
}

void setup(){
  espnow.begin();
  espnow.set_cb_sent(OnDataSent);
  espnow.set_cb_receive(OnDataRecv);

  xTaskCreatePinnedToCore( loop2, "loop2", 10000, NULL, 1, NULL, 0);
  delay(500);
}

