#include "EspNow.h"
#include "Protocol_LabVIEW_Arduino.h"
#include "Dashboard.h"

byte buffer[50];
LABVIEW labview(buffer, sizeof(buffer));
Dashboard dashboard;

const uint8_t modu_mac[] = {0xC4, 0x4F, 0x33, 0x65, 0xDE, 0x3D}; 
const uint8_t ctrl_mac[] = {0x3C, 0x61, 0x05, 0x0D, 0x46, 0x80};

EspNow espnow(true, ctrl_mac, modu_mac);
double lsp=0, rsp =0;

int64_t pasttime_labview = esp_timer_get_time();
bool timeout_flag = false;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  espnow.receive(incomingData, len);  
  espnow.set(lsp, rsp);
  espnow.send();
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  espnow.cb_sent(status);
}

void labview_loop(void *pv){
  labview.begin();
  while (true){
      if ((esp_timer_get_time() > (pasttime_labview + 500000)) && (!timeout_flag)){
        timeout_flag = true;
        lsp = 0;
        rsp = 0;
      }

      if (labview.new_infos()){
        timeout_flag = false;
        pasttime_labview = esp_timer_get_time();

        RAW_DATA data = labview.get_data();
        esp_ctrl_data commands = dashboard.get(data);
        lsp = commands.l_sp;
        rsp = commands.r_sp;

        while (labview.new_infos()) labview.get_data();

        esp_status_data status;

        status.l_sp =               commands.l_sp; 
        status.l_speed =            espnow.get_l_speed(); 
        status.r_sp =               commands.r_sp; 
        status.r_speed =            espnow.get_r_speed(); 
        status.error =              0; 
        status.filtered_error =     0; 
        status.P =                  0;
        status.I =                  0; 
        status.D =                  0; 
        status.u =                  0; 
        status.sat_flag =           0; 
        status.IR =                 espnow.get_ir();

        data = dashboard.set(status);
        labview.send_data(data);
      }
      delay(2);
  }
}

void setup(){
  xTaskCreatePinnedToCore(
    labview_loop,
    "labview",
    10000,
    NULL,
    1,
    NULL,
    0
  );
  delay(200);

  espnow.begin();
  espnow.set_cb_sent(OnDataSent);
  espnow.set_cb_receive(OnDataRecv);
}
 
void loop(){}