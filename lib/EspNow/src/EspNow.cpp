#include "EspNow.h"

double EspNow::saturation(double VALUE){
    if (VALUE >  3000.0) return  3000.0;
    if (VALUE < -3000.0) return -3000.0;
    return VALUE;
}
void EspNow::receive(ctrl_t &DATA){
    l_sp = DATA.l_sp / 10.0;
    r_sp = DATA.r_sp / 10.0;
}
void EspNow::receive(modu_t &DATA){
    l_speed = DATA.l_speed / 10.0;
    r_speed = DATA.r_speed / 10.0;
    ir = DATA.ir;
}

void EspNow::send(ctrl_t &DATA){
    DATA.l_sp = l_sp*10.0;
    DATA.r_sp = r_sp*10.0;
}
void EspNow::send(modu_t &DATA){
    DATA.l_speed = l_speed*10;
    DATA.r_speed = r_speed*10;
    DATA.ir = ir;
}

EspNow::EspNow(bool ISITCTRL, const uint8_t CTRL_ADDRESS[], const uint8_t MODU_ADDRESS[]): 
     ctrl_macaddress(CTRL_ADDRESS), modu_macaddress(MODU_ADDRESS), IsItCtrl(ISITCTRL){}

void EspNow::set(double L_SP, double R_SP){
    r_sp = saturation(R_SP);
    l_sp = saturation(L_SP);
}
void EspNow::set(double L_SPEED, double R_SPEED, uint8_t IR){
    l_speed = saturation(L_SPEED);
    r_speed = saturation(R_SPEED);
    ir = IR;
}

double EspNow::get_l_speed() {return l_speed;}
double EspNow::get_l_sp()    {return l_sp;}
double EspNow::get_r_speed() {return r_speed;}
double EspNow::get_r_sp()    {return r_sp;}
uint8_t EspNow::get_ir()     {return ir;}

esp_err_t EspNow::send(){
    if(esp_timer_get_time() > (pasttime + 500000)){
    pasttime = esp_timer_get_time();
    fail_packets_counter = 0; 
    }
    if(fail_packets_counter >= 10) return ESP_ERR_ESPNOW_INTERNAL; 
    if(IsItCtrl){
    ctrl_t data;
    send(data);
    return esp_now_send(modu_macaddress, (uint8_t *)&data, sizeof(data));
    } else {
    modu_t data;
    send(data);
    return esp_now_send(ctrl_macaddress, (uint8_t *)&data, sizeof(data));
    }
}
void EspNow::receive(const uint8_t *DATA, int len){
    if(IsItCtrl){
    modu_t data;
    if (len != sizeof(data)) return;
    memcpy(&data, DATA, sizeof(data));
    receive(data);
    } else {
    ctrl_t data;
    if (len != sizeof(data)) return;
    memcpy(&data, DATA, sizeof(data));
    receive(data);
    }
}

void EspNow::begin(){
    WiFi.mode(WIFI_STA);
    if(esp_now_init() != ESP_OK){
        while (true)  {
        Serial.println("Error Initializing ESP_NOW");
        }
    }

    memcpy(peerInfo.peer_addr, IsItCtrl?modu_macaddress:ctrl_macaddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        while (true)  {
        Serial.println("Failed to add peer");
        }
    }
}
void EspNow::set_cb_sent(void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)){
    esp_now_register_send_cb(OnDataSent);
}
void EspNow::set_cb_receive(void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)){
    esp_now_register_recv_cb(OnDataRecv);
}
void EspNow::cb_sent(esp_now_send_status_t status){
    if (status != ESP_NOW_SEND_SUCCESS) fail_packets_counter++;
}
