#include <Arduino.h>
#include "Protocol_LabVIEW_Arduino.h"
#include "Dashboard.h"

byte buffer[50];
LABVIEW labview(buffer, sizeof(buffer));
Dashboard dashboard;

void setup() {
  labview.begin();
}

double ir = 0;

void loop() {
  if(labview.new_infos()){

    RAW_DATA data = labview.get_data();
    esp_ctrl_data commands = dashboard.get(data);

    while(labview.new_infos()){
      labview.get_data();
    };

    esp_status_data status;

    status.l_sp =               commands.l_sp; 
    status.l_speed =            commands.r_sp; 
    status.r_sp =               commands.r_sp; 
    status.r_speed =            commands.l_sp; 
    status.error =              commands.tau_error; 
    status.filtered_error =     commands.tau_d; 
    status.P =                  commands.kp;
    status.I =                  commands.ki; 
    status.D =                  commands.kd; 
    status.u =                  commands.speed_ref; 
    status.sat_flag =           commands.manual_control; 
    
    ir += commands.turn_on*0.02;
    status.IR = ir; 

    data = dashboard.set(status);
    labview.send_data(data);
  }
}