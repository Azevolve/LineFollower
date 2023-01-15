#include "Arduino.h"
#include "Motor.h"
#include "Pinout.h"
#include "Protocol_LabVIEW_Arduino.h"
#include "Dashboard.h"

const Motor_Pins motor_pins = {D19, 1, D26, D27, D15, D14};
Motor motor(motor_pins);
byte buffer[50];
LABVIEW labview(buffer, sizeof(buffer));
Dashboard dashboard;

void enc_fase_a(){motor.encFaseA.interrupt();}
void enc_fase_b(){motor.encFaseB.interrupt();}

bool manual_control = 0;

void loop2(void *pv){
   labview.begin();
   while(true){
    if (labview.new_infos()){
        RAW_DATA data = labview.get_data();
        esp_ctrl_data commands = dashboard.get(data);

        double kc = commands.kc;
        double ti = commands.ti;
        double td = commands.td;
        double tau_d = commands.tau_d;

        double pwm = commands.r_sp/1500.0;
        double setpoint = commands.l_sp;

        if (commands.manual_control && (!manual_control))  motor.enable_control(true);
        if ((!commands.manual_control) && (manual_control)) motor.enable_control(false);
        manual_control = commands.manual_control;

        motor.pid.set_params(kc, ti, td, tau_d);

        if (manual_control){
            motor.hbridge.set_duty_cycle(pwm);
        } else {
            motor.set_speed(setpoint);
        }

        while (labview.new_infos()){
            labview.get_data();
        }

        esp_status_data status;
        Motor_PID_status pid_infos = motor.pid.get();

        status.P = pid_infos.P;
        status.I = pid_infos.I;
        status.D = pid_infos.D;
        status.error = pid_infos.error;
        status.sat_flag = pid_infos.sat_flag;
        status.l_sp = pid_infos.setpoint;
        status.r_sp = pid_infos.setpoint;
        status.u = pid_infos.u;

        status.l_speed = motor.encFaseA.get_speed();
        status.r_speed = motor.encFaseB.get_speed();

        data = dashboard.set(status);
        labview.send_data(data);
    }
    delay(2);
   }
}

void setup(){
    xTaskCreatePinnedToCore(
        loop2,
        "loop2",
        10000,
        NULL,
        1,
        NULL,
        0
    );
    delay(200);
    motor.begin(enc_fase_a, enc_fase_b);

}

void loop(){
    motor.update();
    delay(1);
}