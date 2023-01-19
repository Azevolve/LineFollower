#include "Arduino.h"
#include "Motor.h"
#include "Pinout.h"
#include "Protocol_LabVIEW_Arduino.h"
#include "Dashboard.h"



const Motor_Pins motor_pins = {D19, 0, D26, D27, D15, D14};
Motor motor(motor_pins);
byte buffer[50];
LABVIEW labview(buffer, sizeof(buffer));
Dashboard dashboard;

void enc_fase_a(){motor.encFaseA.interrupt();}
void enc_fase_b(){motor.encFaseB.interrupt();}

bool manual_control = 1;
bool control_flag = 0;

void loop2(void *pv){
   labview.begin();
   motor.enable_control(false);
   motor.set_speed(0);

   while(true){
    if (labview.new_infos()){
        RAW_DATA data = labview.get_data();
        esp_ctrl_data commands = dashboard.get(data);

        double pwm = commands.r_sp;
        //double tau_en = commands.tau_error;
        //motor.encFaseA.set_tau(tau_en);
        //motor.encFaseB.set_tau(tau_en);

        if ((!commands.manual_control) && (manual_control)){
            motor.enable_control(true);
            control_flag = 1;
        }
        if ((commands.manual_control) && (!manual_control)){
            motor.enable_control(false);
            control_flag = 0;
        }
        manual_control = commands.manual_control;

        double setpoint = commands.l_sp;

        int16_t applied_pwm = 0;

        if (manual_control){
            if(pwm > 100.0) pwm = 100;
            if(pwm < -100.0) pwm = -100;
            applied_pwm = motor.hbridge.set_duty_cycle(pwm);
        } else {
            double kc = commands.kc;
            double ti = commands.ti;
            double td = commands.td;
            double taud = commands.tau_d;
            motor.pid.set_params(kc, ti, td, taud);
            motor.set_speed(setpoint);
        }

        while (labview.new_infos()){
            labview.get_data();
        }

        esp_status_data status;
        Motor_PID_status pid_status = motor.pid.get();

        if (manual_control){
            status.l_sp = motor.hbridge.get_duty_cycle()*10;
            status.r_sp = motor.hbridge.get_duty_cycle()*10;
        } else {
            status.l_sp = pid_status.setpoint;
            status.r_sp = pid_status.setpoint;
        }

        status.l_speed = motor.encFaseA.get_speed();
        status.r_speed = motor.encFaseB.get_speed();
        status.sat_flag = pid_status.sat_flag;
        status.P = pid_status.P;
        status.I = pid_status.I;
        status.D = pid_status.D;
        status.u = pid_status.u;
        status.error = pid_status.error;
        status.A_ = applied_pwm;

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