#include "Arduino.h"
#include "Motor.h"
#include "Pinout.h"
#include "Protocol_LabVIEW_Arduino.h"
#include "Dashboard.h"


Motor left(LeftMotorPins);
Motor right(RightMotorPins);

byte buffer[50];
LABVIEW labview(buffer, sizeof(buffer));
Dashboard dashboard;

void lenc_fase_a(){left.encFaseA.interrupt();}
void lenc_fase_b(){left.encFaseB.interrupt();}
void renc_fase_a(){right.encFaseA.interrupt();}
void renc_fase_b(){right.encFaseB.interrupt();}

bool manual_control = 1;
bool control_flag = 0;

void loop2(void *pv){
   labview.begin();
   left.enable_control(false);
   right.enable_control(false);
   left.set_speed(0);
   right.set_speed(0);

   while(true){
    if (labview.new_infos()){
        RAW_DATA data = labview.get_data();
        esp_ctrl_data commands = dashboard.get(data);

        double pwm = commands.r_sp;
        //double tau_en = commands.tau_error;
        //motor.encFaseA.set_tau(tau_en);
        //motor.encFaseB.set_tau(tau_en);

        if ((!commands.manual_control) && (manual_control)){
            right.enable_control(true);
            left.enable_control(true);
            control_flag = 1;
        }
        if ((commands.manual_control) && (!manual_control)){
            right.enable_control(false);
            left.enable_control(false);
            control_flag = 0;
        }
        manual_control = commands.manual_control;

        double setpoint = commands.l_sp;

        int16_t applied_pwm = 0;

        if (manual_control){
            if(pwm > 100.0) pwm = 100;
            if(pwm < -100.0) pwm = -100;
            applied_pwm = left.hbridge.set_duty_cycle(pwm);
            right.hbridge.set_duty_cycle(pwm);
        } else {
            // double kc = commands.kc;
            // double ti = commands.ti;
            double td = commands.td;
            left.pid.fu.set(td);
            right.pid.fu.set(td);
            // double taud = commands.tau_d;
            // motor.pid.set_params(kc, ti, td, taud);
            left.set_speed(setpoint);
            right.set_speed(setpoint);
        }

        while (labview.new_infos()){
            labview.get_data();
        }

        esp_status_data status;
        //Motor_PID_status pid_status = motor.pid.get();

        // if (manual_control){
        //     status.l_sp = left.hbridge.get_duty_cycle()*10;
        //     status.r_sp = right.hbridge.get_duty_cycle()*10;
        // } else {
        //     //status.l_sp = pid_status.setpoint;
        //     //status.r_sp = pid_status.setpoint;
        // }


        status.l_sp = left.encFaseA.get_speed();
        status.l_speed = left.encFaseB.get_speed();

        status.r_sp = right.encFaseA.get_speed();
        status.r_speed = right.encFaseB.get_speed();
        //status.sat_flag = pid_status.sat_flag;
        status.P = left.pid.fu.get();
        status.I = right.pid.fu.get();

        //status.I = pid_status.I;
        //status.D = pid_status.D;
        //status.u = pid_status.u;
        //status.error = pid_status.error;
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
    left.begin(lenc_fase_a, lenc_fase_b);
    left.pid.set_params(0.5, 0.1, 0.03, 0.1);
    right.begin(renc_fase_a, renc_fase_b);
    right.pid.set_params(0.5, 0.1, 0.03, 0.1);
}

int64_t past_time_1 = esp_timer_get_time();
int64_t past_time_2 = esp_timer_get_time();

void loop(){
    if(esp_timer_get_time() >= (past_time_1 + 1000)){
        past_time_1 = esp_timer_get_time();
        left.update();
    }
    if(esp_timer_get_time() >= (past_time_2 + 1000)){
        past_time_2 = esp_timer_get_time();
        right.update();
    }
}