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

int16_t applied_pwml = 0;
int16_t applied_pwmr = 0;

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

        if (manual_control){
            if(pwm > 100.0) pwm = 100;
            if(pwm < -100.0) pwm = -100;
            applied_pwml = left.hbridge.set_duty_cycle(pwm);
            //applied_pwmr = right.hbridge.set_duty_cycle(pwm);
        } else {
            double tu = commands.tau_error;
            left.pid.instability.set(tu);
            //left.pid.fu.set(tu);
            //right.pid.fu.set(tu);
            left.set_speed(setpoint);
            //right.set_speed(setpoint);
        }

        while (labview.new_infos()){
            labview.get_data();
        }


        esp_status_data status;
        status.l_sp = left.encFaseA.get_speed();
        status.l_speed = -left.encFaseB.get_speed();
        //status.l_sp = right.pid.get().gama;

        //status.r_sp = right.encFaseA.get_speed();
        //status.r_speed = -right.encFaseB.get_speed();
        status.r_sp = left.pid.instability.get();

        //status.P = left.pid.fu.get();
        //status.I = right.pid.fu.get();
        status.A_ = applied_pwml;
        status.B_ = applied_pwmr;
        status.sat_flag = right.encFaseA.motor_turned_on;

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
    left.pid.set_params(0.4, 0.1, 0.03, 0.05);
    right.begin(renc_fase_a, renc_fase_b);
    right.pid.set_params(0.4, 0.1, 0.03, 0.05);
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