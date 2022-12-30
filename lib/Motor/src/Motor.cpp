#include "Motor.h"

HBridgeChannel::HBridgeChannel(Motor_Pins PINS):
    ctrl1(PINS.CTROl_1), ctrl2(PINS.CTROL_2), 
    pwm_pin(PINS.PWM_PIN), pwm_channel(PINS.PWM_CHANNEL) {}

void HBridgeChannel::begin(){
    pinMode(ctrl1, OUTPUT);
    pinMode(ctrl2, OUTPUT);
    pinMode(pwm_pin, OUTPUT);

    ledcSetup(pwm_channel, 5000, 12);
    ledcAttachPin(pwm_pin, pwm_channel);
    set_duty_cycle(0);
}

int HBridgeChannel::set_duty_cycle(double DUTY_CYCLE){    
    if (DUTY_CYCLE == 0){
        digitalWrite(ctrl1, LOW);
        digitalWrite(ctrl2, LOW);
        ledcWrite(pwm_channel, 0);
        return 0;
    }

    if (DUTY_CYCLE >  100.0) DUTY_CYCLE = 100.0;
    if (DUTY_CYCLE < -100.0) DUTY_CYCLE = -100.0;
    
    int value_pwm = DUTY_CYCLE*40.95;
    if (value_pwm > 0){
        digitalWrite(ctrl1, HIGH);
        digitalWrite(ctrl2, LOW);
        ledcWrite(pwm_channel, value_pwm);
        return value_pwm;
    } 
    if (value_pwm < 0){
        digitalWrite(ctrl1, LOW);
        digitalWrite(ctrl2, HIGH);
        ledcWrite(pwm_channel, -value_pwm);
        return value_pwm;
    }
    return 0;
}

EncoderFase::EncoderFase(Motor_Pins PINS, int FaseNumber){
    if (!FaseNumber){
        main_pin = PINS.ENC_A;
        supp_pin = PINS.ENC_B;
    } else {
        main_pin = PINS.ENC_B;
        supp_pin = PINS.ENC_A;
    }
}

void EncoderFase::begin(void ISR()){
    pinMode(main_pin, INPUT);
    attachInterrupt(main_pin, ISR, RISING);
}

double EncoderFase::get_speed(){
    return speed;
}

void EncoderFase::interrupt(){
    if ((esp_timer_get_time() - last_pulse_time) < 215) return;
    last_pulse_time = esp_timer_get_time();

    count += digitalRead(supp_pin)?1:-1;
    if ((abs(count) == max_count) || (count == 0)){
        double delta_time = (esp_timer_get_time() - pasttime)*1e-6;
        double counts_per_second = count/delta_time;
        double omega = counts_per_second/pulses_per_revolution * 60.0;
        speed = 0.9*speed + 0.1*omega;
        count = 0;
        pasttime = esp_timer_get_time();
    }

    max_count = 10.0*abs(speed)/1000.0;
    if (max_count < 1) max_count = 1;
    else if (max_count > 15) max_count = 15;
}

void EncoderFase::update(){
    double delta_time = esp_timer_get_time() - pasttime;
    if (delta_time < 20000) return;
    double counts_per_second = count/delta_time;
    double omega = counts_per_second/pulses_per_revolution*60.0;
    speed = 0.6*speed + 0.4*omega; //100Hz
    count = 0;
    max_count = 1;
    pasttime = esp_timer_get_time(); 
}

FilteredVariable::FilteredVariable(double TAU):
    tau(TAU){}

double FilteredVariable::get(double X){
    double delta_time = (esp_timer_get_time()-past_time)*1e-6;
    double den = delta_time + tau;
    y = tau/den*y + delta_time/den*X;
    return y;
}

double PID::compute(double Y, double SP){
    double error = SP - Y;
    double delta_time = (esp_timer_get_time() - past_time)*1e-6;
    double dy = Y-past_y;

    P = error*kp;
    I += sat_flag*ki*(error+past_error)/2.0*delta_time;
    D = 0.9*D + 0.1*(-dy/delta_time*kd); //100Hz

    u = P + I + D;

    if(u > 100.0){
        u = 100.0;
        sat_flag = false;
    } else if (u < -100.0){
        u = -100.0;
        sat_flag = false;
    }  else {
        sat_flag = true;
    }

    past_y = Y;
    past_error = error;
    past_time = esp_timer_get_time();
    return u;
}

void PID::set_params(double KC, double TI, double TD){
    kp = KC;
    ki = KC/TI;
    kd = KC*TD;
}

void PID::print(){
    Serial.print("P:");
    Serial.print(P);
    Serial.print(",");
    Serial.print("I:");
    Serial.print(I);
    Serial.print(",");
    Serial.print("D:");
    Serial.print(D);
    Serial.print(",");
    Serial.print("Y:");
    Serial.print(past_y);
    Serial.print(",");
    Serial.print("SP:");
    Serial.println(past_y+past_error);
}

Motor::Motor(Motor_Pins PINS):
    pins(PINS), encFaseA(PINS, 0), encFaseB(PINS, 1), hbridge(PINS) {}

double Motor::get_speed(){
    return (encFaseA.get_speed() - encFaseB.get_speed())/2.0;
}

void Motor::set_speed(double SETPOINT){
    setpoint = SETPOINT;
}

void Motor::begin(void ISRA(), void ISRB()){
    hbridge.begin();
    encFaseA.begin(ISRA);
    encFaseB.begin(ISRB);
}

void Motor::update(){
    encFaseA.update();
    encFaseB.update();
    double y = get_speed();
    double u = pid.compute(y, setpoint);
    hbridge.set_duty_cycle(u);
}