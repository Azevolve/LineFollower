#include "Motor.h"

FilteredVariable::FilteredVariable(double TAU):
    tau(TAU){}

double FilteredVariable::get(double X){
    double delta_time = (esp_timer_get_time()-past_time)*1e-6;
    past_time = esp_timer_get_time();
    double den = delta_time + tau;
    y = tau/den*y + delta_time/den*X;
    return y;
}

double FilteredVariable::get(){
    return y;
}

void FilteredVariable::set(double TAU){
    tau = TAU;
}

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
    duty_cycle = DUTY_CYCLE;
    if (DUTY_CYCLE == 0){
        digitalWrite(ctrl1, LOW);
        digitalWrite(ctrl2, LOW);
        ledcWrite(pwm_channel, 0);
        return 0;
    }
    
    if (DUTY_CYCLE > 0){
        digitalWrite(ctrl1, HIGH);
        digitalWrite(ctrl2, LOW);
    } else if (DUTY_CYCLE < 0){
        digitalWrite(ctrl1, LOW);
        digitalWrite(ctrl2, HIGH);
    }

    int value_pwm = abs(DUTY_CYCLE)/100.0*(4095.0 - deadzone) + deadzone;
    if (value_pwm >  4095) value_pwm = 4095;
    if (value_pwm < 0) value_pwm = 0;

    ledcWrite(pwm_channel, value_pwm);
    return value_pwm;
}

double HBridgeChannel::get_duty_cycle(){
    return duty_cycle;
}

void HBridgeChannel::set_deadzone(int PWM_VALUE){
    deadzone = PWM_VALUE;
}

EncoderFase::EncoderFase(Motor_Pins PINS, int FaseNumber): speed(0.1){
    if (!FaseNumber){
        main_pin = PINS.ENC_A;
        supp_pin = PINS.ENC_B;
    } else {
        main_pin = PINS.ENC_B;
        supp_pin = PINS.ENC_A;
    }
}

void EncoderFase::begin(void ISR()){
    pinMode(main_pin, INPUT_PULLDOWN);
    attachInterrupt(main_pin, ISR, RISING);
}

double EncoderFase::get_speed(){
    return speed.get();
}

void EncoderFase::interrupt(){
    if ((esp_timer_get_time() - last_pulse_time) < 250) return;
    last_pulse_time = esp_timer_get_time();

    //count += digitalRead(supp_pin)?1:-1;
    count++;

    if (abs(count) >= max_count){
        double delta_time = double(esp_timer_get_time() - pasttime)*1e-6;
        double counts_per_second = double(count)/delta_time;
        double omega = counts_per_second/pulses_per_revolution * 60.0;
        speed.get(omega);
        count = 0;
        pasttime = esp_timer_get_time();

        max_count = abs(speed.get())/100.0;
        if (max_count < 1) max_count = 1;
        else if (max_count > 15) max_count = 15;
    }
}

void EncoderFase::update(){
    double delta_time = esp_timer_get_time() - pasttime;
    if (delta_time < 10000) return;
    double counts_per_second = count/delta_time;
    double omega = counts_per_second/pulses_per_revolution*60.0;
    speed.get(omega); 
    count = 0;
    max_count = 1;
    pasttime = esp_timer_get_time(); 
}

void EncoderFase::set_tau(double TAU){
    speed.set(TAU);
}
    
Motor_PID::Motor_PID(): fD(0.01){}

double Motor_PID::compute(double Y, double SP){
    //Read Block
    error = SP-Y; 
    setpoint = SP;
    double delta_time = (esp_timer_get_time() - past_time)*1e-6;
    double dy = Y-past_y;

    //Computation Block
    P = error*kp;
    I += sat_flag*ki*(error+past_error)/2.0*delta_time;
    D = fD.get(-dy/delta_time*kd);

    u = P + I + D;

    //Saturation Block
    if(u > 100.0){
        u = 100.0;
        sat_flag = false;
    } else if (u < -100.0){
        u = -100.0;
        sat_flag = false;
    }  else {
        sat_flag = true;
    }

    //Update Block
    past_y = Y;
    past_error = error;
    past_time = esp_timer_get_time();
    return u;
}

void Motor_PID::set_params(double KC, double TI, double TD, double TAU_D){
    kp = KC;
    ki = KC/TI;
    kd = KC*TD;
    fD.set(TAU_D);
}

void Motor_PID::print(){
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
    Serial.print("E:");
    Serial.print(error);
    Serial.print(",");
    Serial.print("SP:");
    Serial.println(setpoint);
}

Motor_PID_status Motor_PID::get(){
    Motor_PID_status data;
    data.P = P;
    data.I = I;
    data.D = D;
    data.error = error;
    data.sat_flag = sat_flag;
    data.setpoint = setpoint;
    data.u = u;
    return data;
}

void Motor_PID::set_I(double I_){
    I = I_;
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
    hbridge.set_deadzone(250);
}

void Motor::update(){
    encFaseA.update();
    encFaseB.update();
    double y = get_speed();
    double u = pid.compute(y, setpoint);
    if (!controller_enable) return;
    hbridge.set_duty_cycle(u);
}

void Motor::enable_control(bool ENABLE){
    controller_enable = ENABLE;
    if (!controller_enable) return;
    pid.set_I(hbridge.get_duty_cycle());
}