#include "Motor.h"

FilteredVariable::FilteredVariable(){}

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
    if (abs(duty_cycle) <= 3.0){
        digitalWrite(ctrl1, LOW);
        digitalWrite(ctrl2, LOW);
        pwm = 0;
        ledcWrite(pwm_channel, 0);
        return 0;
    }
    
    if (duty_cycle > 0){
        digitalWrite(ctrl1, HIGH);
        digitalWrite(ctrl2, LOW);
    } else if (duty_cycle < 0){
        digitalWrite(ctrl1, LOW);
        digitalWrite(ctrl2, HIGH);
    }

    int value_pwm = abs(duty_cycle)/100.0*(4095.0 - deadzone) + deadzone;
    if (value_pwm >  4095) value_pwm = 4095;
    if (value_pwm < 0) value_pwm = 0;
    pwm = value_pwm;
    ledcWrite(pwm_channel, pwm);
    return pwm;
}

double HBridgeChannel::get_duty_cycle(){
    return duty_cycle;
}

int HBridgeChannel::get_pwm(){
    return pwm;
}

void HBridgeChannel::set_deadzone(int PWM_VALUE){
    deadzone = PWM_VALUE;
}

EncoderFase::EncoderFase(Motor_Pins PINS, int FaseNumber): speed(0.03), wise(0.15){
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

void EncoderFase::add_to_speed(double DELTA_TIME_US){
        double omega = 0;
        if(motor_turned_on){
            double counts_per_second = double(count)/(DELTA_TIME_US*1e-6);
            omega = counts_per_second/pulses_per_revolution * 60.0;
            if (wise.get() < 0.0) omega = -omega;
        }
        speed.get(omega);
        count = 0;
        pasttime = esp_timer_get_time();
}

void EncoderFase::interrupt(){
    if ((esp_timer_get_time() - last_pulse_time) < 250) return;
    last_pulse_time = esp_timer_get_time();

    wise.get(digitalRead(supp_pin)?1:-1);
    count++;

    if (count >= max_count){
        double delta_time = esp_timer_get_time() - pasttime;
        add_to_speed(delta_time);

        max_count = abs(speed.get())/50.0;
        if (max_count < 7) max_count = 7;
        else if (max_count > 20) max_count = 20;
    }
}

void EncoderFase::update(){
    double delta_time = esp_timer_get_time() - pasttime;
    if (delta_time < 10000) return;
    add_to_speed(delta_time);
    max_count = 7;
}

void EncoderFase::set_tau(double TAU){
    speed.set(TAU);
}
    
void EncoderFase::set_motor_state(bool TURNEDON){
    motor_turned_on = TURNEDON;
}

double InstabilityCounter::get(double Y){
    double dy = abs(Y - past_y);
    past_y = Y;

    //Add to circular buffer
    index = (index+1)%circularbuffersize;
    double past_dy = buffer[index];
    buffer[index] =  dy;

    X = X + dy - past_dy;
    double value = gama.get(X);

    return value;
}

double InstabilityCounter::get(){
    return gama.get();
}

void InstabilityCounter::set(double TAU){
    gama.set(TAU);
}

Motor_PID::Motor_PID(): fD(0.01){}

double Motor_PID::compute(double Y, double SP){
    //Read Block
    error = SP-Y; 
    setpoint = SP;
    double delta_time = (esp_timer_get_time() - past_time)*1e-6;
    double dy = Y-past_y;

    instability.get(Y);
    //double kp_ = SP<=500?kp/2:kp;

    //Computation Block
    P = error*kp;
    I += sat_flag*ki*(error+past_error)/2.0*delta_time;
    //D = fD.get((error-past_error)/delta_time*kd);
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
    data.gama = instability.get();
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
    int hbridgevalue = hbridge.get_pwm(); 
    encFaseA.set_motor_state(hbridgevalue != 0);
    encFaseB.set_motor_state(hbridgevalue != 0);

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