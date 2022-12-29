#ifndef MOTORLIB
#define MOTORLIB

#include "Arduino.h"
#include "Pinout.h"

class HBridgeChannel {
    private:
        const int ctrl1; //Control Pin 1
        const int ctrl2; // Control Pin 2
        const int pwm_pin; //PWM Pin
        const int pwm_channel; //Channel used in PWM pin

    public:
        /**
         @brief Class constructor
         @param PINS All pins and channel used for motor 
        */
        HBridgeChannel(Motor_Pins PINS);

        /**
         @brief Initialize all ports used for this H Bridge Channel
        */
        void begin();

        /**
         @brief Apply a duty cycle to PWM
         @param DUTY_CYCLE can be a value between -100 (-100%) and 100 (100%)
         @return Applied value in PWM, it's a value between -4095 and 4095;
        */
        int set_duty_cycle(double DUTY_CYCLE);
};

#define pulses_per_revolution 70.0
#define count_pulses_max 10 

class EncoderFase {
    private:
        volatile int main_pin;
        volatile int supp_pin;

        volatile int64_t last_pulse_time;
        volatile int64_t pasttime;
        volatile int count = 0;

        volatile double speed; 

    public:
        EncoderFase(Motor_Pins PINS, int FaseNumber);
        void begin(void ISR());
        void interrupt();
        void update();
        double get_speed();
};

class PID{
    private:
        double kp;
        double ki;
        double kd;

        double P = 0;
        double I = 0 ;
        double D = 0;
        double u;
        bool sat_flag = true;
        double past_error = 0;
        double past_time = 0;
        double past_y = 0;

    public:
        void set_params(double KP, double KI, double KD);
        double compute(double Y, double SP);
        void print();

};


#endif