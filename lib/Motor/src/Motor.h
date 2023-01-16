#ifndef MOTORLIB
#define MOTORLIB

#include "Arduino.h"
#include "Pinout.h"

class FilteredVariable{
    private:
        double tau; //Time constant
        int64_t past_time; //Last time compute
        double y; //Filtered Variable 
    public:
        /**
         @brief Class Constructor
         @param TAU Time constant 
        */
        FilteredVariable(double TAU);

        /**
         @brief Get the filtered variable
         @param X New value
         @return Filtered Variable 
        */ 
        double get(double X);

        /**
         @brief Get the filtered variable, it doesn't update the variable
         @return Filtered Variable
        */
        double get();

        /**
         @brief Set Time Constant
         @param TAU Time Constant
        */
        void set(double TAU);
};

class HBridgeChannel {
    private:
        const int ctrl1; //Control Pin 1
        const int ctrl2; // Control Pin 2
        const int pwm_pin; //PWM Pin
        const int pwm_channel; //Channel used in PWM pin
        int deadzone = 0; //Band which the motor doesn't answer
        double duty_cycle = 0; //Duty Cycle

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

        /**
         @brief Set the deadzone, band which the motor doesn't answer
         @param PWM_VALUE Min value applicable in PWM 
        */
        void set_deadzone(int PWM_VALUE);

        /**
         @brief Get the last duty cycle apllied
        */
        double get_duty_cycle();
};

#define pulses_per_revolution 70.0 //Pulses per revolution according encoder's hardware and motor gearbox 

class EncoderFase {
    private:
        volatile int main_pin; //Pin used in interrupt function
        volatile int supp_pin; //Pin used to get the motor spin way 
        volatile int max_count = 1; //Cumpute Trigger 

        volatile int64_t last_pulse_time; //Variable used as antibouncing, considering the period between each pulse in 4000rpm
        volatile int64_t pasttime; //Variable used to compute motor's speed
        volatile int count = 0; //Pulse Counter

        FilteredVariable speed;

    public:
        /**
         @brief Class Constructor 
         @param PINS All motor pins
         @param FaseNumber 0 to first encoder; 1 to other encoder.
        */
        EncoderFase(Motor_Pins PINS, int FaseNumber);

        /**
         @brief Start the encoder 
         @param ISR Declared function contains the method interrupt() of this object.
        */
        void begin(void ISR());

        /**
         @brief Function what must be called inside other main code function 
        */
        void interrupt();

        /**
         @brief Update the speed considering mainly the elipsed time instead the pulse counter. This is used to low speed.
        */
        void update();

        /**
         @brief Get motor's speed.
         @return Motor's speed, in rpm
        */
        double get_speed();

        /**
         @brief Set Time Constant Filter
         @brief Time Constant
        */
        void set_tau(double TAU);
};

struct Motor_PID_status {
    double P;
    double I;
    double D;
    double u;
    bool sat_flag;
    double error;
    double setpoint;
};

class Motor_PID{
    private:
        double kp = 0;  //Proportional Gain
        double ki = 0;  //Integrative Gain
        double kd = 0;  //Derivative Gain

        double P = 0; //Proportional Partial
        double I = 0; //Integrative Partial 
        double D = 0; //Derivative Partial

        double u;   //Output value (in percent)
        bool sat_flag = true; //Anti-windup flag
        double past_error = 0;  //previous iteration error 
        double past_time = 0;   //previous iteration time
        double past_y = 0;  //previous iteration controlled variable
        double error = 0; //Actual error 
        double setpoint = 0; //Setpoint 
        FilteredVariable fD;    //Filter applied in derivative partial, 

    public:
        Motor_PID();

        /**
         @brief Set controller parameters 
         @param KC Proportional Gain
         @param TI Integrative Time
         @param TD Derivative Gain
         @param TAU_D Time Constant Filter of Derivative Partial
        */
        void set_params(double KC, double TI, double TD, double TAU_D);

        /**
         @brief Compute an iteration of the controller 
         @param Y Controlled Variable
         @param SP Setpoint   
        */
        double compute(double Y, double SP);

        /**
         @brief Print all data to Plotter Serial
        */
        void print();

        /**
         @brief Get last iteration data
        */
        Motor_PID_status get();

        /**
         @brief Set Integrative Partial
         @param I Integrative Partial
        */
        void set_I(double I_);

};

class Motor{
    private:
        Motor_Pins pins; //All motor pins 
        double setpoint;  //Speed setpoint 
    public:
        EncoderFase encFaseA; //Encoder Fase A
        EncoderFase encFaseB;   //Encoder Fase B
        HBridgeChannel hbridge; //HBridge Channel used for motor 
        Motor_PID pid;    //Speed Controller 
        bool controller_enable = true; //Controller Enable

        /**
         @brief Class Constructor 
        */
        Motor(Motor_Pins PINS);

        /**
         @brief Start the motor 
         @param ISRA Interrupt function of encoder Fase A 
         @param ISRB Interrupt function of encoder Fase B 
        */
        void begin(void ISRA(), void ISRB());

        /**
         @brief Get motor's speed
         @return Motor's speed, in RPM
        */
        double get_speed();

        /**
         @brief Update the encoders and controller
        */
        void update();

        /**
         @brief Set the speed setpoint 
         @param SETPOINT Speed setpoint, in RPM
        */
        void set_speed(double SETPOINT);

        /**
         @brief Enable or not the PID controller
         @param ENABLE True to enable, false to disable
        */
        void enable_control(bool ENABLE);
};

#endif