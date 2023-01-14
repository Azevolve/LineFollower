#ifndef DASHBOARD
#define DASHBOARD

#include "Arduino.h"
#include "Protocol_LabVIEW_Arduino.h"

/**
 Defines a struct data to send data to LabVIEW application. The user must use 
 this struct to manipule the data.
*/
struct esp_status_data{
    double l_sp;
    double l_speed;
    double r_sp;
    double r_speed;
    double error;
    double filtered_error;
    double P;
    double I;
    double D;
    double u;
    bool sat_flag;
    uint8_t IR;

    double A_;
    double B_;
    double C_;
    double D_;   
};

/*
 Defines a struct data to convert esp_status_data to RAW_DATA 
*/
struct lbv_status_data {
    lv_int16_t l_sp;
    lv_int16_t l_speed;
    lv_int16_t r_sp;
    lv_int16_t r_speed;
    lv_int16_t error;
    lv_int16_t filtered_error;
    lv_int16_t P;
    lv_int16_t I;
    lv_int16_t D;
    lv_int16_t u;
    uint8_t sat_flag;
    uint8_t IR;

    lv_int16_t A_;
    lv_int16_t B_;
    lv_int16_t C_;
    lv_int16_t D_;

    /**
     @brief Convert the data, you can use multipliers to ajust the range values 
    */
    lbv_status_data(esp_status_data DATA); 
};


/**
    Defines a struct data to convert RAW_DATA to esp_ctrl_data 
*/
struct lbv_ctrl_data {
    uint8_t manual_control;
    uint8_t turn_on;
    lv_int16_t l_sp;
    lv_int16_t r_sp;

    lv_uint16_t tau_error;
    lv_uint16_t kp;
    lv_uint16_t ki;
    lv_uint16_t kd;
    lv_uint16_t tau_d;
    lv_uint16_t speed_ref;

    lv_int16_t A_;
    lv_int16_t B_;
    lv_int16_t C_;
    lv_int16_t D_;
    lv_int16_t E_;
    lv_int16_t F_;
};

/**
 Defines a struct data to receive data by LabVIEW application. The user must use 
 this struct to manipule the data.
*/
struct esp_ctrl_data {
    bool manual_control;
    bool turn_on;
    double l_sp;
    double r_sp;
    double tau_error;
    double kp;
    double ki;
    double kd;
    double tau_d;
    double speed_ref;

    double A_;
    double B_;
    double C_;
    double D_;
    double E_;
    double F_;

    /**
     @brief Convert the data, you can use multipliers to ajust the range values 
    */
    esp_ctrl_data(lbv_ctrl_data DATA);
};

class Dashboard{
    public:
        esp_ctrl_data get(RAW_DATA DATA); //convert RAW_DATA to esp_ctrl_data
        RAW_DATA set(esp_status_data DATA); //convert esp_status_data to RAW_DATA
};

#endif