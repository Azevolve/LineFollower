#ifndef ESPNOWCOMMUNICATION_LIB
#define ESPNOWCOMMUNICATION_LIB

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

/**
 @brief Data sent by ESP-Controller
*/
struct ctrl_t {
    int16_t l_sp; //Left Motor Speed Setpoint multiply by 10
    int16_t r_sp; //Right Motor Speed Setpoint multiply by 10
};

/**
 @brief Data sent by ESP-Module
*/
struct modu_t{
    int16_t l_speed; //Left Motor Speed Measurement multiply by 10
    int16_t r_speed; //Right Motor Speed Measuremente multiply by 10 
    uint8_t ir; //InfraRed Sensors Bit Mask
};

class EspNow {
    private:
        const uint8_t *ctrl_macaddress; //ESP-Controller's MAC Address
        const uint8_t *modu_macaddress; //ESP-Module's MAC Address
        bool IsItCtrl = false; // Is this ESP an ESP-Controller? 
        esp_now_peer_info peerInfo; //Connect Infos

        double l_speed; //Left Motor Speed Measurement
        double l_sp;    //Left Motor Speed Setpoint
        double r_speed; //Right Motor Speed Measurement
        double r_sp; //Right Motor Speed Setpoint
        uint8_t ir; //InfraRed Sensors Bit Mask

        int fail_packets_counter = 0; 
        int64_t pasttime = esp_timer_get_time(); //Used to restart fail_packets_counter

        /**
         @brief It makes VALUE to be between -3000 and 3000
        */
        double saturation(double VALUE);   

        /**
         @brief Get data from a ctrl_t packet, store values in properties
         @param DATA Original packet
        */
        void receive(ctrl_t &DATA);

        /**
         @brief Get data from a modu_t packet, store values in properties
         @param DATA Original packet
        */
        void receive(modu_t &DATA);

        /**
         @brief Pass the values from properties to DATA
         @param DATA Buffer which the values will be overwritten
        */
        void send(ctrl_t &DATA);

        /**
         @brief Pass the values from properties to DATA
         @param DATA Buffer which the values will be overwritten
        */
        void send(modu_t &DATA);

    public:
        /**
         @brief Constructor 
         @param ISITCTRL Is this ESP an ESP-CONTROLLER, if yes, pass True
         @param CTRL_ADDRESS ESP-CONTROLLER MAC Address
         @param MODU_ADDRESS ESP_MODULE MAC Address
        */
        EspNow (bool ISITCTRL, const uint8_t CTRL_ADDRESS[], const uint8_t MODU_ADDRESS[]);

        /**
         @brief Set ESP-Controller Data
         @param L_SP  Left Motor Speed Setpoint
         @param R_SP  Right Motor Speed Setpoint 
        */
        void set(double L_SP, double R_SP);

        /**
         @brief Set ESP-Module Data
         @param L_SPEED Left Motor Speed Measurement
         @param R_SPEED Right Motor Speed Measurement
         @param IR InfraRed Bit Mask
        */
        void set(double L_SPEED, double R_SPEED, uint8_t IR); 

        /**
         @brief Get Left Motor Speed Measurement
        */
        double get_l_speed();

        /**
         @brief Get Left Motor Speed Setpoint
        */
        double get_l_sp();

        /**
         @brief Get Right Motor Speed Measurement
        */
        double get_r_speed();

        /**
         @brief Get Right Motor Speed Setpoint
        */
       double get_r_sp();

        /**
         @brief Get InfraRed Bit Mask
        */
       uint8_t get_ir();

       /**
        @brief Send a packet to other ESP, data is gotten from class properties
        @return Esp Error Message, If it was ok, return ESP_OK.
       */
       esp_err_t send();

       /**
        @brief Process Data Received, store in class properties 
        @param DATA Variable given by receive callback function
        @param len DATA's length, in bytes
       */
       void receive(const uint8_t *DATA, int len);

       /**
        @brief Initialize the communication
       */
       void begin();

       /**
        @brief Set the sent callback-function 
        @param OnDataSent  Function declared in main software 
       */
       void set_cb_sent(void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status));

       /**
        @brief Set the receive callback-function
        @param OnDataRecv  Function declared in main software
       */
       void set_cb_receive(void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len));
    
       /**
        @brief Function which it must called inside OnDataSent (param of set_cb_sent())
        @param status Esp Now Send Status, used to increment or not fail packets counter
       */
       void cb_sent(esp_now_send_status_t status);
};  



#endif