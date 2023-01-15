#include "Arduino.h"
//Pinout ESP32 WROOM with 30 pins
#define  VP 36 //GPI36  Input Only ADC1 RTC-PIN No pullup and pulldown resistors, capacitor between this pin and GPIO37
#define  VN 39 //GPI39  Input Only ADC1 RTC-PIN No pullup and pulldown resistors, capacitor between this pin and GPIO38
#define D34 34 //GPI34  Input Only ADC1 RTC-PIN No pullup and pulldown resistors
#define D35 35 //GPI35  Input Only ADC1 RTC-PIN No pullup and pulldown resistors
#define D32 32 //GPIO32 ADC1 RTC-PIN TOUCH-PIN 
#define D33 33 //GPIO33 ADC1 RTC-PIN TOUCH-PIN
#define D25 25 //GPIO25 ADC2 RTC-PIN DAC-PIN
#define D26 26 //GPIO26 ADC2 RTC-PIN DAC-PIN 
#define D27 27 //GPIO27 ADC2 RTC-PIN TOUCH-PIN
#define D14 14 //GPIO14 PWM signal at boot ADC2 RTC-PIN TOUCH-PIN HSPI-CLK-PIN
#define D12 12 //GPIO12 Boot fail if pulled high, strapping pin ADC2 RTC-PIN TOUCH-PIN HSPI-MISO-PIN
#define D13 13 //GPIO13 ADC2 RTC-PIN TOUCH-PIN HSPI-MOSI-PIN
#define D15 15 //GPIO15 PWM signal at boot, strapping pin ADC2 RTC-PIN TOUCH-PIN HSPI-CS0-PIN
#define D2   2 //GPIO2  Must be left floating or LOW to enter flashing mode ADC2 RTC-PIN TOUCH-PIN ON-BOARD-LED
#define D4   4 //GPIO4  ADC2 RTC-PIn TOUCH-PIN 
#define RX2 16 //GPIO16 UART Serial RX2 
#define TX2 17 //GPIO17 UART Serial TX2
#define D5   5 //GPIO5  PWM signal at boot, strapping pin VSPI-CS0-PIN
#define D18 18 //GPIO18 VSPI-CLK-PIN
#define D19 19 //GPIO19 VSPI-MISO-PIN
#define D21 21 //GPIO21 I2C-SDA
#define RX0  3 //GPIO3  HIGH at boot UART Serial RX0
#define TX0  1 //GPIO1  Debug output at boot UART Serial TX0
#define D22 22 //GPIO22 I2C-SCL 
#define D23 23 //GPIO23 VSPI-MOSI-PIN 

struct Motor_Pins{
    int PWM_PIN;
    int PWM_CHANNEL;
    int CTROl_1;
    int CTROL_2;
    int ENC_A;
    int ENC_B;
};

struct IR_Pins{
    int pins[8];
};

//Line Sensor Pins
const IR_Pins IR_pins = {VP, VN, D34, D35, D32, D33, D25, D22};

//Left Motor Pins and Channel PWM
const Motor_Pins LeftMotorPins  = {D21, 0, D2, D13, TX2, RX2};

//Right Motor Pins and Channel PWM
const Motor_Pins RightMotorPins = {D19, 1, D26, D27, D15, D14};

//Modu ESP Mac Address
const uint8_t modu_mac[] = {0xC4, 0x4F, 0x33, 0x65, 0xDE, 0x3D}; 

//Ctrl ESP Mac Address
const uint8_t ctrl_mac[] = {0x3C, 0x61, 0x05, 0x0D, 0x46, 0x80}; 