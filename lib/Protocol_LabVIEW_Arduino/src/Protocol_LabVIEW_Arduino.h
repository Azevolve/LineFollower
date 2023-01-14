/*
Programador: Gabriel Barbosa
Email: gabriel.barbosa@polo.ufsc.br
Data: 25/05/2022

Objetivo: Implementar uma classe que permita a tranferência de dados de uma microcontrolador com uma determinada VI no software LabVIEW.
Add1: Adição de estruturas de dados para facilitar a troca de informações entre o arduino e o LabVIEW.

*/

#ifndef Protocol_LabVIEW_Arduino
#define Protocol_LabVIEW_Arduino

#include <Arduino.h>
#include <CRC16_lib.h>
#include <Queues.h>

class LABVIEW {
  private:
    QUEUE queue; //Fila usada como buffer pra analisar os bytes da Serial
    byte new_packet = false; //Variável que verifica se tem pacotes novos
    DATA data; //Variável que armazena o último DADO lido
    CRC crc; //Objeto usado para utilizar as funções de CRC

    /**
     @brief Lê a serial e adiciona na queue
    */
    void add_to_buffer(); 
    /**
     @brief Procura o byte de sincronia de ínicio
     @return O index do byte de sincronia na fila ou -1 se o byte de sincronia n estiver contido
    */
    byte find_sync_begin();
    /**
     @brief Verifica se os próximos elementos da fila atendem os requisitos do protocolo, contendo o byte de sincronia de início e de fim, o CRC e o pacote de dados
    */
    void check_new_packet();
    
  public:
    /**
     @param BUFFER Ponteiro para um array que será usado como buffer
     @param SIZE Tamanho, em bytes, do BUFFER
    */
    LABVIEW(byte BUFFER[], uint16_t SIZE);
    /**
     @brief Verifica se tem alguma informação nova pra ser processada se nenhuma outra informação estiver esperando ser processada
     @return TRUE se tem alguma informação a ser processada
     @return FALSE se n tem nenhuma informação a ser processada
    */
    byte new_infos();
    /**
     @brief Retorna o último pacote de dados lido e confiável da Serial
    */
    RAW_DATA get_data();
    /**
     @brief Envia um determinado pacote de dados para o LabView.
    */
    void send_data(RAW_DATA &INFO);

    /**
     @brief Inicializa a comunicação utilizando a infraestrutura da Serial do Arduino/Esp
     @param BAUD_RATE Taxa de envio de bits da Serial, normalmente setado para 115200
    */
    void begin();
};

/*
DataConversions to Labview.
LabVIEW Data structure: [Most Significant Byte -> least Significant Byte]
Arduino Data structure: [Least Significant Byte -> Most Significant Byte]

This library converts the data between them, it is optimized to be used with memory block copy 
*/

//LabVIEW unsigned 16-bit integer
struct lv_uint16_t {
    uint16_t data; //Stores the data in 2bytes
    lv_uint16_t(uint16_t value = 0); //Constructor which converts an uint16_t data to lv_uint16_t data
    operator uint16_t(); //Allows to convert lv_uint16_t data to uint16_t
};

//LabVIEW signed 16-bit integer
struct lv_int16_t{
    int16_t data = 0; //Stores the data in 2bytes
    lv_int16_t(int16_t value = 0); //Constructor which converts an int16_t data to lv_int16_t data
    operator int16_t(); //Allows to convert lv_int16_t data to int16_t
};

//LabVIEW signed 32-bit integer
struct lv_int32_t{
    int32_t data =0; //Stores the data in 4bytes
    lv_int32_t(int32_t value = 0); //Constructor which converts an int32_t data to lv_int32_t data
    operator int32_t(); //Allows to convert lv_int32_t data to int32_t
};

//LabVIEW 32-bit float
struct lv_float{
    float data = 0; //Stores the data in 4bytes
    lv_float(float value = 0); //Constructor which converts an float data to lv_float data
    operator float(); //Allows to convert lv_float data to float
};

//LabVIEW 64-bit float
struct lv_double{
    double data = 0; //Stores the data in 8bytes
    lv_double(double value = 0); //Constructor which converts an double data to lv_double data
    operator double(); //Allows to convert lv_double data to double
};

#endif