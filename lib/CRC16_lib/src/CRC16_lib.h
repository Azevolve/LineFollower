/*
Programador: Gabriel Barbosa
Email: gabriel.barbosa@polo.ufsc.br
Data: 25/05/2022

Objetivo: Implementar uma classe que permita a manipulação de bytes de informação, adicionando 16 bits verificadores usando a técnica de CRC.

*/

#ifndef CRC16_lib
#define CRC16_lib

#include <Arduino.h>

const uint8_t SIZE_DATA = 32; //Tamanho do pacote de dados, em bytes, dados úteis + CRC (2 bytes) = SIZE_DATA
const byte SYNC_BEGIN = 0b10010010; //Byte de sincronia de inicio do envio/leitura da Serial
const byte SYNC_END = 0b01101101; //Byte de sincronia de fim do envio/leitura da Serial
const uint32_t POLY = 0xac9a; //Polinômio usado para o cálculo do CRC, com a notação de +1 implicita;

/**
 @brief Estrutura que armazena o vetor de dados do pacote a ser enviado/processado 
*/
struct DATA{
    byte infos[SIZE_DATA];
};

/**
 @brief Classe que armaneza os dados úteis a serem enviados/processados
*/
class RAW_DATA{
  public:
    /**
     @brief Informações armazenadas do objeto
    */ 
    byte infos[SIZE_DATA - 2];
    /**
     @brief Converte RAW_DATA em DATA com os bytes de CRC zerados
    */
    DATA to_DATA();

    /**
     @brief Converte DATA para RAW_DATA, apagando os bits de CRC
    */
    void from_DATA(DATA &INFO);
};
/**
 @brief Dados que serão enviados/lidos de fato, com bytes de sincronia e as informações (RAW_DATA+CRC)
*/
struct PACKET {
    byte sync_begin = SYNC_BEGIN;
    byte infos[SIZE_DATA];
    byte sync_end = SYNC_END;
};
/**
 @brief Estrutura usada para manipular os coeficientes durante o calculo do CRC. MSB = MSB do coeffs[0] e LSB = MSB do coeffs[2] (17bits)
*/
struct POLYNOMIAL {
    byte coeffs[3];
};

/**
 @brief Classe que implementa a comunicação com o LabView usando de interface a comunicação serial implementada. 
*/
class CRC { 
  private:
    /**
     @brief Polinômio usado para o calculo do CRC explicitando a notação +1
    */
    POLYNOMIAL poly = {byte(POLY/256), byte(POLY), 0b10000000};

    /**
     @brief Retorna o bit em uma certa posição. Posição 0 = MSB de MESSAGE.infos[0]. Posição 31 = LSB de MESSAGE.infos[3]
    */
    byte get_bit(DATA &MESSAGE, uint16_t POSITION);
    /**
     @brief Retorna o bit em uma certa posição. Posição 0 = MSB de POLY.coeffs[0]. Posição 16 = MSB de POLY.coeffs[2]
    */
    byte get_bit(POLYNOMIAL &POLY, uint16_t POSITION);
    /**
     @brief Define o bit em uma certa posição. Posição 0 = MSB de POLY.coeffs[0]. Posição 16 = MSB de POLY.coeffs[2]
    */
    void set_bit(POLYNOMIAL &POLY, uint16_t POSITION, byte STATE);
    /**
     @brief Define o bit em uma certa posição. Posição 0 = MSB de MESSAGE.infos[0]. Posição 31 = LSB de MESSAGE.infos[3]
    */
    void set_bit(DATA &MESSAGE, uint16_t POSITION, byte STATE);
    /**
     @brief Desloca os bits de POLY para a esquerda, o estado do bit na posição 16 não é definida. 
    */ 
    void shift_left_polynomial(POLYNOMIAL &POLY);
    /**
     @brief Faz o calculo XOR bit a bit entre dois polinômios se o MSB de INITIAL_POLY for 1 
    */
    POLYNOMIAL calculate_XOR(POLYNOMIAL INITIAL_POLY, POLYNOMIAL DIVISOR);
    /**
     @brief Calcula o CRC e adiciona os 16 do CRC no final de DATA_ 
    */
    DATA crc(DATA DATA_);

  public:
    /**
     @brief Printa os bits do polinômio usado no cálculo do CRC
    */
    void print_polynomial();
    /**
     @brief Printa os bits de MESSAGE
    */
    void print_data (DATA MESSAGE);
    /**
     @brief Calcula um novo pacote de dados contendo DATA_ e o respectivo CRC
     @return DATA_ + CRC
    */ 
    DATA add_crc(RAW_DATA &DATA_);
    /**
     @brief Verifica se DATA_ está com o seu respectivo CRC
     @return TRUE se o CRC estiver correto
    */
    byte check_crc(DATA &DATA_);
};

#endif

