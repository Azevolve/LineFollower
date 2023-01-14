/*
Programador: Gabriel Barbosa
Email: gabriel.barbosa@polo.ufsc.br
Data: 26/05/2022

Objetivo: Implementar uma classe que gerencia um buffer de bytes como uma queue com inserção e remoção em O(1)  

*/

#ifndef Queues_lib
#define Queues_lib

#include <Arduino.h>

class QUEUE{
  private:
    byte *bytes; //Ponteiro do array passado como buffer
    const uint16_t size_buffer; //Tamanho, em bytes, do buffer
    uint16_t elements = 0; //Elementos sendo armazenados no buffer
    uint16_t index_to_add = 0; //Indice que o próximo elemento deve ser inserido deve ser inserido

  public:
    /**
     @brief Instancia um objeto
     @param BUFFER ponteiro de um array que será usado como buffer
     @param SIZE tamanho, em bytes, do array
    */
    QUEUE(byte BUFFER[], uint16_t SIZE);   
    /**
     @brief Retorna o número de elementos que ainda podem ser inseridos na queue
    */ 
    uint16_t available_to_write();

    /**
     @brief Retorna o número de elementos que podem ser lidos
    */
    uint16_t available_to_read();
    /**
     @brief Remove um elemento da queue
     @return Elemento removido
    */
    byte pop();
    /**
     @brief Adiciona um elemento na queue
     @param VALUE Elemento a ser adicionado
    */
    void add(byte VALUE);
    /**
     @brief Procura a posição do primeiro elemento igual a VALUE
     @param VALUE Byte a ser encotrado
     @return Posição do elemento igual a VALUE na queue. Se não existir um elemento igual, retorna -1
    */
    int16_t search(byte VALUE);

    /**
     @brief Deleta todos os elementos da queue
    */
    void pop_all();
    /**
     @brief Lê um elemento em uma determinada posição
     @param INDEX Posição da queue que se desaja ler
    */
    byte read(uint16_t INDEX);

    /**
     @brief Printa a queue na serial
    */
    void print();
};

#endif