
#ifndef CIRCULARBUFFER
#define CIRCULARBUFFER

#include <Arduino.h>

class CircularBuffer{
  private:
    uint16_t *buffer; //Pointer to buffer array
    uint16_t index = 0; //Index of the last data  
    uint16_t size; //Size of buffer
  public:

    /**
     @brief Class Constructor 
     @param BUFFER Buffer's pointer 
     @param SIZE Buffer's size
    */
    CircularBuffer(uint16_t BUFFER[], uint16_t SIZE);

    /**
     @brief Add a value replacing the odder data
     @param VALUE Value what will be added 
     @return the index of the value in the buffer
    */
    uint16_t add(uint16_t VALUE);

    /**
     @brief Return the value in POS. the first POS (POS = 0) is the newest value.
     @param POS Relative of the insertion order
     @return The value in POS 
    */
    uint16_t read(uint16_t POS);
};

#endif