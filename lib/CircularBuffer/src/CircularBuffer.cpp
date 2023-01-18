#include "CircularBuffer.h"
#include <Arduino.h>

CircularBuffer::CircularBuffer(uint16_t BUFFER[], uint16_t SIZE): 
    buffer(BUFFER), size(SIZE){}

uint16_t CircularBuffer::add(uint16_t VALUE){
    index = (index+1)%size;
    buffer[index] = VALUE;
    return index;
}

uint16_t CircularBuffer::read(uint16_t INDEX){
    uint16_t relative_index = (index+INDEX)%size;
    return buffer[relative_index];
}


