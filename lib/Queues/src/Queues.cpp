#include <Queues.h>
#include <Arduino.h>

QUEUE::QUEUE(byte BUFFER[], uint16_t SIZE): bytes(BUFFER), size_buffer(SIZE){}  
uint16_t QUEUE::available_to_write(){
  return size_buffer - elements;
}
uint16_t QUEUE::available_to_read(){
  return elements;
}
byte QUEUE::pop(){
  int16_t index = index_to_add - elements;
  if (index < 0) index = size_buffer + index;
  elements--;
  return bytes[index];
}
void QUEUE::add(byte VALUE){
  bytes[index_to_add] = VALUE;
  index_to_add = (index_to_add + 1)%size_buffer;
  elements++;  
}
int16_t QUEUE::search(byte VALUE){
  int16_t first = index_to_add - elements;
  if (first < 0) first = size_buffer + first;
  for (int16_t i = 0; i < elements; i++){
    if (bytes[(i + first)%size_buffer] == VALUE) return i; 
  }
  return -1;
}
void QUEUE::pop_all(){
  elements = 0;
  index_to_add = 0;
}
byte QUEUE::read(uint16_t INDEX){
  int16_t first = index_to_add - elements;
  if (first < 0) first = size_buffer + first;
  return bytes[(INDEX + first)%size_buffer]; 
}
void QUEUE::print(){
  Serial.println();
  for (uint16_t i = 0; i < elements; i++){
    byte value = read(i);
    if (value < 16) Serial.print("0");
    Serial.print(value, HEX);
    Serial.print(" ");
  }
  Serial.println();
}


