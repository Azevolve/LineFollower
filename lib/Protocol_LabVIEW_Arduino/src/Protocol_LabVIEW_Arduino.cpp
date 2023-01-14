#include "Protocol_LabVIEW_Arduino.h"

LABVIEW::LABVIEW(byte BUFFER[], uint16_t SIZE): queue(BUFFER, SIZE){}
void LABVIEW::add_to_buffer(){
  uint16_t to_read = Serial.available();
  if (to_read == 0) return;
  uint16_t to_write = queue.available_to_write();
  uint16_t to_process = 0;
  if (to_read < to_write) to_process = to_read;
    else to_process = to_write;
  byte values [to_process];
  //Serial.println();
  Serial.readBytes(values, to_process);
  for (uint16_t i = 0; i < to_process; i++){
    queue.add(values[i]);
  }
  //Serial.print("Add Buffer");
  //queue.print();
  //Serial.println();    
}
byte LABVIEW::find_sync_begin(){
  int32_t index = queue.search(SYNC_BEGIN);
  //Serial.printf("\n SYNC_BEGIN %d", index);
  if (index < 0) {
    queue.pop_all();
    //Serial.print("Não achou o SYNC_BEGIN");
    //queue.print();
    //Serial.println();
    return false;
  } 
  for (int32_t i = 0; i < index; i++){
    queue.pop();
  }
  //Serial.print("Achou o SYNC_BEGIN");
  //queue.print();
  //Serial.println();
  return true;    
}
void LABVIEW::check_new_packet(){
  if (find_sync_begin() && (queue.available_to_read() >= sizeof(PACKET))){
    if (queue.read(sizeof(PACKET) - 1) == SYNC_END){
      queue.pop();
      for (uint8_t i = 0; i < sizeof(DATA); i++){
        data.infos[i] = queue.pop();
      }
      queue.pop();
      new_packet = crc.check_crc(data);
    } else{
      queue.pop();
    }
    //Serial.print("Process");
    //queue.print();
    //Serial.println();
  } else {
    //Serial.print("Not Process");
    //queue.print();
    //Serial.println();
  }    
}
byte LABVIEW::new_infos() {
  while((!new_packet) && ((Serial.available() > 0) || (queue.available_to_read() > sizeof(PACKET)))){
    add_to_buffer();
    check_new_packet();
  }
  return new_packet;
}
RAW_DATA LABVIEW::get_data() {
    new_packet = false;
    RAW_DATA infos;
    infos.from_DATA(data);
    return infos;
}
void LABVIEW::send_data(RAW_DATA &INFO){
    DATA data = crc.add_crc(INFO);
    PACKET packet;
    for (uint8_t i = 0; i < sizeof(data); i++){
      packet.infos[i] = data.infos[i];
    }
    Serial.write((byte*)&packet, sizeof(PACKET));
}
void LABVIEW::begin(){
  Serial.begin(115200);
}

void invert(void *In, void *Out, int len){
  uint8_t buf1[len]; //Declares a buffer with the same size of In data 
  memcpy(&buf1, In, len); //Copy memory block of In to buffer pointer address
  uint8_t buf2[len]; //Declares other buffer with the same size of In data too
  for(int i =0; i < len; i++) buf2[i] = buf1[len-1-i]; //Invert que bytes sequence
  memcpy(Out, &buf2, len); //Copy memory block of buf2 to Out pointer address
}

lv_uint16_t::lv_uint16_t(uint16_t value){
  invert(&value, &data, 2);
}
lv_uint16_t::operator uint16_t(){
  uint16_t value;
  invert(&data, &value, 2);
  return value;
}

lv_int16_t::lv_int16_t(int16_t value){
  invert(&value, &data, 2);
}
lv_int16_t::operator int16_t(){
  int16_t value;
  invert(&data, &value, 2);
  return value;
}

lv_int32_t::lv_int32_t(int32_t value){
  invert(&value, &data, 4);
}
lv_int32_t::operator int32_t(){
  int32_t value;
  invert(&data, &value, 4);
  return value;
}

lv_float::lv_float(float value){
  invert(&value, &data, 4);
}
lv_float::operator float(){
  float value;
  invert(&data, &value, 4);
  return value;
}

lv_double::lv_double(double value){
  invert(&value, &data, 8);
}
lv_double::operator double(){
  double value;
  invert(&data, &value, 8);
  return value;
}

