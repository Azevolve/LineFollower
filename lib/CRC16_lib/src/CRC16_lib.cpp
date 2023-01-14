#include "CRC16_lib.h"

DATA RAW_DATA::to_DATA(){
  DATA data;
  for (uint16_t i = 0; i < sizeof(RAW_DATA); i++){
    data.infos[i] = infos[i];
  }
  data.infos[sizeof(DATA) - 2] = 0;
  data.infos[sizeof(DATA) - 1] = 0;
  return data;
}

void RAW_DATA::from_DATA(DATA &INFO){
  for(uint16_t i = 0; i < sizeof(RAW_DATA); i++){
    infos[i] = INFO.infos[i];
  }
}

byte CRC::get_bit(DATA &MESSAGE, uint16_t POSITION){
  uint16_t which_byte = POSITION/8;
  uint16_t which_bit = POSITION%8;
  return bitRead(MESSAGE.infos[which_byte], 7-which_bit);
}
byte CRC::get_bit(POLYNOMIAL &POLY, uint16_t POSITION){
  uint16_t which_byte = POSITION/8;
  uint16_t which_bit = POSITION%8;
  return bitRead(POLY.coeffs[which_byte], 7-which_bit);
}
void CRC::set_bit(POLYNOMIAL &POLY, uint16_t POSITION, byte STATE){
  uint16_t which_byte = POSITION/8;
  uint16_t which_bit = POSITION%8;
  bitWrite(POLY.coeffs[which_byte], 7 - which_bit, STATE);
}
void CRC::set_bit(DATA &MESSAGE, uint16_t POSITION, byte STATE){
  uint16_t which_byte = POSITION/8;
  uint16_t which_bit = POSITION%8;
  bitWrite(MESSAGE.infos[which_byte], 7 - which_bit, STATE);
}
void CRC::shift_left_polynomial(POLYNOMIAL &POLY){
  POLY.coeffs[0] = POLY.coeffs[0]*2+POLY.coeffs[1]/128;
  POLY.coeffs[1] = POLY.coeffs[1]*2+POLY.coeffs[2]/128;
  POLY.coeffs[2] = 0;
}
POLYNOMIAL CRC::calculate_XOR(POLYNOMIAL INITIAL_POLY, POLYNOMIAL DIVISOR){
  POLYNOMIAL resto = INITIAL_POLY;
  if (get_bit(INITIAL_POLY, 0) == 1){
    for (uint8_t i = 0; i < 3; i++){
      resto.coeffs[i] = INITIAL_POLY.coeffs[i] ^ DIVISOR.coeffs[i];
    }
  }
  return resto;
}
DATA CRC::crc(DATA DATA_){
  POLYNOMIAL dividendo;
  for (uint8_t i = 0; i < 17; i++){
    set_bit(dividendo, i, get_bit(DATA_, i));
  }
  dividendo = calculate_XOR(dividendo, poly);
  for (uint16_t i = 17; i < 8*sizeof(DATA); i++){
    shift_left_polynomial(dividendo);
    set_bit(dividendo, 16, get_bit(DATA_, i));
    dividendo = calculate_XOR(dividendo, poly);
    // for(int i = 0; i < 3; i++){
    //   Serial.print(dividendo.coeffs[i]);
    //   Serial.print(" ");
    // }
    // Serial.println();
  }
  shift_left_polynomial(dividendo);
  DATA_.infos[sizeof(DATA)-2] = dividendo.coeffs[0];
  DATA_.infos[sizeof(DATA)-1] = dividendo.coeffs[1];
  
  // Serial.println();
  // for (int i = 0; i < sizeof(DATA); i++){
  //   Serial.print(DATA_.infos[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();

  return DATA_;
}

void CRC::print_polynomial (){
 Serial.print("\t");
 for (uint8_t i = 0; i < 17; i++){
   Serial.print(get_bit(poly, i));
 }
 Serial.println(); 
}
void CRC::print_data (DATA MESSAGE){
  Serial.print("\t");
  for (uint16_t i = 0; i < sizeof(MESSAGE)*8; i++){
   if (i == (sizeof(MESSAGE)-2)*8) Serial.print("\t");
   Serial.print(get_bit(MESSAGE, i));
  }
  Serial.println(); 
}
DATA CRC::add_crc(RAW_DATA &RAW_DATA_){
  DATA data = RAW_DATA_.to_DATA();
  return crc(data);
}
byte CRC::check_crc(DATA &DATA_){
  DATA data = crc(DATA_);
  if ((data.infos[sizeof(DATA) - 2] == 0) || (data.infos[sizeof(DATA) - 1] == 0)) return true;
  return false;
}