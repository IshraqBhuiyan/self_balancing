static const uint8_t IMU_ADDR = 0x68;
static const uint16_t timeout = 100;

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop){
  return i2cWrite(registerAddress, &data, 1, sendStop);
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop){
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop);
  if(rcode){
    Serial.print("I2C Write Failed: ");
    Serial.println(rcode);
  }
  return rcode;
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes){
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false);
  if(rcode){
    Serial.print("I2C Read Failed: ");
    Serial.println(rcode);
    return rcode;
  }
  Wire.requestFrom(IMU_ADDR, nbytes, (uint8_t)true);
  for(uint8_t i = 0; i< nbytes; i++){
    if(Wire.available()){
      data[i]=Wire.read();
    }else{
      timeOutTimer = micros();
      while(((micros()-timeOutTimer)<timeout) && !Wire.available());
      if(Wire.available()){
        data[i]=Wire.read();
      }else{
        Serial.println("I2C Read timeout");
        return 5;
      }
    }
  }
  return 0;
}
