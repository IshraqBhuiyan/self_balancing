bool calibrateGyro() {
  int16_t gyroXbuffer[25];
  for (uint8_t i = 0; i < 25; i++) {
    while (i2cRead(0x43, i2cbuffer, 2));
    gyroXbuffer[i] = ((i2cbuffer[0] << 8) | i2cbuffer[1]);
    delay(10);
  }
  if (!checkMinMax(gyroXbuffer, 25, 2000)) {
    Serial.println(F("Gyro calibration error"));
    //buzzer::Set();
    return 1;
  }
  //for (uint8_t i = 0; i < 25; i++)
  //  gyroXzero += gyroXbuffer[i];
  //0 /= 25.0f;
  return 0;
}

bool checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference) { // Used to check that the robot is laying still while calibrating
  int16_t min = array[0], max = array[0];
  for (uint8_t i = 1; i < length; i++) {
    if (array[i] < min)
      min = array[i];
    else if (array[i] > max)
      max = array[i];
  }
  return max - min < maxDifference;
}
