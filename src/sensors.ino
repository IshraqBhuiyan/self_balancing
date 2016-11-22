
void MPU_setup(){
  Wire.begin();
  Wire.setClock(400000UL);
  while(i2cWrite(0x6B, 0x80, true));
  do{
    while(i2cRead(0x6B, i2cbuffer, 1));
  } while(i2cbuffer[0] & 0x80);
  delay(5);
  while(i2cWrite(0x6B, 0x09, true));
  i2cbuffer[0]=1;
  i2cbuffer[1]=0x03;
  i2cbuffer[2]=0x00;
  i2cbuffer[3]=0x00;
  while(i2cWrite(0x19,i2cbuffer, 4, true));
  delay(100);
  while(i2cRead(0x3D, i2cbuffer, 8));
  int16_t accY = ((i2cbuffer[0]<<8) | i2cbuffer[1]);
  int16_t accZ = ((i2cbuffer[2]<<8) | i2cbuffer[3]);
  int16_t gyroX = ((i2cbuffer[6]<<8) | i2cbuffer[7]);
  accAngle = (atan2((float)accY-0,(float)accZ-0)+PI)*RAD_TO_DEG;
  kalman.setAngle(accAngle);
  pitch = accAngle;
  gyroAngle = accAngle;

  while(calibrateGyro());

  kalmanTimer = micros();
  imuTimer = millis();
  encoderTimer = imuTimer;
  PIDTimer = imuTimer;
}

void MPU_update(){
  while(i2cRead(0x3D, i2cbuffer, 8));
  int16_t accY = ((i2cbuffer[0]<<8) | i2cbuffer[1]);
  int16_t accZ = ((i2cbuffer[2]<<8) | i2cbuffer[3]);
  int16_t gyroX = ((i2cbuffer[6]<<8) | i2cbuffer[7]);
  Serial.println("accY = " + (String)accY);
  Serial.println("accZ = " + (String)accZ);
  Serial.println("GyroX = " + (String)gyroX);
  accAngle = (atan2((float)accY-0,(float)accZ-0)+PI)*RAD_TO_DEG;
  Serial.println("accAngle = " + (String)accAngle);

  uint32_t timer = micros();

  if ((accAngle < 90 && pitch > 270) || (accAngle > 270 && pitch < 90)) {
    kalman.setAngle(accAngle);
    pitch = accAngle;
    gyroAngle = accAngle;
  } else {
    float gyroRate = ((float)gyroX - 0) / 131.0f; // Convert to deg/s
    float dt = (float)(timer - kalmanTimer) / 1000000.0f;
    gyroAngle += gyroRate * dt; // Gyro angle is only used for debugging
    if (gyroAngle < 0 || gyroAngle > 360)
      gyroAngle = pitch; // Reset the gyro angle when it has drifted too much
    pitch = kalman.getAngle(accAngle, gyroRate, dt); // Calculate the angle using a Kalman filter
  }
  kalmanTimer = timer;
}

void setup_encoder(){
  pinMode(leftEncoder1, INPUT);
  pinMode(leftEncoder2, INPUT);
  pinMode(rightEncoder1, INPUT);
  pinMode(rightEncoder2, INPUT);
  digitalWrite(leftEncoder1, HIGH);
  digitalWrite(leftEncoder2, HIGH);
  digitalWrite(rightEncoder1, HIGH);
  digitalWrite(rightEncoder2, HIGH);
  attachInterrupt(digitalPinToInterrupt(leftEncoder1), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoder1), rightEncoder, CHANGE);
}
void updateEncoder(){
  uint32_t timer = millis();
  if(timer - encoderTimer >= 100){
    encoderTimer = timer;
    int32_t wheelPosition = getWheelsPosition();
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;
  }
}
void leftEncoder(){
  if (digitalRead(leftEncoder1) == HIGH) {
    if (digitalRead(leftEncoder2) == LOW) {
      leftCounter++;
    } else {
      leftCounter--;
    }
  } else {
    if (digitalRead(leftEncoder2) == LOW) {
      leftCounter--;
    } else {
      leftCounter++;
    }
  }
}

void rightEncoder(){
  if (digitalRead(leftEncoder1) == HIGH) {
    if (digitalRead(leftEncoder2) == LOW) {
      rightCounter++;
    } else {
      rightCounter--;
    }
  } else {
    if (digitalRead(leftEncoder2) == LOW) {
      rightCounter--;
    } else {
      rightCounter++;
    }
  }
}

int32_t getWheelsPosition(){
  return leftCounter + rightCounter;
}
