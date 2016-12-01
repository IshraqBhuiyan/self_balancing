
#ifndef ROBOT
#define ROBOT

#include <stdint.h>
#include <Servo.h>

static uint8_t i2cbuffer[8];
static uint32_t kalmanTimer, imuTimer, encoderTimer, PIDTimer, testTimer, nTimer, n3Timer;
static float accAngle, pitch, gyroAngle;

#define rightEncoder1 2
#define rightEncoder2 5
#define leftEncoder1 3
#define leftEncoder2 6

static volatile int32_t leftCounter = 0;
static volatile int32_t rightCounter = 0;

static int32_t lastWheelPosition;
static int32_t wheelVelocity;
static int32_t targetPosition;
static int32_t lastLeftPosition;
static int32_t lastRightPosition;
static int32_t leftVelocity;
static int32_t rightVelocity;

static bool steerStop = true;

#define leftMotorPin 9
#define rightMotorPin 10

#define min_pulsewidth 1100
#define max_pulsewidth 1900

typedef struct {
  float P, I, D; // PID variables
  float targetAngle; // Resting angle of the robot
  uint8_t backToSpot; // Set whenever the robot should stay in the same spot
  uint8_t controlAngleLimit; // Set the maximum tilting angle of the robot
  uint8_t turningLimit; // Set the maximum turning value
  //float Qangle, Qbias, Rmeasure; // Kalman filter values
  float accYzero, accZzero; // Accelerometer zero values
  float leftMotorScaler, rightMotorScaler;
  //bool bindSpektrum;
} cfg_t;

extern cfg_t cfg;

static float integratedError;
static float i2Term; //placeholder
static float lastError;
static float lastRestAngle;
#endif
