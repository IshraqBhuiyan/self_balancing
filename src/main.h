
#ifndef ROBOT
#define ROBOT

#include <stdint.h>
#include <Servo.h>

static uint8_t i2cbuffer[8];
static uint32_t kalmanTimer, imuTimer, encoderTimer, PIDTimer, testTimer, nTimer, n3Timer, testTimer2;
static uint32_t testTimer3, tt4, avoidTimer;
static float accAngle, pitch, gyroAngle;

#define trigPin 4
#define echoPin 3
#define trigPin2 5
#define echoPin2 2

#define rightEncoder1 3
#define rightEncoder2 6
#define leftEncoder1 2
#define leftEncoder2 5

static volatile int32_t leftCounter = 0;
static volatile int32_t rightCounter = 0;

double setpoint, input, output;

static volatile uint32_t startTimer, endTimer, distance;
static volatile uint32_t startTimer2, endTimer2, distance2;
static volatile int state, state2;

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
  float accYzero, accZzero; // Accelerometer zero values
  float leftMotorScaler, rightMotorScaler;
  uint8_t avoidDistance;
} cfg_t;

extern cfg_t cfg;

static float integratedError;
static float i2Term; //placeholder
static float lastError;
static float lastRestAngle;
#endif
