#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define YAW 0
#define PITCH 1
#define ROLL 2
#define BT_RX 10
#define BT_TX 11

SoftwareSerial bluetooth(BT_RX, BT_TX); // RX, TX
char receivedChar;

MPU6050 mpu;
Servo esc5;
Servo esc6;
Servo esc3;
Servo esc9;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// Orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // Indicates whether MPU interrupt pin has gone high

void dmpDataReady()
{
  mpuInterrupt = true;
}

float Kp = 1.0;
float Ki = 3;
float Kd = 1.0;
// متغيرات PID
float rollIntegral = 0.0;
float pitchIntegral = 0.0;
float yawIntegral = 0.0;
float rollderivative = 0.0;
float pitchDerivative = 0.0;
float yawDerivative = 0.0;
float controlOutput5 = 0.0;
float controlOutput6 = 0.0;
float controlOutput3 = 0.0;
float controlOutput9 = 0.0;
float derivativeMax = 500.0;
float integralMax = 200.0;
float previousRollError = 0.0;
float previousPitchError = 0.0;
float previousYawError = 0.0;
float setpoint = 0.0;
float currentRollAngle = 0.0;
float currentPitchAngle = 0.0;
float currentYawAngle = 0.0;
float outputMax = 1900.0;
float outputMin = 1010.0;
float speed = 1010.0;
bool stopped = false;
float contini = 0.0;

// متغيرات الوقت
unsigned long currentTime;
unsigned long previousTime;
float deltaT;

void setup()
{
  esc5.attach(5, 1000, 2000);
  esc6.attach(6, 1000, 2000);
  esc9.attach(9, 1000, 2000);
  esc3.attach(3, 1000, 2000);
  esc5.writeMicroseconds(2000);
  esc6.writeMicroseconds(2000);
  esc9.writeMicroseconds(2000);
  esc3.writeMicroseconds(2000);
  bluetooth.begin(9600);
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  Serial.begin(57600);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // Verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // Load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // MPU calibration: set YOUR offsets here.
  mpu.setXAccelOffset(-222);
  mpu.setYAccelOffset(1000);
  mpu.setZAccelOffset(614);
  mpu.setXGyroOffset(148);
  mpu.setYGyroOffset(35);
  mpu.setZGyroOffset(56);

  // Returns 0 if it worked
  if (devStatus == 0)
  {
    // Turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // Enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0 : #pin2)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // Set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // Get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {

    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  Serial.println("high");
  delay(4000);
  esc5.writeMicroseconds(1000);
  esc6.writeMicroseconds(1000);
  esc9.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  Serial.println("low");
  delay(6000);

  previousTime = millis();
}

void loop()
{
  // Check if 60 seconds have elapsed; if so, stop all motors
  if ((currentTime - contini) / 1000.0 >= 6.0)
  {
    Serial.println("bluetooth timeout - stopping motors.");
    // Stop all motors
    esc5.writeMicroseconds(1000);
    esc6.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc9.writeMicroseconds(1000);
    delay(10000); // Hold stopped for 10 seconds to prevent re-entering loop
    return;
  }

  // Serial.println(currentTime / 1000);
  if (stopped)
  {
    esc3.writeMicroseconds(1000);
    esc5.writeMicroseconds(1000);
    esc6.writeMicroseconds(1000);
    esc9.writeMicroseconds(1000);
    return;
  }

  // If programming failed, don't try to do anything
  if (!dmpReady)
  {
    return;
  }

  // Wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    // Do nothing...
  }

  // Reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // Get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // Check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // Otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // Wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize)
    {
      fifoCount = mpu.getFIFOCount();
    }

    // Read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // Track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // Convert Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Print angle values in degrees.
    // Serial.print(ypr[YAW] * (180 / M_PI));
    // Serial.print("\t");
    // Serial.print(ypr[PITCH] * (180 / M_PI));
    // Serial.print("\t");
  }

  currentRollAngle = ypr[ROLL] * (180 / M_PI);
  currentPitchAngle = ypr[PITCH] * (180 / M_PI);
  currentYawAngle = ypr[YAW] * (180 / M_PI);
  float rollError = setpoint - currentRollAngle;
  float pitchError = setpoint - currentPitchAngle;
  float yawError = setpoint - currentYawAngle;
  currentTime = millis();
  deltaT = (currentTime - previousTime) / 1000.0; // تحويل إلى ثوانٍ
  previousTime = currentTime;
  rollIntegral = rollIntegral + (rollError * deltaT);
  pitchIntegral = pitchIntegral + (pitchError * deltaT);
  yawIntegral = yawIntegral + (yawError * deltaT);
  // قيد التكامل ليكون ضمن الحدود
  if (rollIntegral > integralMax)
    rollIntegral = integralMax;
  if (rollIntegral < -integralMax)
    rollIntegral = -integralMax;

  if (pitchIntegral > integralMax)
    pitchIntegral = integralMax;
  if (pitchIntegral < -integralMax)
    pitchIntegral = -integralMax;

  if (yawIntegral > 50.0)
    yawIntegral = 50.0;
  if (yawIntegral < -50.0)
    yawIntegral = -50.0;

  pitchDerivative = (pitchError - previousPitchError) / deltaT;
  previousPitchError = pitchError;
  // قيد المشتق ليكون ضمن الحدود
  if (pitchDerivative > derivativeMax)
    pitchDerivative = derivativeMax;
  if (pitchDerivative < -derivativeMax)
    pitchDerivative = -derivativeMax;

  rollderivative = (rollError - previousRollError) / deltaT;
  previousRollError = rollError;
  // قيد المشتق ليكون ضمن الحدود
  if (rollderivative > derivativeMax)
    rollderivative = derivativeMax;
  if (rollderivative < -derivativeMax)
    rollderivative = -derivativeMax;
  yawDerivative = (yawError - previousYawError) / deltaT;
  previousYawError = yawError;
  // قيد المشتق ليكون ضمن الحدود
  if (yawDerivative > derivativeMax)
    yawDerivative = derivativeMax;
  if (yawDerivative < -derivativeMax)
    yawDerivative = -derivativeMax;

  controlOutput5 = speed + (Kp * rollError + Ki * rollIntegral + Kd * rollderivative) - (Kp * yawError + Ki * yawIntegral);
  controlOutput6 = speed - (Kp * rollError + Ki * rollIntegral + Kd * rollderivative) - (Kp * yawError + Ki * yawIntegral);
  controlOutput3 = speed - (Kp * pitchError + Ki * pitchIntegral + Kd * pitchDerivative) + (Kp * yawError + Ki * yawIntegral);
  controlOutput9 = speed + (Kp * pitchError + Ki * pitchIntegral + Kd * pitchDerivative) + (Kp * yawError + Ki * yawIntegral);

  if (controlOutput5 > outputMax)
    controlOutput5 = outputMax;
  if (controlOutput5 < outputMin)
    controlOutput5 = outputMin;
  if (controlOutput6 > outputMax)
    controlOutput6 = outputMax;
  if (controlOutput6 < outputMin)
    controlOutput6 = outputMin;
  if (controlOutput3 > outputMax)
    controlOutput3 = outputMax;
  if (controlOutput3 < outputMin)
    controlOutput3 = outputMin;
  if (controlOutput9 > outputMax)
    controlOutput9 = outputMax;
  if (controlOutput9 < outputMin)
    controlOutput9 = outputMin;

  esc5.writeMicroseconds(controlOutput5);
  esc6.writeMicroseconds(controlOutput6);
  esc3.writeMicroseconds(controlOutput3);
  esc9.writeMicroseconds(controlOutput9);

  if (bluetooth.available())
  {
    int incoming = bluetooth.read(); // read as int to detect -1 if no data
    if (incoming != -1)
    {
      char c = (char)incoming;
      // ignore common line endings
      if (incoming != '\r' && incoming != '\n')
      {
        receivedChar = c; // use for control below
      }
    }
  }

  if (receivedChar == 'N')
  {
    Serial.println("N");
    speed = speed;
  }
  if (receivedChar == 'L')
  {
    Serial.println("L");
    speed += -1.0;
  }
  if (receivedChar == 'R')
  {
    Serial.println("R");
    speed += 1.0;
  }
  if (receivedChar == 'S')
  {
    Serial.println("Stopping motors.");
    stopped = true;
  }
  if (receivedChar == 'B')
  {
    Serial.println("refreshing");
    contini = currentTime;
  }

  if (speed > outputMax)
    speed = outputMax;
  if (speed < outputMin)
    speed = outputMin;
  // Serial.print("Speed: ");
  // Serial.println(speed);

  // // Serial.print("Setpoint: ");
  // // Serial.print(setpoint);
  // // Serial.print("    ");
  // Serial.print("Error: ");
  // Serial.print(error);
  // Serial.print("    ");
  // Serial.print("rollIntegral: ");
  // Serial.print(Ki*rollIntegral);
  // Serial.print("    ");
  // Serial.print("rollderivative: ");
  // Serial.println(Kd*rollderivative);
  // // Serial.print("    ");
  // // Serial.print("rollIntegral: ");
  // // Serial.print(rollIntegral);
  // // Serial.print("    ");
  // // Serial.print("rollderivative: ");
  // // Serial.println(rollderivative);
}