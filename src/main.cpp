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
float Ki = 1.5;
float Kd = 2.5;

// متغيرات PID
float integral = 0.0;
float derivative = 0.0;
float controlOutput5 = 0.0;
float derivativeMax = 200.0;
float integralMax = 200.0;
float controlOutput6 = 0.0;
float previousError = 0.0;
float setpoint = 0.0;
float currentAngle = 0.0;
float outputMax = 1900.0;
float outputMin = 1010.0;
// متغيرات الوقت
unsigned long currentTime;
unsigned long previousTime;
float deltaT;

void setup()
{
  esc5.attach(5, 1000, 2000);
  esc6.attach(6, 1000, 2000);
  esc5.writeMicroseconds(2000);
  esc6.writeMicroseconds(2000);

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
  mpu.setXAccelOffset(-281);
  mpu.setYAccelOffset(1042);
  mpu.setZAccelOffset(606);
  mpu.setXGyroOffset(149);
  mpu.setYGyroOffset(32);
  mpu.setZGyroOffset(50);

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
  delay(3000);
  esc5.writeMicroseconds(1000);
  esc6.writeMicroseconds(1000);
  Serial.println("low");
  delay(6000);

  previousTime = millis();
}

void loop()
{
  Serial.println(currentTime/1000);
  if (currentTime/1000 > 200) {
  esc5.writeMicroseconds(1000);
  esc6.writeMicroseconds(1000);
  Serial.println("MPU timeout — motors stopped.");
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
  currentAngle = ypr[ROLL] * (180 / M_PI);
  float error = setpoint - currentAngle;
  currentTime = millis();
  deltaT = (currentTime - previousTime) / 1000.0; // تحويل إلى ثوانٍ
  if (deltaT <= 0.1) deltaT = 0.1;
  previousTime = currentTime;
  integral = integral + (error * deltaT);
  // قيد التكامل ليكون ضمن الحدود
    if (integral > integralMax) integral = integralMax;
    if (integral < -integralMax) integral = -integralMax;
  derivative = (error - previousError) / deltaT;
  previousError = error;
  // قيد المشتق ليكون ضمن الحدود
  if (derivative > derivativeMax) derivative = derivativeMax;
  if (derivative < -derivativeMax)  derivative = -derivativeMax; 

  controlOutput5 = 1300 + (Kp * error) + (Ki * integral) + (Kd * derivative);
  controlOutput6 = 1300 - (Kp * error) + (Ki * integral) + (Kd * derivative);

  if (controlOutput5 > outputMax)
    controlOutput5 = outputMax;
  if (controlOutput5 < outputMin)
    controlOutput5 = outputMin;
  if (controlOutput6 > outputMax)
    controlOutput6 = outputMax;
  if (controlOutput6 < outputMin)
    controlOutput6 = outputMin;

  esc5.writeMicroseconds(controlOutput5);
  esc6.writeMicroseconds(controlOutput6);

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

  if (receivedChar == 'N') setpoint = 0;

  if (receivedChar == 'L') setpoint = 10.0;

  if (receivedChar == 'R') setpoint = -10.0;

  // Serial.print("Setpoint: ");
  // Serial.print(setpoint);
  // Serial.print("    ");
  // Serial.print("Error: ");
  // Serial.print(error);
  // Serial.print("    ");
  // Serial.print("Integral: ");
  // Serial.print(integral);
  // Serial.print("    ");
  // Serial.print("Derivative: ");
  // Serial.println(derivative);
}