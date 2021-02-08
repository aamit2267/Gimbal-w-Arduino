// Copyright © 2021 - https://github.com/aamit2267/Gimbal-w-Arduino/
// Author - Amit Agarwal
// Give credit if you are copying the code.

// add Dependencies
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <Servo.h>
// Dependencies Complete

MPU6050 mpu; // store MPU6050 in mpu variable

int i = 0;
Servo servo0; // Servo number 0.
Servo servo1; // Servo number 1.
Servo servo2; // Servo number 2.
float correct;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino-uno

bool dmpReady = false; 
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    // expected DMP size (default = 42 bytes)
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 
bool blinkState = false;

Quaternion q;           
VectorFloat gravity;        
float tr[3];       


uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



volatile bool mpuInterrupt = false;    
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); 
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(38400);
  while (!Serial); 
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(17);
  mpu.setYGyroOffset(-69);
  mpu.setZGyroOffset(27);
  mpu.setZAccelOffset(1551);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    
    dmpReady = true;
  }
  servo0.attach(10);
  servo1.attach(9);
  servo2.attach(8);
}

void loop() {
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
     
      fifoCount = mpu.getFIFOCount();
    }
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));

    
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(tr, &q, &gravity);

    tr[0] = tr[0] * 180 / M_PI;
    tr[1] = tr[1] * 180 / M_PI;
    tr[2] = tr[2] * 180 / M_PI;
    
    if (i <= 300) {
      correct = tr[0];
      i++;
    }
    
    else {
      tr[0] = tr[0] - correct;
      
      int servo0Val = map(tr[0], -90, 90, 0, 180);
      int servo1Val = map(tr[1], -90, 90, 0, 180);
      int servo2Val = map(tr[2], -90, 90, 180, 0);
      
      
      servo0.write(servo0Val);
      servo1.write(servo1Val);
      servo2.write(servo2Val);
    }
#endif
  }
}
