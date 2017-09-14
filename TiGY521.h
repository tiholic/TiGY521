/*
  TiGY521 - Library for Firing ESC's.
  Created by Rohit R. Abbadi, September 13, 2017.
  In the private domain.
*/
#ifndef TiGY521_h
#define TiGY521_h

#include "MPU6050_6Axis_MotionApps20.h"

class TiGY521{
	public:
      TiGY521();
      void initialize();
      void read();
      void caliberate(int LED_PIN);
      float getYaw();
      float getPitch();
      float getRoll();

  private:
    // bool isDirectionCaliberated(float** readings);
    bool isDirectionCaliberated(float* readings);
    void debug(char* message);
    void debugln(char* message);
    int readingCount = 60;
    float yawOffset = 0;
    float pitchOffset = 0;
    float rollOffset = 0;
    MPU6050 mpu;
    bool dmpReady;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
};

#endif