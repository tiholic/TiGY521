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
	    float* read();

    private:
      void debug(const char* message);
      MPU6050 mpu;
      bool dmpReady;  // set true if DMP init was successful
      uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
      uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
      uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
      uint16_t fifoCount;     // count of all bytes currently in FIFO
      uint8_t fifoBuffer[64]; // FIFO storage buffer
};

#endif