/*
  TiGY521 - Library for Firing ESC's.
  Modified by Rohit R. Abbadi<rr.16566@gmail.com>, September 13, 2017.

// Originally taken from
//
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "Arduino.h"
#include "TiGY521.h"
#include "I2Cdev.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define TiGY521_INTERRUPT_PIN 2

// MPU6050 mpu;
TiGY521::TiGY521(){
	// MPU6050 mpu;
	bool dmpReady = false;  // set true if DMP init was successful
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	// orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	
	// join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    pinMode(TiGY521_INTERRUPT_PIN, INPUT);
}

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void TiGY521::initialize(){
    
    mpu.initialize();
    // verify connection
    debug("Testing device connections...");
    debugln(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // load and configure the DMP
    debugln("Initializing DMP...");
    uint8_t devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    //mpu.setXGyroOffset(20);
    //mpu.setYGyroOffset(15);
    //mpu.setZGyroOffset(52);
    //mpu.setXAccelOffset(-2772);
    //mpu.setYAccelOffset(771);
    //mpu.setZAccelOffset(4403); // 1688 factory default for my test chip

    //defaults
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        debugln("Enabling DMP...");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        debugln("Enabling interrupt detection (Arduino external interrupt 0)...");
        attachInterrupt(digitalPinToInterrupt(TiGY521_INTERRUPT_PIN), dmpDataReady, RISING);

        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        debugln("DMP ready! Waiting for first interrupt...");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        // fifoCount = 0;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        debug("DMP Initialization failed - code:");
        debugln(devStatus);
    }
}

float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
void TiGY521::read(){
	// if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    // if un unterrupted, 
    /*if (!mpuInterrupt && fifoCount < packetSize) {*/
    /*if (!mpuInterrupt) {
        ypr[0] = 0;
        ypr[1] = 0;
        ypr[2] = 0;
        return;
    }*/

    // reset interrupt flag and get INT_STATUS byte
    // mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        debugln("FIFO overflow!");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorFloat gravity;    // [x, y, z]            gravity vector
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        ypr[0] = ypr[0] * 180/M_PI;
		ypr[1] = ypr[1] * 180/M_PI;
		ypr[2] = ypr[2] * 180/M_PI;
    }
}

float TiGY521::getYaw() {
    return ypr[0] - yawOffset;
}

float TiGY521::getPitch() {
    return ypr[1] - pitchOffset;
}

float TiGY521::getRoll() {
    return ypr[2] - rollOffset;
}

bool TiGY521::isDirectionCaliberated(float* readings){
    float avg = 0;
    int i=0;
    for(i=0; i<readingCount; i++){
        avg += readings[i];
    }
    avg = (avg/readingCount);
    float readDiff = abs(readings[readingCount-1] - readings[0]);
    float avgDiff1 = abs(readings[0] - avg);
    float avgDiff2 = abs(readings[readingCount-1] - avg);
    float minDiff = 0.01;
    Serial.print(":::");
    Serial.print(readDiff);
    Serial.print(" - ");
    Serial.print(avgDiff1);
    Serial.print(" - ");
    Serial.print(avgDiff2);
    Serial.print(" - ");
    Serial.print(minDiff);
    Serial.print(":::");
    if( readDiff < minDiff && avgDiff1 < minDiff && avgDiff2 < minDiff) {
        return true;
    }
    return false;
}

void TiGY521::caliberate(int LED_PIN) {
    float yaws[readingCount];
    float pitches[readingCount];
    float rolls[readingCount];
    int tssc = 20;
    int ssc = tssc;
    while( true ) {
        Serial.print("Still caliberating...::");
        int i = 0;
        // Serial.print(readingCount);
        for(i=0; i<readingCount; i++){
            read();
            yaws[i] = getYaw();
            pitches[i] = getPitch();
            rolls[i] = getRoll();
        }
        digitalWrite(LED_PIN, LOW);
        bool yc = isDirectionCaliberated(yaws);
        bool pc = isDirectionCaliberated(pitches);
        bool rc = isDirectionCaliberated(rolls);
        if(yc || pc || rc){
            digitalWrite(LED_PIN, HIGH);
        }else{
            digitalWrite(LED_PIN, LOW);
        }
        if( yc && pc && rc ){
            ssc--;
            Serial.print("caliberated... Checking for successive caliberations.");
        }else{
            ssc = tssc;
        }
        Serial.println();
        if(ssc==0){
            Serial.println("caliberation complete... (y)");
            break;
        }
    }
    read();
    yawOffset = getYaw();
    pitchOffset = getPitch();
    rollOffset = getRoll();
    Serial.print("Offsets: Y:: ");
    Serial.print(yawOffset);
    Serial.print(" P:: ");
    Serial.print(pitchOffset);
    Serial.print(" R:: ");
    Serial.println(rollOffset);
}

void TiGY521::debug(char* message) {
	if(!Serial){
		Serial.begin(115200);
	}
	Serial.print(message);
}

void TiGY521::debugln(char* message) {
    if(!Serial){
        Serial.begin(115200);
    }
    Serial.println(message);
}