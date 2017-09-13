#include <TiGY521.h>

/**
* instanciating GY521
**/
#define GY521_INTERRUPT_PIN = 2;
TiGY521 acceleroGyro = TiGY521(2);

void setup() {
  /**
  * initializing GY521
  **/
  Serial.begin(115200);
  acceleroGyro.initialize();
}

void GYProcess(float y, float p, float r){
  if(y==0 && p==0 && r==0){
    //interrupt...
  }else{  
    Serial.print(y);
    Serial.print("\t");
    Serial.print(p);
    Serial.print("\t");
    Serial.println(r);
  }
}
  
void loop() {
  /**
  * Start reading
  **/
  float* reading = acceleroGyro.read();
  GYProcess(reading[0], reading[1], reading[2]);
}