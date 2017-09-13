#include <TiGY521.h>

/**
* instanciating GY521
**/
TiGY521 acceleroGyro = TiGY521();

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
    Serial.println(y);
    Serial.println(p);
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