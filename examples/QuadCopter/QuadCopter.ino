#include <TiGY521.h>

/**
* instanciating GY521
**/
TiGY521 gyro = TiGY521();
int LED_PIN = 12;

void setup() {
  /**
  * initializing GY521
  **/
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Initializing..."));
  gyro.initialize();
  Serial.println(F("Caliberating..."));
  gyro.caliberate(LED_PIN);
}

void GYProcess(float y, float p, float r){
  if(y==0 && p==0 && r==0){
   Serial.println("No change in data...");
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
  gyro.read();
  GYProcess(gyro.getYaw(), gyro.getPitch(), gyro.getRoll());
}