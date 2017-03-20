#include "Barometer.h"
#include "PidController.h"
#include "MPU6050.h"

MPU6050 gyroscope;
Barometer barometer;
double altitude = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("REBOOT");
  barometer.initialize();
}

void loop() {
  for(int aux=0; aux<120; aux++) {
    delay(6);
    barometer.update();
    altitude += barometer.getAltitude();
  }
  Serial.println("alt: " + String(altitude/12));
  altitude = 0;
}