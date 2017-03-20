#include "EEPROM.h"
#include "Barometer.h"
#include "PidController.h"
#include "MPU6050.h"

byte EEPROM_DATA[36];

MPU6050 gyroscope;
Barometer barometer;


double altitude = 0;


void setup() {
	readEEPROM();
	setOutputPins();
	
	if(isSystemReady() != 0) {
		//TODO Led error infinite loop
	}

//	MPU6050.initialize();

//  Serial.begin(9600);
//  Serial.println("REBOOT");
//  barometer.initialize();
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

void readEEPROM() {
	for(int i=0; i < 36; i++) {
		EEPROM_DATA[i] = EEPROM.read(i);
	}
}

void setOutputPins() {
	//TODO maps and sets output pins (ESCs & LED)
}

int isSystemReady() {
	if(EEPROM_DATA[33] != 'J' || EEPROM_DATA[34] != 'M' || EEPROM_DATA['B']) {
		return -1;
	}

	if(EEPROM_DATA[31] == 2 || EEPROM_DATA[31] == 3) {
		return -2;
	}

	return 0;
}