#include "EEPROM.h"
#include "Barometer.h"
#include "PidController.h"
#include "MPU6050.h"

byte EEPROM_DATA[36];

volatile int receiverInputChannelOne, receiverInputChannelTwo, receiverInputChannelThree, receiverInputChannelFour;
int escOne, escTwo, escThree, escFour;
int throttle, batteryVoltage;
int receiverInput[5];
int start;

unsigned long loopTimer;

MPU6050 gyroscope;
Barometer barometer;
PidController pidController(0, 0, 0, 0, 0, 0, 0, 0);


double altitude = 0;


void setup() {
//  Serial.begin(9600);
//  Serial.println("REBOOT");
//  barometer.initialize();

	readEEPROM();
	setOutputPins();
	if(isSystemReady() != 0) {
		//TODO Led error infinite loop
	}
	if(gyroscope.initialize(EEPROM_DATA[28], EEPROM_DATA[29], EEPROM_DATA[30]) != 0) {
		//TODO Led error infinite loop
	}
	gyroscope.calibrate();
	gyroscope.requestGyroRead();
	gyroscope.readGyro();
	setInterrupts();
	waitForThrottleDown();
	readBatteryVoltage();
	loopTimer = micros();
}

void loop() {
	//for(int aux=0; aux<120; aux++) {
	//	delay(6);
	//	barometer.update();
	//	altitude += barometer.getAltitude();
	//}
	//Serial.println("alt: " + String(altitude/12));
	//altitude = 0;

	gyroscope.processData();

	if(receiverInputChannelThree < 1050 && receiverInputChannelFour < 1050) {
		start = 1;	//Pre-engine start.
	}

	if(start == 1 && receiverInputChannelThree < 1050 && receiverInputChannelFour > 1450) {
		start = 2; //Engines start.
		gyroscope.takeoffOps();		//Sets the angles zero to the accelerometers measurements.
		pidController.takeoffOps();	//Sets past errors to 0.
	}

	if(start == 2 && receiverInputChannelThree < 1050 && receiverInputChannelFour > 1950) {
		start = 0;	//Engines stop.
	}

	//The PID set point in degrees per second is determined by the roll receiver input.
	//In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
	//pid_roll_setpoint = 0;


}

void readEEPROM() {
	for(int i=0; i < 36; i++) {
		EEPROM_DATA[i] = EEPROM.read(i);
	}
}

void setOutputPins() {
	//TODO maps and sets output pins (ESCs & LED)
}

void setInterrupts() {
	//Set PCIE0 to enable PCMK0 scan.
	//set radio pins to trigger on an interrupt on state change.
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

void waitForThrottleDown() {
	while(receiverInputChannelThree < 999 || receiverInputChannelThree > 1020 || receiverInputChannelFour < 1400) {
		receiverInputChannelThree = convertReceiverChannel(3);
		receiverInputChannelFour = convertReceiverChannel(4);
	}
	start = 0;
}

void readBatteryVoltage() {
	//Load the battery voltage to the battery_voltage variable.
	//65 is the voltage compensation for the diode.
	//12.6V equals ~5V @ Analog 0.
	//12.6V equals 1023 analogRead(0).
	//1260 / 1023 = 1.2317.
	//The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
	batteryVoltage = (analogRead(0) + 65) * 1.2317;
}

int convertReceiverChannel(byte function) {
	byte channel;
	bool reverse;
	int low, center, high, actual;
	int difference;

	channel = EEPROM_DATA[function + 23] & 0b00000111;
	reverse = EEPROM_DATA[function + 23] & 0b10000000;

	actual = receiverInput[channel];
	low = (EEPROM_DATA[channel * 2 + 15] << 8 | EEPROM_DATA[channel * 2 + 14]);
	center = (EEPROM_DATA[channel * 2 - 1] << 8 | EEPROM_DATA[channel * 2 - 2]);
	high = (EEPROM_DATA[channel * 2 + 7] << 8 | EEPROM_DATA[channel * 2 + 6]);
	
	if(actual < center) {	//The actual receiver value is lower than the center value
		if(actual < low) {
			actual = low;	//Limit the lowest value to the value that was detected during setup
		}
		difference = ((long)(center - actual) * (long)500) / (center - low);	//Calculate and scale the actual value to a 1000 - 2000us value
		if(reverse) {
			return 1500 + difference;
		} else {
			return 1500 - difference;
		}
	} else if(actual > center) {                                                                        //The actual receiver value is higher than the center value
		if(actual > high) {
			actual = high;	//Limit the lowest value to the value that was detected during setup
		}
		difference = ((long)(actual - center) * (long)500) / (high - center);	//Calculate and scale the actual value to a 1000 - 2000us value
		if(reverse == 1) {
			return 1500 - difference;
		} else {
			return 1500 + difference;
		}
	} else {
		return 1500;
	}
}