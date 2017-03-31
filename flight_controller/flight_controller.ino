#include "EEPROM.h"
#include "Barometer.h"
#include "PidController.h"
#include "MPU6050.h"

byte EEPROM_DATA[36];

float pidRollSetpoint, pidPitchSetpoint, pidYawSetpoint;

volatile int receiverInputChannelOne, receiverInputChannelTwo, receiverInputChannelThree, receiverInputChannelFour;
bool lastChannelOne, lastChannelTwo, lastChannelThree, lastChannelFour;
int escOne, escTwo, escThree, escFour;
int throttle, batteryVoltage;
int receiverInput[5];
int start;

unsigned long loopTimer;
unsigned long escOneTimer, escTwoTimer, escThreeTimer, escFourTimer, escLoopTimer;

unsigned long receiverChannelOneTimer, receiverChannelTwoTimer, receiverChannelThreeTimer, receiverChannelFourTimer;
unsigned long receiverCurrentTime;

double altitude = 0;
double groundDistance = 0;

MPU6050 gyroscope;
Barometer barometer;
PidController pidController(0, 0, 0, 0, 0, 0, 0, 0);

void setup() {
	  Serial.begin(9600);
	  Serial.println("REBOOT");
	//  barometer.initialize();
  //	doSetup();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  
  if(gyroscope.initialize(0, 1, 2, 0x68) != 0) {
    Serial.println("ERROR INITIALIZING GYRO.");
    while(1);
  } else {
    Serial.println("GYRO INITIALIZED");
  }
  Serial.println("CALIBRATING");
  gyroscope.calibrate();
  Serial.println("CALIBRATED");
}

void loop() {
	//for(int aux=0; aux<120; aux++) {
	//	delay(6);
	//	barometer.update();
	//	altitude += barometer.getAltitude();
	//}
	//Serial.println("alt: " + String(altitude/12));
	//altitude = 0;
  //	doLoop();
 
  gyroscope.requestGyroRead();
  gyroscope.readGyro(); 
  gyroscope.processData();
  Serial.print("ROLL: ");
  Serial.print(gyroscope.getRollInput());
  Serial.print(" | PITCH: ");
  Serial.print(gyroscope.getPitchInput());
  Serial.print(" | YAW: ");
  Serial.print(gyroscope.getYawInput());
  Serial.print(" | ROLL LEVEL ADJUST: ");
  Serial.print(gyroscope.getRollLevelAdjust());
  Serial.print(" | PITCH LEVEL ADJUST: ");
  Serial.print(gyroscope.getPitchLevelAdjust());
  Serial.println();
  delay(4);
}

void doSetup() {
	readEEPROM();
	setOutputPins();
	if(isSystemReady() != 0) {
		//TODO Led error infinite loop
	}
	if(gyroscope.initialize(EEPROM_DATA[28], EEPROM_DATA[29], EEPROM_DATA[30], 0x68) != 0) {
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

void doLoop() {
	gyroscope.processData();
	startAndStopCheck();
	calculatePidSetpoints();
	pidController.calculatePid(gyroscope.getRollInput(), pidRollSetpoint,
                               gyroscope.getPitchInput(), pidPitchSetpoint,
                               gyroscope.getYawInput(), pidYawSetpoint);
	checkBatteryVoltage();
	generateESCOutputs();
	checkLoopTimeoutAndWait();
	startESCPulses();
	gyroscope.requestGyroRead();
	convertReceiverChannels();
	gyroscope.readGyro();
}

void convertReceiverChannels() {
	receiverInputChannelOne = convertReceiverChannel(1);
	receiverInputChannelTwo = convertReceiverChannel(2);
	receiverInputChannelThree = convertReceiverChannel(3);
	receiverInputChannelFour = convertReceiverChannel(4);
}

void checkLoopTimeoutAndWait() {
	if(micros() - loopTimer > 4050) {
		//TODO Too much time problem handling
	}
	while(micros() - loopTimer < 4000);
}

void startAndStopCheck() {
	if(start = 0 && receiverInputChannelThree < 1050 && receiverInputChannelFour < 1050) {
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
}

void checkBatteryVoltage() {
	readBatteryVoltage();

	if(batteryVoltage < 1000 && batteryVoltage > 600) {
		// TODO Led LOW BATTERY SIGNAL
	}
}

void calculatePidSetpoints() {
	//The PID set point in degrees per second is determined by the roll receiver input.
	//In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
	pidRollSetpoint = 0;

	if(receiverInputChannelOne > 1508) {
		pidRollSetpoint = receiverInputChannelOne - 1508;
	} else if(receiverInputChannelOne < 1492) {
		pidRollSetpoint = receiverInputChannelOne - 1492;
	}

	pidRollSetpoint -= gyroscope.getRollLevelAdjust();
	pidRollSetpoint /= 3.0;

	pidPitchSetpoint = 0;

	if(receiverInputChannelTwo > 1508) {
		pidPitchSetpoint = receiverInputChannelTwo - 1508;
	} else if(receiverInputChannelTwo < 1492) {
		pidPitchSetpoint = receiverInputChannelTwo - 1492;
	}

	pidPitchSetpoint -= gyroscope.getPitchLevelAdjust();
	pidPitchSetpoint /= 3.0;

	pidYawSetpoint = 0;

	if(receiverInputChannelThree > 1050) {	//Dont yaw when turning motors off.
		if(receiverInputChannelFour > 1508) {
			pidYawSetpoint = (receiverInputChannelFour - 1508)/3.0;
		} else if(receiverInputChannelFour < 1492) {
			pidYawSetpoint = (receiverInputChannelFour - 1492)/3.0;
		}
	}
}

void startESCPulses() {
	loopTimer = micros();
	PORTD |= B11110000;
	escOneTimer = escOne + loopTimer;
	escTwoTimer = escTwo + loopTimer;
	escThreeTimer = escThree + loopTimer;
	escFourTimer = escFour + loopTimer;
}

void stopESCPulses() {
	while(PORTD >= 16) {
		escLoopTimer = micros();
		if(escOneTimer <= escLoopTimer) {
			PORTD &= B11101111;
		}
		if(escTwoTimer <= escLoopTimer) {
			PORTD &= B11011111;
		}
		if(escThreeTimer <= escLoopTimer) {
			PORTD &= B10111111;
		}
		if(escFourTimer <= escLoopTimer) {
			PORTD &= B01111111;
		}
	}
}

void generateESCOutputs() {
	throttle = receiverInputChannelThree;

	if(start == 2) {
		if(throttle > 1880) {
			throttle = 1880;
		}

		escOne = 	throttle - pidController.getPitchOutput() + pidController.getRollOutput() - pidController.getYawOutput();
		escTwo = 	throttle + pidController.getPitchOutput() + pidController.getRollOutput() + pidController.getYawOutput();
		escThree = 	throttle + pidController.getPitchOutput() - pidController.getRollOutput() - pidController.getYawOutput();
		escFour = 	throttle - pidController.getPitchOutput() - pidController.getRollOutput() + pidController.getYawOutput();

		if(batteryVoltage > 1240 && batteryVoltage < 800) {
			escOne = escOne 	* ((1240 - batteryVoltage)/(float)3500);
			escTwo = escTwo 	* ((1240 - batteryVoltage)/(float)3500);
			escThree = escThree * ((1240 - batteryVoltage)/(float)3500);
			escFour = escFour 	* ((1240 - batteryVoltage)/(float)3500);
		}

		if(escOne < 1100) {
			escOne = 1100;
		}

		if(escTwo < 1100) {
			escTwo = 1100;
		}

		if(escThree < 1100) {
			escThree = 1100;
		}

		if(escFour < 1100) {
			escFour = 1100;
		}

		if(escOne > 2000) {
			escOne = 2000;
		}

		if(escTwo > 2000) {
			escTwo = 2000;
		}

		if(escThree > 2000) {
			escThree = 2000;
		}

		if(escFour > 2000) {
			escFour = 2000;
		}
	} else {
		escOne = 1000;
		escTwo = 1000;
		escThree = 1000;
		escFour = 1000;
	}
}

void readEEPROM() {
	for(int i=0; i < 36; i++) {
		EEPROM_DATA[i] = EEPROM.read(i);
	}
}

void setOutputPins() {
	//TODO maps and sets output pins (ESCs & LED)
	DDRB |= B00011110;	//sets pins 12, 11, 10 and 9 as outputs.
}

void setInterrupts() {
	//Set PCIE2 to enable PCMK2 scan.
	//set radio pins to trigger on an interrupt on state change.
	PCICR |= (1 << PCIE1);
	PCMSK1 |= (1 << PCINT8);
	PCMSK1 |= (1 << PCINT9);
	PCMSK1 |= (1 << PCINT10);
	PCMSK1 |= (1 << PCINT11);
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
	//Load the battery voltage to the batteryVoltage variable.
	//65 is the voltage compensation for the diode.
	//12.6V equals ~5V @ Analog 0.
	//12.6V equals 1023 analogRead(0).
	//1260 / 1023 = 1.2317.
	//The variable batteryVoltage holds 1050 if the battery voltage is 10.5V.
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

ISR(PCINT1_vect) {
	receiverCurrentTime = micros();
	
	if(PINC & B00000001) {
		if(!lastChannelOne) {
			lastChannelOne = true;                                        
			receiverChannelOneTimer = receiverCurrentTime;
		}
	} else if(lastChannelOne) {
		lastChannelOne = false;
		receiverInput[1] = receiverCurrentTime - receiverChannelOneTimer;
	}
	
	if(PINC & B00000010) {
		if(!lastChannelTwo) {
			lastChannelTwo = true;                                        
			receiverChannelTwoTimer = receiverCurrentTime;
		}
	} else if(lastChannelTwo) {
		lastChannelTwo = false;
		receiverInput[2] = receiverCurrentTime - receiverChannelTwoTimer;
	}
	
	if(PINC & B00000100) {
		if(!lastChannelThree) {
			lastChannelThree = true;                                        
			receiverChannelThreeTimer = receiverCurrentTime;
		}
	} else if(lastChannelThree) {
		lastChannelThree = false;
		receiverInput[3] = receiverCurrentTime - receiverChannelThreeTimer;
	}
	
	if(PINC & B00001000) {
		if(!lastChannelFour) {
			lastChannelFour = true;                                        
			receiverChannelFourTimer = receiverCurrentTime;
		}
	} else if(lastChannelFour) {
		lastChannelFour = false;
		receiverInput[4] = receiverCurrentTime - receiverChannelFourTimer;
	}
}
