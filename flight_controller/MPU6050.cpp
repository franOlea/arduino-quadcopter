#include "Arduino.h"
#include "Wire.h"
#include "MPU6050.h"

MPU6050::MPU6050() {
}

void MPU6050::takeoffOps() { 
    _pitchAngle = _accelPitchAngle;
    _rollAngle = _accelRollAngle;
}

float MPU6050::getRollInput() {
    return _rollInput;
}

float MPU6050::getPitchInput() {
    return _pitchInput;
}

float MPU6050::getYawInput() {
    return _yawInput;
}

float MPU6050::getRollLevelAdjust() {
    return _rollLevelAdjust;
}

float MPU6050::getPitchLevelAdjust() {
    return _pitchLevelAdjust;
}

int MPU6050::processData() {
    if(!_hasDataToProcess) {
        return -1;
    }

    // Gyro PID input in deg/seg.
    _rollInput = (_rollInput * 0.7) + ((_gyroRoll / 65.5) * 0.3);
    _pitchInput = (_pitchInput * 0.7) + ((_gyroPitch / 65.5) * 0.3);
    _yawInput = (_yawInput * 0.7) + ((_gyroYaw / 65.5) * 0.3);

    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    _rollAngle += _gyroRoll * 0.0000611;    //Calculates traveled roll angle and adds to the _rollAngle variable.
    _pitchAngle += _gyroPitch * 0.0000611;  //Calculates traveled pitch angle and adds to the _pitchAngle variable.

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
    _pitchAngle -= _rollAngle * sin(_gyroYaw * 0.000001066);    //If the IMU has yawed transfers the roll angle to the pitch angle.
    _rollAngle += _pitchAngle * sin(_gyroYaw * 0.000001066);    //If the IMU has yawed transfers the roll angle to the roll angle.

    //Accelerometer angle calculations
    _accelTotalVector = sqrt((_accelX*_accelX) + (_accelY*_accelY) + (_accelZ*_accelZ));    //Calculate the total accelerometer vector.
  
    if(abs(_accelY) < _accelTotalVector) {                                  //Prevents the asin function to produce a NaN
        _accelPitchAngle = asin((float)_accelY/_accelTotalVector)* 57.296;  //Calculates the accelerometer's pitch angle
    }
    if(abs(_accelX) < _accelTotalVector){                                   //Prevents the asin function to produce a NaN
        _accelRollAngle = asin((float)_accelX/_accelTotalVector)* -57.296;  //Calculates the accelerometer's roll angle.
    }

    _pitchAngle = _pitchAngle * 0.9996 + _accelPitchAngle * 0.0004; //Corrects the drift of the gyro with the accelerometer.
    _rollAngle = _rollAngle * 0.9996 + _accelRollAngle * 0.0004;    //Corrects the drift of the gyro with the accelerometer.

    _pitchLevelAdjust = _pitchAngle * 15;   //Calculates the pitch angle correction
    _rollLevelAdjust = _rollAngle * 15;     //Calculates the roll angle correction

    _hasDataToProcess = false;
    return 0;
}

int MPU6050::initialize(int rollAxis, int pitchAxis, int yawAxis, int gyroAddress) {
    Wire.begin();
    TWBR = 12;  //Set the I2C clock speed to 400kHz.

    _correctAxis[0] = rollAxis;
    _correctAxis[1] = pitchAxis;
    _correctAxis[2] = yawAxis;
    _gyroAddress = gyroAddress;

    Wire.beginTransmission(_gyroAddress);
    Wire.write(0x6B);       //Writes to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);       //Sets the register bits as 00000000 to activate the gyro
    Wire.endTransmission();

    Wire.beginTransmission(_gyroAddress);
    Wire.write(0x1B);       //Writes to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);       //Sets the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();

    Wire.beginTransmission(_gyroAddress);
    Wire.write(0x1C);       //Writes to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);       //Sets the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();

    //Performs a register check to see if the written values are correct
    Wire.beginTransmission(_gyroAddress);
    Wire.write(0x1B);                   //Starts reading @ register 0x1B
    Wire.endTransmission();
    Wire.requestFrom(_gyroAddress, 1);  //Requests 1 bytes from the gyro
    while(Wire.available() < 1);        //Waits until the 6 bytes are received
    if(Wire.read() != 0x08) {           //Checks if the value is 0x08
        return -1;                      //Setup error
    }

    Wire.beginTransmission(_gyroAddress);
    Wire.write(0x1A);       //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);       //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();

    return 0;
}

void MPU6050::calibrate() {
    for(int calInt=0; calInt<2000; calInt++) {
        requestGyroRead();
        readGyro();
        _gyroAxisCalibration[0] += _gyroAxis[0];
        _gyroAxisCalibration[1] += _gyroAxis[1];
        _gyroAxisCalibration[2] += _gyroAxis[2];
    }

    _gyroAxisCalibration[0] /= 2000;
    _gyroAxisCalibration[1] /= 2000;
    _gyroAxisCalibration[2] /= 2000;

    _isCalibrated = true;
}

void MPU6050::requestGyroRead() {
    Wire.beginTransmission(_gyroAddress);
    Wire.write(0x3B);                   //Starts reading @ register 43h and auto increment with every read.
    Wire.endTransmission();             
    Wire.requestFrom(_gyroAddress,14);  //Requests 14 bytes from the gyro.

    _hasMadeRequest = true;
}

int MPU6050::readGyro() {
    if(!_hasMadeRequest) {
        return -1;
    }

    while(Wire.available()<14);                   //Waits until the 14 bytes are available.
    _accelAxis[0] = Wire.read()<<8|Wire.read(); //Reads the low and high byte to the X acc axis variable.
    _accelAxis[1] = Wire.read()<<8|Wire.read(); //Reads the low and high byte to the Y acc axis varaible.
    _accelAxis[2] = Wire.read()<<8|Wire.read(); //Reads the log and high byte to the Z acc axis variable.
    _temperature = Wire.read()<<8|Wire.read();  //Reads the low and high byte to the temperature variable.
    _gyroAxis[0] = Wire.read()<<8|Wire.read();  //Reads high and low part of the angular data.
    _gyroAxis[1] = Wire.read()<<8|Wire.read();  //Reads high and low part of the angular data.
    _gyroAxis[2] = Wire.read()<<8|Wire.read();  //Reads high and low part of the angular data.

    if(_isCalibrated) {
        _gyroAxis[0] -= _gyroAxisCalibration[0];
        _gyroAxis[1] -= _gyroAxisCalibration[1];
        _gyroAxis[2] -= _gyroAxisCalibration[2];
    }

    _gyroRoll = _gyroAxis[_correctAxis[0] & 0b00000011];    //Sets gyroRoll to the correct axis that was stored in EEPROM.
    if(_correctAxis[0] & 0b10000000) {
        _gyroRoll = -_gyroRoll;                             //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
    }
    _gyroPitch = _gyroAxis[_correctAxis[1] & 0b00000011];   //Sets gyroPitch to the correct axis that was stored in EEPROM.
    if(_correctAxis[1] & 0b10000000) {
        _gyroPitch = -_gyroPitch;                           //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
    }
    _gyroYaw = _gyroAxis[_correctAxis[2] & 0b00000011];     //Sets gyroYaw to the correct axis that was stored in EEPROM.
    if(_correctAxis[2] & 0b10000000) {
        _gyroYaw = -_gyroYaw;                               //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.
    }

    _accelX = _accelAxis[_correctAxis[1] & 0b00000011];     //Sets accel_x to the correct axis that was stored in EEPROM.
    if(_correctAxis[1] & 0b10000000) {
        _accelX = -_accelX;                                 //Invert accel_x if the MSB of EEPROM bit 29 is set.
    }
    _accelY = _accelAxis[_correctAxis[0] & 0b00000011];     //Sets accel_y to the correct axis that was stored in EEPROM.
    if(_correctAxis[0] & 0b10000000) {
        _accelY = -_accelY;                                 //Invert accel_y if the MSB of EEPROM bit 28 is set.
    }
    _accelZ = _accelAxis[_correctAxis[2] & 0b00000011];     //Sets accel_z to the correct axis that was stored in EEPROM.
    if(_correctAxis[2] & 0b10000000) {
        _accelZ = -_accelZ;                                 //Invert accel_z if the MSB of EEPROM bit 30 is set.
    }

    _hasMadeRequest = false;
    _hasDataToProcess = true;
}
