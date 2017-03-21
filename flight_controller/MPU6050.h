#ifndef MPU6050_h
#define MPU6050_h

#include "Arduino.h"
#include "Wire.h"

class MPU6050 {
    public:
        MPU6050();

        int initialize(int rollAxis, int pitchAxis, int yawAxis);
        void takeoffOps();
        void calibrate();
        void requestGyroRead();
        int readGyro();
        int processData();

        float getRollInput();
        float getPitchInput();
        float getYawInput();
    private:
        int _gyroAddress;
        bool _isCalibrated;
        bool _hasDataToProcess;
        bool _hasMadeRequest;

        int _correctAxis[3];
        int _gyroAxis[3];
        int _accelAxis[3];
        int _temperature;

        long _accelX, _accelY, _accelZ, _accelTotalVector;
        
        double _gyroAxisCalibration[3];
        double _gyroPitch;
        double _gyroRoll;
        double _gyroYaw;
        
        float _accelPitchAngle, _accelRollAngle;
        float _pitchAngle, _rollAngle;

        float _rollInput, _pitchInput, _yawInput;
        float _pitchLevelAdjust, _rollLevelAdjust;
};

#endif