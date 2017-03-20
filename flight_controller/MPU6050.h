#ifndef MPU6050_h
#define MPU6050_h

#include "Arduino.h"
#include "Wire.h"

class MPU6050 {
    public:
        MPU6050();

        int initialize(int rollAxis, int pitchAxis, int yawAxis);
        void calibrate();
        void requestGyroRead();
        void readGyro();

        float getPitchAngle();
        float getRollAngle();
    private:
        int _gyroAddress;
        bool _isCalibrated;

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
};

#endif