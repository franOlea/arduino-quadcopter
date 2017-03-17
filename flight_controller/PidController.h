#ifndef PIDCONTROLLER_h
#define PIDCONTROLLER_h

#include "Arduino.h"

class PidController {
    public:
        PidController(const float rollPropGain, 
                      const float rollIntGain, 
                      const float rollDerivGain, 
                      const float yawPropGain, 
                      const float yawIntGain, 
                      const float yawDerivGain, 
                      const int maxRoll, 
                      const int maxYaw, 
                      const bool autoLevel);
        void calculatePid(const float gyroRollInput, 
                          const float gyroPitchInput, 
                          const float gyroYawInput);
        float getRollOutput();
        float getPitchOutput();
        float getYawOutput();
    private:
        float _rollPropGain;
        float _rollIntGain;
        float _rollDerivGain;
        float _pitchPropGain;
        float _pitchIntGain;
        float _pitchDerivGain;
        float _yawPropGain;
        float _yawIntGain;
        float _yawDerivGain;
        int _maxRoll;
        int _maxYaw;
        bool _autoLevel;

        float _tempError;
        float _rollIntPastError, _rollSetPoint, _rollLastDerivError;
        float _rollOutput;
        float _pitchIntPastError, _pitchSetPoint, _pitchLastDerivError;
        float _pitchOutput;
        float _yawIntPastError, _yawSetpoint, _yawLastDerivError;
        float _yawOutput;
};

#endif