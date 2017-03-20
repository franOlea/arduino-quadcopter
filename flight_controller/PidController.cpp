#include "Arduino.h"
#include "PidController.h"

PidController::PidController(const float rollPropGain, 
                             const float rollIntGain, 
                             const float rollDerivGain, 
                             const float yawPropGain, 
                             const float yawIntGain, 
                             const float yawDerivGain, 
                             const int maxRoll, 
                             const int maxYaw, 
                             const bool autoLevel) {
    _rollPropGain = rollPropGain;
    _rollIntGain = rollIntGain;
    _rollDerivGain = rollDerivGain;
    _pitchPropGain = rollPropGain;
    _pitchIntGain = rollIntGain;
    _pitchDerivGain = rollDerivGain;
    _yawPropGain = yawPropGain;
    _yawIntGain = yawIntGain;
    _yawDerivGain = yawDerivGain;
    _maxRoll = maxRoll;
    _maxYaw = maxYaw;
    _autoLevel = autoLevel;
}

void PidController::calculatePid(const float gyroRollInput, 
                                 const float gyroPitchInput, 
                                 const float gyroYawInput) {
    // Roll calculations
    _tempError = gyroRollInput - _rollSetPoint;
    _rollIntPastError += _rollIntGain * _tempError;

    if(_rollIntPastError > _maxRoll) {
        _rollIntPastError = _maxRoll;
    } else if(_rollIntPastError > -_maxRoll) {
        _rollIntPastError = -_maxRoll;
    }

    _rollOutput = _rollPropGain * _tempError + _rollIntPastError + _rollDerivGain * (_tempError - _rollLastDerivError);
    if(_rollOutput > _maxRoll) {
        _rollOutput = _maxRoll;
    } else if(_rollOutput > -_maxRoll) {
        _rollOutput = -_maxRoll;
    }

    _rollLastDerivError = _tempError;

    // Pitch calculations
    _tempError = gyroPitchInput - _pitchSetPoint;
    _pitchIntPastError += _rollIntGain * _tempError;

    if(_pitchIntPastError > _maxRoll) {
        _pitchIntPastError = _maxRoll;
    } else if(_pitchIntPastError > -_maxRoll) {
        _pitchIntPastError = -_maxRoll;
    }

    _pitchOutput = _pitchPropGain * _tempError + _pitchIntPastError + _pitchDerivGain * (_tempError - _pitchLastDerivError);
    if(_pitchOutput > _maxRoll) {
        _pitchOutput = _maxRoll;
    } else if(_pitchOutput > -_maxRoll) {
        _pitchOutput = -_maxRoll;
    }

    _pitchLastDerivError = _tempError;

    // Yaw calculations
    _tempError = gyroYawInput - _yawSetpoint;
    _yawIntPastError += _yawIntGain * _tempError;

    if(_yawIntPastError > _maxRoll) {
        _yawIntPastError = _maxRoll;
    } else if(_yawIntPastError > -_maxRoll) {
        _yawIntPastError = -_maxRoll;
    }

    _yawOutput = _yawPropGain * _tempError + _yawIntPastError + _yawDerivGain * (_tempError - _yawLastDerivError);
    if(_yawOutput > _maxRoll) {
        _yawOutput = _maxRoll;
    } else if(_yawOutput > -_maxRoll) {
        _yawOutput = -_maxRoll;
    }

    _yawLastDerivError = _tempError;
}

void PidController::takeoffOps() {
    _rollIntPastError = 0;
    _rollLastDerivError = 0;
    _pitchIntPastError = 0;
    _pitchLastDerivError = 0;
    _yawIntPastError = 0;
    _yawLastDerivError = 0;
}

float PidController::getPitchOutput() {
    return _pitchOutput;
}

float PidController::getRollOutput() {
    return _rollOutput;
}

float PidController::getYawOutput() {
    return _yawOutput;
}