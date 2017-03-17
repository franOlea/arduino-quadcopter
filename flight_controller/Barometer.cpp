#include "Arduino.h"
#include "Barometer.h"
#include "SFE_BMP180.h"

Barometer::Barometer() {
    _updateAux = 0;
}

boolean Barometer::initialize() {
    if(!_barometerSensor.begin()) {
        return false;
    }

    _baseLine = _getInitialPressure();
    _altitude = _barometerSensor.altitude(_pressure, _baseLine);
}

void Barometer::update() {
    if(_updateAux == 0) {
        _status = _barometerSensor.startTemperature();
        if(_status == 0) {
            _updateAux = 0;
            return;
        }
    }

    if(_updateAux == 2) {
        _status = _barometerSensor.getTemperature(_temperature);
        if(_status == 0) {
            _updateAux = 0;
            return;
        }
    }

    if(_updateAux == 3) {
        _status = _barometerSensor.startPressure(3);
        if(_status == 0) {
            _updateAux = 0;
            return;
        }
    }

    if(_updateAux == 10) {
        _status = _barometerSensor.getPressure(_pressure, _temperature);
        if(_status == 0) {
            _updateAux = 0;
            return;
        }
    }

    if(_updateAux == 11) {
        _altitude = _barometerSensor.altitude(_pressure, _baseLine);
    }

    _updateAux++;

    if(_updateAux > 11) {
        _updateAux = 0;
    }
}

double Barometer::_getInitialPressure() {
    _status = _barometerSensor.startTemperature();
    if(_status != 0) {
        delay(_status);
        _status = _barometerSensor.getTemperature(_temperature);
        if(_status != 0) {
            _status = _barometerSensor.startPressure(3);
            if(_status != 0) {
                delay(_status);
                _status = _barometerSensor.getPressure(_pressure, _temperature);
                if(_status != 0) {
                    _status = 0;
                    return _pressure;
                } else {
                    return -4;
                }
            } else {
                return -3;
            }
        } else {
            return -2;
        }
    } else {
        return -1;
    }
}

double Barometer::getTemperature() {
    return _temperature;
}

double Barometer::getAltitude() {
    return _altitude;
}
