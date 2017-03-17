#ifndef BAROMETER_h
#define BAROMETER_h

#include "Arduino.h"
#include "SFE_BMP180.h"

class Barometer {
	public:
		Barometer();

		boolean initialize();
		void update();
		double getAltitude();
		double getTemperature();
	private:
		int _updateAux;
		double _baseLine;
		double _altitude;
		double _pressure;
		double _temperature;
		char _status;

		SFE_BMP180 _barometerSensor;

		double _getInitialPressure();
};

#endif