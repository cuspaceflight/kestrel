#ifndef Sensors_h
#define Sensors_h

#include <stdint.h>

#define fabsf fabs
#undef abs
#undef round
#undef INTERNAL

#include <gmtl/gmtl.h>  // vector math
#include <gmtl/Vec.h>  // vector math

#include <I2Cdev.h>
#include <BMP085.h>
#include <HMC5883L.h>
#include <ITG3200.h>
#include <ADXL345.h>
#include <SD.h>

using namespace gmtl;

struct rocket_vector3f {
	float front; // out from switch
	float right; // with switch facing forward
	float up;    // towards nose cone
};

class Sensors {
private:
	Vec<int16_t, 3> gyroOffset;
	float gyroSensitivity; // deg.s^-1 / LSB
public:
	BMP085 barometer;
	HMC5883L compass;
	ADXL345 accelerometer;
	ITG3200 gyro;

	Sensors() {
		gyroOffset = Vec<int16_t, 3>(0, 0, 0);
		gyroSensitivity  = 1 / 14.375;
	}

	void logConnections(File& f);
	void initialize();
	void calibrate();
	Vec<float, 3> getAngularVelocity();
};

#endif
