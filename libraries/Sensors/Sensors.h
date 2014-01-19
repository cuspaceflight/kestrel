#ifndef Sensors_h
#define Sensors_h

#include <I2Cdev.h>
#include <BMP085.h>
#include <HMC5883L.h>
#include <ITG3200.h>
#include <ADXL345.h>

class Sensors {
public:
	BMP085 barometer;
	HMC5883L compass;
	ADXL345 accelerometer;
	ITG3200 gyro;

	void logConnections(File& f) {
		if(accelerometer.testConnection()) f.println("Accelerometer connected!");
		if(compass.testConnection()) f.println("Compass connected!");
		if(gyro.testConnection()) f.println("Gyro connected!");
		if(barometer.testConnection()) f.println("Barometer connected!");
	}

	void initialize() {
		// TODO: compass.initialize();
		compass.setMode(HMC5883L_MODE_CONTINUOUS);

		barometer.initialize(); //calibrate Barometer

		//configure the gyro
		gyro.setFullScaleRange(ITG3200_FULLSCALE_2000); //only ITG3200_FULLSCALE_2000 is documented
		gyro.setDLPFBandwidth(ITG3200_DLPF_BW_98);
		gyro.setRate(9); // = 100Hz - I think the time for a sample should match the loop time

		// configure accelerometer
		accelerometer.initialize();
		accelerometer.setRange(ADXL345_RANGE_16G);
		accelerometer.setRate(ADXL345_RATE_100); //Hz
		accelerometer.setMeasureEnabled(true);
		accelerometer.setOffset(0, 0, 0);  //for some strange reason setting these to 0 gave more false readings
	}
};

#endif
