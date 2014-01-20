#include <Sensors.h>

void Sensors::logConnections(File& f) {
	if(accelerometer.testConnection()) f.println("Accelerometer connected!");
	if(compass.testConnection()) f.println("Compass connected!");
	if(gyro.testConnection()) f.println("Gyro connected!");
	if(barometer.testConnection()) f.println("Barometer connected!");
}

void Sensors::initialize() {
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

void Sensors::calibrate() {
	const int n = 50;

	gyroOffset = Vec<int16_t, 3>(0, 0, 0);

	Vec<int16_t, 3> sum(0, 0, 0);
	for (uint8_t i=0; i<n; i++) {
		Vec<int16_t, 3> temp;
		gyro.getRotation(&temp[0], &temp[1], &temp[2]);
		gyroOffset += temp;
		delay(10);
	}

	// TODO: check no overflow occured above
	gyroOffset = gyroOffset / n;

	// TODO: calibrate accelerometer/record magnemometer?
}

Vec3f Sensors::getAngularVelocity() {
	Vec<int16_t, 3> temp;
	gyro.getRotation(&temp[0], &temp[1], &temp[2]);
	temp -= gyroOffset;

	Vec3f result(temp[0], temp[1], temp[2]);

	return result * gyroSensitivity;
}
