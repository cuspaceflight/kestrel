#ifndef Sensors_h
#define Sensors_h

#include <I2Cdev.h>
#include <BMP085.h>
#include <HMC5883L.h>
#include <ITG3200.h>
#include <ADXL345.h>
#include <SD.h>

template<typename T>
struct vector3 {
	T x; 
	T y;
	T z;

	inline vector3<T> operator+(const vector3<T>& v) const {
		return (vector3<T>){x + v.x, y + v.y, z + v.z};
	}
	inline vector3<T>& operator+=(const vector3<T>& v) {
		x += v.x; y += v.y; z += v.z; return *this;
	}

	inline vector3<T> operator-(const vector3<T>& v) const {
		return (vector3<T>){x - v.x, y - v.y, z - v.z};
	}
	inline vector3<T>& operator-=(const vector3<T>& v) {
		x -= v.x; y -= v.y; z -= v.z; return *this;
	}

	inline vector3<T> operator/(T k) {
		return (vector3<T>){x / k, y / k, z / k};
	}
	inline vector3<T> operator*(T k) {
		return (vector3<T>){x * k, y * k, z * k};
	}

	template<typename U>
	inline operator vector3<U>() {
		return (vector3<U>){(U) x, (U) y, (U) z};
	}
};

struct rocket_vector3f {
	float front; // out from switch
	float right; // with switch facing forward
	float up;    // towards nose cone
};

class Sensors {
private:
	vector3<int16_t> gyroOffset;
	float gyroSensitivity; // deg.s^-1 / LSB
public:
	BMP085 barometer;
	HMC5883L compass;
	ADXL345 accelerometer;
	ITG3200 gyro;

	Sensors() {
		gyroOffset = (vector3<float>){0, 0, 0};
		gyroSensitivity  = 1 / 14.375;
	}

	void logConnections(File& f);
	void initialize();
	void calibrate();
	vector3<float> getAngularVelocity();
};

#endif
