#ifndef Thruster_h
#define Thruster_h

#include <math.h>
#include <Servo.h>


// Utility class for 2-vectors
struct Point { float x, y; };
inline float dist(Point a, Point b) {
	float dx = b.x - a.x;
	float dy = b.y - a.y;
	return sqrt(dx*dx + dy*dy);
}


class ServoLink {
private:
	Servo _servo;
	float _zeroAngle;
	float _mountAngle;

public:
	// geometry, in mm
	static const float INNER_RADIUS;
	static const float OUTER_RADIUS;
	static const float LINK_LENGTH;
	static const float HORN_LENGTH;

	ServoLink(float zeroAngle, float mountAngle);
	void attach(int pin);
	void setLocation(float r, float theta);
};

class Thruster {
private:
	ServoLink _s1;
	ServoLink _s2;
	ServoLink _s3;

public:
	Thruster();
	void attach(int p1, int p2, int p3);
	void setLocation(float r, float theta);
};

#endif


