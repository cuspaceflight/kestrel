#include <Thruster.h>

const float ServoLink::HORN_LENGTH = 12; // the length of the yellow arms in mm
const float ServoLink::LINK_LENGTH = 28; // the length of the metal rods in mm
const float ServoLink::INNER_RADIUS = 20; // the radius to the inner hinge points
const float ServoLink::OUTER_RADIUS = 48; // the radius to the center of the horn

ServoLink::ServoLink(float zeroAngle, float mountAngle) : _zeroAngle(zeroAngle), _mountAngle(mountAngle) { }

inline float sq(float x) { return x * x; }

void ServoLink::setLocation(float r, float theta) {

	// center of rocket engine (assume planar movement)
	Point center = {
		r*cos(theta),
		r*sin(theta)
	};
	// unit vector pointing radially
	Point radial = {
		cos(_mountAngle),
		sin(_mountAngle)
	};

	// location of inner metal hinge
	Point innerHinge = {
		center.x + radial.x * INNER_RADIUS,
		center.y + radial.y * INNER_RADIUS
	};

	//location of outer yellow hinge
	Point outerHinge = {
		radial.x * OUTER_RADIUS,
		radial.y * OUTER_RADIUS
	};

	// distance between inner and outer hinge
	float s = dist(innerHinge, outerHinge);

	// change in size required
	s -= LINK_LENGTH;

	// here be dragons
	const float link = LINK_LENGTH,
	            horn = HORN_LENGTH;
	float temp = (2*s + 2*link - 1.0/(2*horn) - sqrt(sq(1.0/(2*horn) - 2*link - 2*s) - 4*(sq(s) + 2*link*s)*(1 + 1/(8*pow(horn,3)))))
	           / (2*(1 + 1/(8*pow(horn,3))));

	// convert into servo degrees 180/M_PI = 57
	float angle = 57*asin(temp/horn);

	//update servos with new positions
	_servo.write(angle + _zeroAngle);
}

void ServoLink::attach(int pin) {
	_servo.attach(pin);
}

// TODO - check directionality of theta
Thruster::Thruster()
	: _s1(95, 0),
	  _s2(90, 2*M_PI / 3),
	  _s3(90, -2*M_PI / 3) { }

void Thruster::attach(int p1, int p2, int p3) {
	_s1.attach(p1);
	_s2.attach(p2);
	_s3.attach(p3);
}

void Thruster::setLocation(float r, float theta) {
	// TODO - separate calculation and updating to improve simultaneity?
	_s1.setLocation(r, theta);
	_s2.setLocation(r, theta);
	_s3.setLocation(r, theta);
}
