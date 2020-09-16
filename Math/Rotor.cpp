#include "Rotor.h";
#include <math.h>

Rotor::Rotor(double a, double b, double c, double d) {
	this->a = a;
	this->b = b;
	this->c = c;
	this->d = d;
}

Rotor::Rotor() {
	this->a = 1;
	this->b = 0;
	this->c = 0;
	this->d = 0;
}

Rotor::Rotor(Vector3D unitAxis, double theta) {
	this->a = cos(theta / 2.0);
	double s = sin(theta / 2.0);
	this->b = -s * unitAxis.z; //e1e2
	this->c = -s * unitAxis.x; //e2e3
	this->d = -s * unitAxis.y; //e3e1
}

Vector3D Rotor::rotate(Vector3D p) {
	/* RpR^-1									:: p-vector
	= (a + B)p(a - B)							:: a-scalar, B-Bivector 
	= (ap + Bp)(a - B)
	= (ap + u + t)(a - B)						:: u-vector output Bp, t-trivector output Bp
	= (aap + au + at - apB + uB + tB)
	= (aap + au + at - a(-u + t) - uB - tB)
	= (aap + 2au - uB - tB)
	= (aap + 2au - k - w)						:: k-vector output uB (no trivector output), w-vector output tB
	*/


	Vector3D u(b * p.y - d * p.z, c * p.z - b * p.x, d * p.x - c * p.y);
	Vector3D k(-b * u.y + d * u.z, -c * u.z + b * u.x, -d * u.x + c * u.y);
	Vector3D w = Vector3D(-c, -d, -b).multiply(b * p.z + c * p.x + d * p.y);
	return p.multiply(a * a).add(u.multiply(2 * a)).sub(k).sub(w);
}

//multiplying r from left
Rotor Rotor::applyRotor(Rotor r) {
	double aVal = a * r.a - b * r.b - c * r.c - d * r.d;
	double bVal = a * r.b + r.a * b + (r.d * c - r.c * d);
	double cVal = a * r.c + r.a * c + (r.b * d - r.d * b);
	double dVal = a * r.d + r.a * d + (r.c * b - r.b * c);
	return Rotor(aVal, bVal, cVal, dVal);
}

Rotor Rotor::getInverse() {
	return Rotor(a, -b, -c, -d);
}

bool Rotor::operator!=(const Rotor& r) {
	return a != r.a || b != r.b || c != r.c || d != r.d;
}

bool Rotor::operator==(const Rotor& r) {
	return a == r.a && b == r.b && c == r.c && d == r.d;
}