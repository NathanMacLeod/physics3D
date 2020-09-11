#pragma once
#include "Vector3D.h"

struct Rotor {
	double a, b, c, d;
	Rotor(double a, double b, double c, double d);
	Rotor(Vector3D unitAxis, double theta);
	Rotor();

	Vector3D rotate(Vector3D p);
	Rotor applyRotor(Rotor r);
	Rotor getInverse();

	bool operator!=(const Rotor& r);
	bool operator==(const Rotor& r);
};