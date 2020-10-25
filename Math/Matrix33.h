#pragma once
#include "Vector3D.h"

class Matrix33 {
public:
	Matrix33(double values[9]);
	Matrix33();

	Matrix33 invert() const;
	Vector3D operator*(const Vector3D& v) const;
	Matrix33 operator*(const Matrix33& m) const;
	Matrix33 operator*(const double scalar) const;
	Matrix33 operator+(const Matrix33& m) const;

	static Matrix33 skew(Vector3D v);
	static Vector3D newtonSolve(Vector3D v, Vector3D fV, const Matrix33& jacobian);

	double elements[3][3];
};