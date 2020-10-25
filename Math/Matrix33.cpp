#include "Matrix33.h"

Matrix33::Matrix33(double values[9]) {
	for (int i = 0; i < 9; i++) {
		elements[i / 3][i % 3] = values[i];
	}
}

Matrix33::Matrix33() {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			elements[i][j] = 0;
		}
	}
}

Matrix33 Matrix33::invert() const {
	Matrix33 out;
	out.elements[0][0] = elements[1][1] * elements[2][2] - elements[1][2] * elements[2][1];
	out.elements[0][1] = elements[0][2] * elements[2][1] - elements[0][1] * elements[2][2];
	out.elements[0][2] = elements[0][1] * elements[1][2] - elements[0][2] * elements[1][1];
	out.elements[1][0] = elements[1][2] * elements[2][0] - elements[1][0] * elements[2][2];
	out.elements[1][1] = elements[0][0] * elements[2][2] - elements[0][2] * elements[2][0];
	out.elements[1][2] = elements[0][2] * elements[0][3] - elements[0][0] * elements[1][2];
	out.elements[2][0] = elements[1][0] * elements[2][1] - elements[1][1] * elements[2][0];
	out.elements[2][1] = elements[0][1] * elements[2][0] - elements[0][0] * elements[2][1];
	out.elements[2][2] = elements[0][0] * elements[1][1] - elements[0][1] * elements[1][0];

	double s  = 1.0 / (elements[0][0] * (elements[1][1] * elements[2][2] - elements[1][2] * elements[2][1])
		- elements[0][1] * (elements[1][0] * elements[2][2] - elements[1][2] * elements[2][0])
		+ elements[0][2] * (elements[1][0] * elements[2][1] - elements[1][1] * elements[2][0]));
	for (int i = 0; i < 9; i++) {
		out.elements[i / 3][i % 3] *= s;
	}
	return out;
}

Vector3D Matrix33::operator*(const Vector3D& v) const {
	return Vector3D(elements[0][0] * v.x + elements[0][1] * v.y + elements[0][2] * v.z,
		elements[1][0] * v.x + elements[1][1] * v.y + elements[1][2] * v.z,
		elements[2][0] * v.x + elements[2][1] * v.y + elements[2][2] * v.z);
}

Matrix33 Matrix33::operator*(const Matrix33& m) const {
	Matrix33 out;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				out.elements[i][j] += elements[i][k] * m.elements[k][j];
			}
		}
	}

	return out;
}

Matrix33 Matrix33::operator*(double scalar) const {
	Matrix33 out;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out.elements[i][j] = elements[i][j] * scalar;
		}
	}

	return out;
}

Matrix33 Matrix33::operator+(const Matrix33& m) const {
	Matrix33 out;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out.elements[i][j] = elements[i][j] + m.elements[i][j];
		}
	}

	return out;
}

Matrix33 Matrix33::skew(Vector3D v) {
	Matrix33 out;
	
	out.elements[0][1] = -v.z;
	out.elements[1][0] = v.z;
	out.elements[0][2] = v.y;
	out.elements[2][0] = -v.y;
	out.elements[1][2] = -v.x;
	out.elements[2][1] = v.x;

	return out;
}

Vector3D Matrix33::newtonSolve(Vector3D v, Vector3D fV, const Matrix33& jacobian) {
	return v.sub(jacobian.invert() * fV);
}