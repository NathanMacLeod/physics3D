#include "Vector3D.h"
#include <math.h>

Vector3D::Vector3D(double x, double y, double z) {
	this->x = x;
	this->y = y;
	this->z = z;
}

Vector3D::Vector3D() {
	x = 0;
	y = 0;
	z = 0;
}

Vector3D::Vector3D(Vector3D a, Vector3D b) {
	x = b.x - a.x;
	y = b.y - a.y;
	z = b.z - a.z;
}

bool Vector3D::operator==(const Vector3D& b) {
	return this->sub(b).getMagnitudeSquared() < 0.000000001;
}

bool Vector3D::operator!=(const Vector3D& b) {
	return this->sub(b).getMagnitudeSquared() >= 0.000000001;
}

double Vector3D::getMagnitudeSquared() const {
	return x * x + y * y + z * z;
}

double Vector3D::getMagnitude() const {
	return sqrt(x * x + y * y + z * z);
}

double Vector3D::dotProduct(const Vector3D vector) const {
	return  x * vector.x + y * vector.y + z * vector.z;
}

Vector3D Vector3D::getInverse() {
	return Vector3D(-x, -y, -z);
}

Vector3D Vector3D::getUnitVector() const {
	Vector3D output;
	double inverseMagnitude = 1 / getMagnitude();
	output.x = x * inverseMagnitude;
	output.y = y * inverseMagnitude;
	output.z = z * inverseMagnitude;
	return output;
}

Vector3D Vector3D::crossProduct(const Vector3D vector) const {
	Vector3D output;
	output.x = y * vector.z - z * vector.y;
	output.y = z * vector.x - x * vector.z;
	output.z = x * vector.y - y * vector.x;
	return output;
}

Vector3D Vector3D::add(const Vector3D vector) const {
	Vector3D output;
	output.x = x + vector.x;
	output.y = y + vector.y;
	output.z = z + vector.z;
	return output;
}
Vector3D Vector3D::sub(const Vector3D vector) const {
	Vector3D output;
	output.x = x - vector.x;
	output.y = y - vector.y;
	output.z = z - vector.z;
	return output;
}

Vector3D Vector3D::multiply(double scalar) const {
	Vector3D output;
	output.x = x * scalar;
	output.y = y * scalar;
	output.z = z * scalar;
	return output;
}

bool Vector3D::notZero() const {
	return !(x == 0 && y == 0 && z == 0);
}