#include "Vector3D.h"
#include <math.h>

Vector3D::Vector3D(double x, double y, double z) {
	this->x = x;
	this->y = y;
	this->z = z;
}

Vector3D::Vector3D(const Point3D p1, const Point3D p2) {
	this->x = p2.x - p1.x;
	this->y = p2.y - p1.y;
	this->z = p2.z - p1.z;
}

Vector3D::Vector3D() {
	x = 0;
	y = 0;
	z = 0;
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

Vector3D* Vector3D::getUnitVector(Vector3D* output) const {
	double inverseMagnitude = 1 / getMagnitude();
	output->x = x * inverseMagnitude;
	output->y = y * inverseMagnitude;
	output->z = z * inverseMagnitude;
	return output;
}

Vector3D* Vector3D::crossProduct(const Vector3D vector, Vector3D* output) const {
	double newX = y * vector.z - z * vector.y;
	double newY = z * vector.x - x * vector.z;
	double newZ = x * vector.y - y * vector.x;
	output->x = newX;
	output->y = newY;
	output->z = newZ;
	return output;
}

Vector3D* Vector3D::add(const Vector3D vector, Vector3D* output) const {
	output->x = x + vector.x;
	output->y = y + vector.y;
	output->z = z + vector.z;
	return output;
}
Vector3D* Vector3D::sub(const Vector3D vector, Vector3D* output) const {
	output->x = x - vector.x;
	output->y = y - vector.y;
	output->z = z - vector.z;
	return output;
}

Vector3D* Vector3D::multiply(double scalar, Vector3D* output) const {
	output->x = x * scalar;
	output->y = y * scalar;
	output->z = z * scalar;
	return output;
}