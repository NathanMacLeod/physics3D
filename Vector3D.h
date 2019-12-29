#pragma once
#ifndef VECTOR3D_H
#define VECTOR3D_H

#include "Point.h"

class Vector3D {
public:
	double x;
	double y;
	double z;
	Vector3D(double x, double y, double z);
	Vector3D(Point3D p1, Point3D p2);
	Vector3D();
	Vector3D* getUnitVector(Vector3D* output) const;
	double getMagnitude() const;
	double getMagnitudeSquared() const;
	double dotProduct(const Vector3D vector) const;
	Vector3D* crossProduct(const Vector3D vector, Vector3D* output) const;
	Vector3D* add(const Vector3D vector, Vector3D* output) const;
	Vector3D* sub(const Vector3D vector, Vector3D* output) const;
	Vector3D* multiply(double scalar, Vector3D* output) const;
};

#endif