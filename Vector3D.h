#pragma once
#ifndef VECTOR3D_H
#define VECTOR3D_H

class Vector3D {
public:
	double x;
	double y;
	double z;
	Vector3D(double x, double y, double z);
	Vector3D();
	Vector3D(Vector3D a, Vector3D b);
	Vector3D getInverse();
	Vector3D getUnitVector() const;
	double getMagnitude() const;
	double getMagnitudeSquared() const;
	double dotProduct(const Vector3D vector) const;
	Vector3D crossProduct(const Vector3D vector) const;
	Vector3D add(const Vector3D vector) const;
	Vector3D sub(const Vector3D vector) const;
	Vector3D multiply(double scalar) const;
	bool notZero() const;
};

#endif