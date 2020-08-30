#pragma once
#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <vector>;
#include "ConvexHull.h"

class RigidBody {

private:
	std::vector<Vector3D*> pointsToTransform;
	std::vector<ConvexHull*> hulls;
	Vector3D centerOfMass;
	Vector3D velocity;
	Vector3D angularVelocity;
	Vector3D orientationPoint1;
	Vector3D orientationPoint2;
	double collisionRadius;
	double collisionRadiusSquared;
	double mass;
	double inverseMass;
	double friction;
	double restitution;
	double* inertiaTensor;
	bool fixed;

	void findBodyMassAndInertia(double density);
	void findCollisionRadius();
	Vector3D findVectorRelativeToBodyFrame(const Vector3D vector);
public:

	RigidBody(const std::vector<ConvexHull*>& hulls, double density, double friction, double resistution, bool fixed);
	~RigidBody();
	bool bodiesInCollisionRange(RigidBody* body);
	double getCollisionRadius() const;
	double getCollisionRadiusSquared() const;
	Vector3D getAngularVelocity();
	std::vector<ConvexHull*>* getHulls();
	Vector3D getCenterOfMass();
	Vector3D getVelocityOfPointDueToAngularVelocity(const Vector3D point) const;
	Vector3D getVelocityOfPoint(const Vector3D point) const;
	Vector3D getVelocity();
	bool verifyCollisionPointNotExiting(RigidBody* body, const Vector3D normalVector, const Vector3D point);
	void applyImpulseAtPosition(const Vector3D impulse, const Vector3D position);
	double findInverseInertiaOfAxis(const Vector3D axis);
	double getRadialDistanceOfPoint(const Vector3D point);
	double getMass() const;
	double getInverseMass() const;
	double getFriction() const;
	double getRestitution() const;
	void translate(const Vector3D translation);
	void acclerateLineraly(const Vector3D changeInVelocity);
	void moveInTime(double time);
	bool getFixed();
};

#endif