#pragma once
#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <vector>;
#include "RigidSurface.h"

class RigidBody {
	std::vector<RigidSurface*> surfacesReferenceCopy;
	std::vector<RigidSurface*> surfaces;
	std::vector<Point3D*> pointsToTransform;
	Point3D centerOfMass;
	Vector3D velocity;
	Vector3D angularVelocity;
	Point3D orientationPoint1;
	Point3D orientationPoint2;
	double collisionRadius;
	double collisionRadiusSquared;
	double mass;
	double inverseMass;
	double friction;
	double restitution;
	//double* inertiaTensor
	bool fixed;
	
	void findBodyMassAndInertia(double particleMass, double particleDensity);
	void findCollisionRadius();
	void createReferenceCopies();
	bool getPointInsideBody(const Point3D point, const std::vector<Point3D*> vectorPositions, const std::vector<Vector3D*> normalVectors);
	Vector3D* findVectorRelativeToBodyFrame(const Vector3D vector, Vector3D* output);
public:
	double* inertiaTensor; //remove
	RigidBody(const std::vector<RigidSurface*>& surfaces, double density, double friction, double resistution, bool fixed); //not done
	void recalibrateFromReference();
	bool bodiesInCollisionRange(RigidBody& body);
	double getCollisionRadius() const;
	double getCollisionRadiusSquared() const;
	double findCollisionInformationAsCollider(Point3D** collidingPoint, RigidSurface** collidngSurface, RigidBody& body);
	bool getPointInsideBody(const Point3D point);
	std::vector<RigidSurface*>* getSurfaces();
	Point3D* getCenterOfMass();
	Vector3D* getVelocityOfPointDueToAngularVelocity(const Point3D point, Vector3D* output) const;
	Vector3D* getVelocityOfPoint(const Point3D point, Vector3D* output) const;
	void applyImpulseAtPosition(const Vector3D impulse, const Point3D position);
	double findInverseInertiaOfAxis(const Vector3D axis);
	double getRadialDistanceOfPoint(const Point3D point);
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