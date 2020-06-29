#pragma once
#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <vector>;
#include "RigidSurface.h"

class RigidBody {

public:
	class ColPointInfo {
	public:
		Point3D point;
		Vector3D colNormVector;
		double penDepth;

		ColPointInfo(double x, double y, double z, Vector3D* colNormVector, double penDepth);
	};

	class Edge {
	public:
		Point3D* p1;
		Point3D* p2;
		double inverseMagnitude;

		Edge(Point3D* p1, Point3D* p2);
	};

private:
	std::vector<RigidSurface*> surfacesReferenceCopy;
	std::vector<RigidSurface*> surfaces;
	std::vector<Point3D*> pointsToTransform;
	std::vector<Point3D*> colPoints;
	std::vector<Edge*> colEdges;
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

	void findBodyMassAndInertia(double density);
	void findCollisionRadius();
	void findColPointsEdges();
	void createReferenceCopies();
	bool getPointInsideBody(const Point3D point, const std::vector<Point3D*> vectorPositions, const std::vector<Vector3D*> normalVectors);
	Vector3D findVectorRelativeToBodyFrame(const Vector3D vector);
public:

	double* inertiaTensor; //remove
	RigidBody(const std::vector<RigidSurface*>& surfaces, double density, double friction, double resistution, bool fixed); //not done
	void recalibrateFromReference();
	bool bodiesInCollisionRange(RigidBody& body);
	double getCollisionRadius() const;
	double getCollisionRadiusSquared() const;
	Vector3D getAngularVelocity();
	void findCollisionInformationAsCollider(std::vector<ColPointInfo*>* colOutputs, RigidBody& body, double timestep);
	bool getPointInsideBody(const Point3D point);
	std::vector<RigidSurface*>* getSurfaces();
	Point3D* getCenterOfMass();
	Vector3D getVelocityOfPointDueToAngularVelocity(const Point3D point) const;
	Vector3D getVelocityOfPoint(const Point3D point) const;
	Vector3D getVelocity();
	bool verifyCollisionPointNotExiting(const RigidBody body, const Vector3D normalVector, const Point3D point);
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