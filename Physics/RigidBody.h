#pragma once
#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <vector>;
#include <inttypes.h>;
#include "ConvexHull.h";
#include "CollisionInfo.h";
#include "../Math/Rotor.h";

class RigidBody {

private:
	std::vector<Vector3D*> pointsToTransform;
	std::vector<Vector3D> pointsOG;
	std::vector<ConvexHull*> hulls;
	std::vector<uint16_t> testedList;
	std::vector<uint16_t> noColList;
	std::vector<CollisionInfo> collHistory;
	Vector3D centerOfMass;
	Vector3D velocity;
	Vector3D angularVelocity;
	Rotor orientation;

	Vector3D fixedDimensions;
	Vector3D dimCenter;

	double collisionRadius;
	double collisionRadiusSquared;
	double mass;
	double inverseMass;
	double friction;
	double restitution;
	Matrix33 inertiaTensor;
	Matrix33 tensorInverse;
	bool fixed;
	bool trackHistory;
	uint16_t ID;

	void copyPoints();
	void findBodyMassAndInertia(double density, bool useCustumCOM = false, Vector3D customCOM = Vector3D(0, 0, 0));
	double getInertiaOfAxis(const Vector3D axis);
	void findCollisionRadius();
	void findDimensions();
	Vector3D gyroAccel(double time);
	
public:

	RigidBody(const std::vector<ConvexHull*>& hulls, double density, double friction, double resistution, bool fixed, bool custumCOM = false, Vector3D com = Vector3D(0, 0, 0));
	RigidBody(const RigidBody& body);
	RigidBody();
	~RigidBody();
	Vector3D findVectorRelativeToBodyFrame(const Vector3D vector);
	std::vector<Vector3D>* getAllPoints();
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
	uint16_t getID();
	void addNoCol(uint16_t ID);
	bool onNoColList(uint16_t ID);
	void setToOrientation(Rotor orientation);
	void setVelocity(Vector3D vel);
	void setAngVelocity(Vector3D aVel);
	void setID(uint16_t ID);
	void translate(const Vector3D translation);
	void acclerateLineraly(const Vector3D changeInVelocity);
	void moveInTime(double time);
	bool alreadyTestedAgainst(uint16_t colliderID);
	void addTestedAgainst(uint16_t colliderID);
	void clearTestedList();
	void addToColHistory(uint16_t collider, double magnitude);
	std::vector<CollisionInfo>* getCollHistory();
	void clearCollHistory();
	bool trackingCollHistory();
	void setTrackCollHistory(bool b);
	bool getFixed();
	Rotor getOrientation();
	Vector3D getDimensions();
	Vector3D getDimCenter();
};

#endif