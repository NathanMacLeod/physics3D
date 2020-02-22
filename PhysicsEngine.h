#pragma once
#include "RigidBody.h"

class PhysicsEngine {
	Vector3D gravity;
	double timestep;
	double reductionVel;
	std::vector<RigidBody*> rigidBodies;
	void resolveImpulses(RigidBody* collider, RigidBody* collidee, const Vector3D nV, Point3D* colPoint, double restitutionFactor, double collisionDepth);
public:
	PhysicsEngine(double timestep, const Vector3D& gravity);
	PhysicsEngine();
	double getTimestep();
	void iterateEngineTimestep();
	void addRigidBody(RigidBody* body);
	void removeRigidBody(RigidBody* body);
};