#pragma once
#include "RigidBody.h"

class PhysicsEngine {
	Vector3D gravity;
	double timestep;
	std::vector<RigidBody*> rigidBodies;
	void resolveImpulses(RigidBody* collider, RigidBody* collidee, RigidSurface* colSurface, Point3D* colPoint, double restitutionFactor, double collisionDepth);
public:
	PhysicsEngine(double timestep, const Vector3D& gravity);
	PhysicsEngine();
	double getTimestep();
	void iterateEngineTimestep();
	void addRigidBody(RigidBody* body);
	void removeRigidBody(RigidBody* body);
};