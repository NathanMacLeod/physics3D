#pragma once
#include "RigidBody.h"

class PhysicsEngine {
	Vector3D gravity;
	double timestep;
	std::vector<RigidBody*> rigidBodies;
public:
	PhysicsEngine(double timestep, const Vector3D& gravity);
	PhysicsEngine();
	double getTimestep();
	void iterateEngineTimestep();
	void addRigidBody(RigidBody* body);
	void removeRigidBody(RigidBody* body);
};