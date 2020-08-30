#pragma once
#include "RigidBody.h"
#include "OctreeNode.h"

class PhysicsEngine {
	Vector3D gravity;
	double timestep;
	double reductionVel;
	double staticVel;
	//std::vector<RigidBody*> rigidBodies;
	void pushBodiesApart(RigidBody* collider, RigidBody* collidee, const Vector3D nV, double colDepth);
	void resolveImpulses(RigidBody* collider, RigidBody* collidee, const Vector3D nV, Point3D colPoint, const std::vector<ConvexHull::ColPointInfo> supPoints, double restitution);
public:
	std::vector<RigidBody*> rigidBodies; //make private again
	OctreeNode root;
	PhysicsEngine(double timestep, const Vector3D& gravity);
	PhysicsEngine();
	double getTimestep();
	void iterateEngineTimestep();
	void addRigidBody(RigidBody* body);
	bool removeRigidBody(RigidBody* body);
};