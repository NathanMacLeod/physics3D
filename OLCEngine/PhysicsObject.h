#pragma once
#include "PolyModel.h"
#include "../Physics/RigidBody.h"
#include "../Physics/PhysicsEngine.h";

class PhysicsObject {
private:
	PolyModel model;
	RigidBody* body;  //rigid body deleted by PhysicsEngine
public:
	PhysicsObject(PolyModel* model, RigidBody* body);
	PolyModel* getModelToDraw();
};