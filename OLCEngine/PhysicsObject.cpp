#include "PhysicsObject.h"

PhysicsObject::PhysicsObject(PolyModel* model, RigidBody* body) {
	this->model = *model;
	this->body = body;
}

PolyModel* PhysicsObject::getModelToDraw() {
	if (model.getOrientation() != body->getOrientation()
		|| model.getPos() != body->getCenterOfMass()) {
		
		model.setPosAndOrientation(body->getOrientation(), body->getCenterOfMass());
	}
	return &model;
}