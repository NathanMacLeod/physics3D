#include "PhysicsEngine.h"
#include <iostream>


const double GRAV_DEFAULT = 20;
const double TIMESTEP_DEFAULT = 0.01;

PhysicsEngine::PhysicsEngine() {
	gravity.y = GRAV_DEFAULT;
	timestep = TIMESTEP_DEFAULT;
}

PhysicsEngine::PhysicsEngine(double timestep, const Vector3D& gravity) {
	this->gravity = gravity;
	this->timestep = timestep;
}

void PhysicsEngine::addRigidBody(RigidBody* body) {
	rigidBodies.push_back(body);
}

void PhysicsEngine::removeRigidBody(RigidBody* body) {
	//no
}

double PhysicsEngine::getTimestep() {
	return timestep;
}

void PhysicsEngine::resolveImpulses(RigidBody* collider, RigidBody* collidee, RigidSurface* colSurface, Point3D* colPoint, double restitutionFactor, double colDepth) {
	double restitution = collidee->getRestitution() * restitutionFactor;
	Vector3D nV(*(colSurface->getPoints()->at(0)), *(colSurface->getNormalVectorPoint()));
	//push bodies apart

	Vector3D p1ToP(*(colSurface->getPoints()->at(0)), *colPoint);
	double distToPlane = abs(nV.dotProduct(p1ToP));
	double dCldr = distToPlane / (1 + collider->getMass() * collidee->getInverseMass());
	double dCldee = distToPlane - dCldr;
	Vector3D cldeTransform;
	nV.multiply(-dCldee, &cldeTransform);
	Vector3D cldrTransform;
	nV.multiply(dCldr, &cldrTransform);
	collider->translate(cldrTransform);
	collidee->translate(cldeTransform);

	Vector3D vCldrP0;
	collider->getVelocityOfPoint(*colPoint, &vCldrP0);
	Vector3D vCldeP0;
	collidee->getVelocityOfPoint(*colPoint, &vCldeP0);
	Vector3D velRel;
	vCldeP0.sub(vCldrP0, &velRel);
	Vector3D rClde(*(collidee->getCenterOfMass()), *colPoint);
	Vector3D rCldr(*(collider->getCenterOfMass()), *colPoint);
	Vector3D jRotAxis;
	rCldr.crossProduct(nV, &jRotAxis);
	double invIClde = collidee->findInverseInertiaOfAxis(jRotAxis);
	double invICldr = collider->findInverseInertiaOfAxis(jRotAxis);
	Vector3D rCldeXn;
	rClde.crossProduct(nV, &rCldeXn);
	Vector3D rCldrXn;
	rCldr.crossProduct(nV, &rCldrXn);

	double numerator = -(restitution + 1) * nV.dotProduct(velRel);
	double denomenator = (collider->getInverseMass() + collidee->getInverseMass() + rCldrXn.getMagnitudeSquared() * invICldr + rCldeXn.getMagnitudeSquared() * invIClde);

	double impulseMagnitude = numerator / denomenator;
	Vector3D impulse;
	nV.multiply(impulseMagnitude, &impulse);

	Vector3D velParralel;
	nV.multiply(nV.dotProduct(velRel), &velParralel);
	Vector3D k;
	Vector3D frictionImpulse;
	velRel.sub(velParralel, &k);
	if (k.getMagnitudeSquared() != 0) {
		k.getUnitVector(&k);
		rCldr.crossProduct(k, &jRotAxis);
		numerator = k.dotProduct(velRel);
		Vector3D rCldrXk;
		rCldr.crossProduct(k, &rCldrXk);
		Vector3D rCldeXk;
		rCldr.crossProduct(k, &rCldeXk);
		denomenator = (collider->getInverseMass() + collidee->getInverseMass() + rCldrXk.getMagnitudeSquared() * invICldr + rCldeXk.getMagnitudeSquared() * invIClde);
		double frictionImpMag = numerator / denomenator;
		if (abs(frictionImpMag) > abs(impulseMagnitude)* collidee->getFriction())
			frictionImpMag = impulseMagnitude * collidee->getFriction();
		k.multiply(-frictionImpMag, &frictionImpulse);
	}

	collidee->applyImpulseAtPosition(impulse, *colPoint);
	impulse.multiply(-1, &impulse);
	collider->applyImpulseAtPosition(impulse, *colPoint);

	if (frictionImpulse.getMagnitudeSquared() != 0) {
		collider->applyImpulseAtPosition(frictionImpulse, *colPoint);
		frictionImpulse.multiply(-1, &frictionImpulse);
		collidee->applyImpulseAtPosition(frictionImpulse, *colPoint);
	}
}

void PhysicsEngine::iterateEngineTimestep() {
	Vector3D gravityAcceleration(gravity.x * timestep, gravity.y * timestep, gravity.z * timestep);
	for (RigidBody* body : rigidBodies) {
		body->acclerateLineraly(gravityAcceleration);
		body->moveInTime(timestep);
	}
	for (RigidBody* body1 : rigidBodies) {
		for (RigidBody* body2 : rigidBodies) {
			if (body1 == body2)
				continue;
			if (!body1->bodiesInCollisionRange(*body2))
				continue;
			//resolve up to n collisions between the bodies.
			double restitutionMultiplier = 1;
			double restitutionReductionFactor = 0.6;
			double maxCollisions = 5;
			for (int i = 0; i < maxCollisions; i++) {
				Point3D* b1ColPoint = nullptr;
				RigidSurface* b1ColSurface = nullptr;
				double b1ColDepth = body1->findCollisionInformationAsCollider(&b1ColPoint, &b1ColSurface, *body2);
				Point3D* b2ColPoint = nullptr;
				RigidSurface* b2ColSurface = nullptr;
				double b2ColDepth = body2->findCollisionInformationAsCollider(&b2ColPoint, &b2ColSurface, *body1);
				RigidBody* collider = nullptr;
				RigidBody* collidee = nullptr;
				Point3D* colPoint = nullptr;
				RigidSurface* colSurface = nullptr;
				double colDepth = 0;
				if (b1ColDepth > b2ColDepth) {
					collider = body1;
					collidee = body2;
					colPoint = b1ColPoint;
					colSurface = b1ColSurface;
					colDepth = b1ColDepth;
				}
				else {
					if (b2ColDepth == -1)
						break; //no collision
					collider = body2;
					collidee = body1;
					colPoint = b2ColPoint;
					colSurface = b2ColSurface;
					colDepth = b2ColDepth;
				}
				resolveImpulses(collider, collidee, colSurface, colPoint, pow(restitutionReductionFactor, i), colDepth);
			}
		}
	}
}