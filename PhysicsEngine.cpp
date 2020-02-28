#include "PhysicsEngine.h"
#include <iostream>


const double GRAV_DEFAULT = 20;
const double TIMESTEP_DEFAULT = 0.015;

PhysicsEngine::PhysicsEngine() {
	gravity.y = GRAV_DEFAULT;
	timestep = TIMESTEP_DEFAULT;
}

PhysicsEngine::PhysicsEngine(double timestep, const Vector3D& gravity) {
	this->gravity = gravity;
	this->timestep = timestep;
	reductionVel = 2 * timestep * timestep * gravity.getMagnitudeSquared();
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

void PhysicsEngine::pushBodiesApart(RigidBody* collider, RigidBody* collidee, const Vector3D nV, double colDepth) {
	double dCldr = colDepth / (1 + collider->getMass() * collidee->getInverseMass());
	double dCldee = colDepth - dCldr;
	Vector3D cldeTransform;
	nV.multiply(-dCldee, &cldeTransform);
	Vector3D cldrTransform;
	nV.multiply(dCldr, &cldrTransform);
	collider->translate(cldrTransform);
	collidee->translate(cldeTransform);
}

void PhysicsEngine::resolveImpulses(RigidBody* collider, RigidBody* collidee, const Vector3D nV, Point3D* colPoint, double restitutionFactor) {
	double restitution = collidee->getRestitution() * restitutionFactor;
	//Vector3D nV(*(colSurface->getPoints()->at(0)), *(colSurface->getNormalVectorPoint()));
	//if (nV.y >= 0 && collidee->getInverseMass() == 0)
	//	std::cout << colSurface->getPoints()->at(0)->y << "\n";
	//push bodies apart

	//Vector3D p1ToP(*(colSurface->getPoints()->at(0)), *colPoint);
	//double distToPlane = abs(nV.dotProduct(p1ToP));

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

	if (velRel.getMagnitudeSquared() < reductionVel) {
		//restitution = 0;
	}

	double numerator = -(restitution + 1) * nV.dotProduct(velRel);
	double denomenator = (collider->getInverseMass() + collidee->getInverseMass() + rCldrXn.getMagnitudeSquared() * invICldr + rCldeXn.getMagnitudeSquared() * invIClde);

	double impulseMagnitude = abs(numerator / denomenator);
	Vector3D impulse;
	nV.multiply(impulseMagnitude, &impulse);

	Vector3D velParralel;
	nV.multiply(nV.dotProduct(velRel), &velParralel);
	Vector3D k;
	Vector3D frictionImpulse;
	velRel.sub(velParralel, &k);
	if (k.notZero()) {
		k.getUnitVector(&k);
		rCldr.crossProduct(k, &jRotAxis);
		numerator = k.dotProduct(velRel);
		Vector3D rCldrXk;
		rCldr.crossProduct(k, &rCldrXk);
		Vector3D rCldeXk;
		rCldr.crossProduct(k, &rCldeXk);
		denomenator = (collider->getInverseMass() + collidee->getInverseMass() + rCldrXk.getMagnitudeSquared() * invICldr + rCldeXk.getMagnitudeSquared() * invIClde);
		double frictionImpMag = abs(numerator / denomenator);
		if (frictionImpMag > impulseMagnitude * collidee->getFriction()) {
			frictionImpMag = impulseMagnitude * collidee->getFriction();
		}
		k.multiply(frictionImpMag, &frictionImpulse);
	}
	/*Vector3D up(0, -1, 0);

	Vector3D pVel;
	collider->getVelocityOfPoint(*colPoint, &pVel);
	Vector3D pRot;
	Vector3D comP(*(collider->getCenterOfMass()), *colPoint);
	comP.getUnitVector(&comP);
	comP.crossProduct(pVel, &pRot);
	std::cout << "pRotVel: " << pRot.dotProduct(up) << "\n";
	std::cout << "curr veloc: " << collider->getAngularVelocity()->dotProduct(up) << "\n";*/

	collider->applyImpulseAtPosition(impulse, *colPoint);

	//std::cout << "veloc after: " << collider->getAngularVelocity()->dotProduct(up) << "\n\n";
	
	impulse.multiply(-1, &impulse);
	collidee->applyImpulseAtPosition(impulse, *colPoint);

	if (frictionImpulse.notZero()) {
		
		/*std::cout << "curr veloc: " << collider->getAngularVelocity()->dotProduct(up) << "\n";
		Vector3D pVel;
		collider->getVelocityOfPoint(*colPoint, &pVel);
		Vector3D pRot;
		Vector3D comP(*(collider->getCenterOfMass()), *colPoint);
		comP.getUnitVector(&comP);
		comP.crossProduct(pVel, &pRot);
		std::cout << "pRotVel: " << pRot.dotProduct(up) << "\n";*/

		double angVelBefore = collider->getAngularVelocity()->getMagnitudeSquared();
		collider->applyImpulseAtPosition(frictionImpulse, *colPoint);

		
		
		//Vector3D rotAxis;
		//comP.crossProduct(frictionImpulse, &rotAxis);
		
		//std::cout << "dot product: " << rotAxis.dotProduct(up) << "\n";
		//std::cout << "veloc after: " << collider->getAngularVelocity()->dotProduct(up) << "\n";

		//std::cout << "vel rel: " << k.x << " " << k.y << " " << k.z << "\n";
		//Vector3D frictionUnit;
		//frictionImpulse.getUnitVector(&frictionUnit);
		//std::cout << "impulse: " << frictionUnit.x << " " << frictionUnit.y << " " << frictionUnit.z << "\n";

		//std::cout << "/t/t-------------------\n";

		frictionImpulse.multiply(-1, &frictionImpulse);
		collidee->applyImpulseAtPosition(frictionImpulse, *colPoint);
		double angVelAfter = collider->getAngularVelocity()->getMagnitudeSquared();
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

			std::vector<RigidBody::ColPointInfo*> b1ColInfo;
			std::vector<RigidBody::ColPointInfo*> b2ColInfo;

			bool collisionsOccuring = true;

			while (collisionsOccuring) {
				body1->findCollisionInformationAsCollider(&b1ColInfo, *body2);
				body2->findCollisionInformationAsCollider(&b2ColInfo, *body1);

				std::vector<RigidBody::ColPointInfo*>* colInfo = nullptr;
				RigidBody* collider = nullptr;
				RigidBody* collidee = nullptr;

				bool b1Collider = true;
				if (b1ColInfo.size() == 0 && b2ColInfo.size() == 0) {
					collisionsOccuring = false;
				}
				else if (b1ColInfo.size() == 0) {
					b1Collider = false;
				}
				else if (b2ColInfo.size() != 0 && ((b1ColInfo.at(0)->edgeCollision && !b2ColInfo.at(0)->edgeCollision) ||
					b1ColInfo.at(0)->penDepth < b2ColInfo.at(0)->edgeCollision)) {
					b1Collider = false;
				}

				if (collisionsOccuring) {

					if (b1Collider) {
						colInfo = &b1ColInfo;
						collider = body1;
						collidee = body2;
					}
					else {
						colInfo = &b2ColInfo;
						collider = body2;
						collidee = body1;
					}

					int maxCollisions = 4;
					double restitutionMultiplier = 1;
					double restitutionReductionFactor = 0.6;
					int colCount = 0;

					int k = colInfo->size();

					if (colInfo->size() == 1) {
						resolveImpulses(collider, collidee, *colInfo->at(0)->colNormVector, colInfo->at(0)->point, restitutionMultiplier);
						pushBodiesApart(collider, collidee, *colInfo->at(0)->colNormVector, colInfo->at(0)->penDepth);
					}
					else {
						while (colCount < maxCollisions) {
							bool colTriggered = false;
							for (int i = 1; colCount < maxCollisions && i < colInfo->size(); i++) {
								if (collider->verifyCollisionPointNotExiting(*collidee, *colInfo->at(i)->colNormVector, *colInfo->at(i)->point)) {
									colTriggered = true;
									colCount++;
									resolveImpulses(collider, collidee, *colInfo->at(i)->colNormVector, colInfo->at(i)->point, restitutionMultiplier);
									restitutionMultiplier *= restitutionReductionFactor;
								}
							}
							if (!colTriggered)
								break;
						}
						//resolveImpulses(collider, collidee, *colInfo->at(0)->colNormVector, colInfo->at(0)->point, restitutionMultiplier);
						pushBodiesApart(collider, collidee, *colInfo->at(0)->colNormVector, colInfo->at(0)->penDepth);
					}

				}

				for (int i = 0; i < b1ColInfo.size(); i++) {
					if (!b1ColInfo.at(i)->pExiting)
						delete b1ColInfo.at(i);
				}
				for (int i = 0; i < b2ColInfo.size(); i++) {
					if (!b2ColInfo.at(i)->pExiting)
						delete b2ColInfo.at(i);
				}
				b1ColInfo.clear();
				b2ColInfo.clear();
			}

			/*
			//resolve up to n collisions between the bodies.
			double restitutionMultiplier = 1;
			double restitutionReductionFactor = 0.6;
			for (int i = 0; i < maxCollisions; i++) {
				Point3D* b1ColPoint = nullptr;
				RigidSurface* b1ColSurface = nullptr;
				Vector3D* b1ColNV = nullptr;
				bool b1EdgeCollision = false;
				double b1ColDepth = body1->findCollisionInformationAsCollider(&b1ColPoint, &b1ColNV, *body2, &b1EdgeCollision);
				Point3D* b2ColPoint = nullptr;
				Vector3D* b2ColNV = nullptr;
				bool b2EdgeCollision = false;
				double b2ColDepth = body2->findCollisionInformationAsCollider(&b2ColPoint, &b2ColNV, *body1, &b2EdgeCollision);
				RigidBody* collider = nullptr;
				RigidBody* collidee = nullptr;
				Point3D* colPoint = nullptr;
				Vector3D* colNV = nullptr;
				bool edgeCollision;
				double colDepth = 0;
				if (b1ColDepth > b2ColDepth) {
					collider = body1;
					collidee = body2;
					colPoint = b1ColPoint;
					colNV = b1ColNV;
					colDepth = b1ColDepth;
					edgeCollision = b1EdgeCollision;
				}
				else {
					if (b2ColDepth == -1)
						break; //no collision
					collider = body2;
					collidee = body1;
					colPoint = b2ColPoint;
					colNV = b2ColNV;
					colDepth = b2ColDepth;
					edgeCollision = b2EdgeCollision;
				}
				resolveImpulses(collider, collidee, *colNV, colPoint, restitutionMultiplier, colDepth);
				restitutionMultiplier *= restitutionReductionFactor;
				if (edgeCollision) //edge collisions generate new point, needs to be deleted
					delete colPoint;
			}
			*/
		}
	}
}
