#include "PhysicsEngine.h"
#include <iostream>


const double GRAV_DEFAULT = 20;
const double TIMESTEP_DEFAULT = 0.03;

PhysicsEngine::PhysicsEngine() {
	gravity.y = GRAV_DEFAULT;
	timestep = TIMESTEP_DEFAULT;
	reductionVel = 35 * timestep * gravity.getMagnitude();
	staticVel = 0.001;
}

PhysicsEngine::PhysicsEngine(double timestep, const Vector3D& gravity) {
	this->gravity = gravity;
	this->timestep = timestep;
	reductionVel = 2 * timestep * timestep * gravity.getMagnitudeSquared();
	staticVel = 0.1;
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
	double pushDist = colDepth + 0.1;
	double dCldr = pushDist / (1 + collider->getMass() * collidee->getInverseMass());
	double dCldee = pushDist - dCldr;
	Vector3D cldeTransform;
	nV.multiply(-dCldee, &cldeTransform);
	Vector3D cldrTransform;
	nV.multiply(dCldr, &cldrTransform);
	collider->translate(cldrTransform);
	collidee->translate(cldeTransform);
}

void PhysicsEngine::resolveImpulses(RigidBody* collider, RigidBody* collidee, const Vector3D nV, Point3D* colPoint, double restitution, bool contactCollision) {
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
	Vector3D jRotAxisCldr;
	Vector3D jRotAxisClde;
	rCldr.crossProduct(nV, &jRotAxisCldr);
	rClde.crossProduct(nV, &jRotAxisClde);
	double invIClde = collidee->findInverseInertiaOfAxis(jRotAxisClde);
	double invICldr = collider->findInverseInertiaOfAxis(jRotAxisCldr);
	double normVel = nV.dotProduct(velRel);
	Vector3D rCldeXn;
	rClde.crossProduct(nV, &rCldeXn);
	Vector3D rCldrXn;
	rCldr.crossProduct(nV, &rCldrXn);
	static double lowestVel = 10;
	if (normVel < lowestVel) {
		lowestVel = normVel;
	}


	double numerator = -(restitution + 1) * normVel;
	double denomenator = (collider->getInverseMass() + collidee->getInverseMass() + rCldrXn.getMagnitudeSquared() * invICldr + rCldeXn.getMagnitudeSquared() * invIClde);

	double impulseMagnitude = abs(numerator / denomenator);
	Vector3D impulse;
	nV.multiply(impulseMagnitude, &impulse);

	Vector3D velParralel;
	nV.multiply(normVel, &velParralel);
	Vector3D k;
	Vector3D frictionImpulse;
	velRel.sub(velParralel, &k);
	if (k.notZero()) {
		k.getUnitVector(&k);
		rCldr.crossProduct(k, &jRotAxisCldr);
		rClde.crossProduct(k, &jRotAxisClde);
		invIClde = collidee->findInverseInertiaOfAxis(jRotAxisClde);
		invICldr = collider->findInverseInertiaOfAxis(jRotAxisCldr);
		numerator = k.dotProduct(velRel);
		Vector3D rCldrXk;
		rCldr.crossProduct(k, &rCldrXk);
		Vector3D rCldeXk;
		rCldr.crossProduct(k, &rCldeXk);
		denomenator = (collider->getInverseMass() + collidee->getInverseMass() + rCldrXk.getMagnitudeSquared() * invICldr + rCldeXk.getMagnitudeSquared() * invIClde);
		double frictionImpMag = abs(numerator / denomenator);
		//std::cout << contactCollision << ", " << (velParralel.getMagnitudeSquared() < staticVel) << "\n";
		double maxFriction = (contactCollision && velParralel.getMagnitudeSquared() < staticVel)? 30 * impulseMagnitude : impulseMagnitude * collidee->getFriction();
		if (frictionImpMag > maxFriction) {
			frictionImpMag = maxFriction;
		}
		k.multiply(frictionImpMag, &frictionImpulse);
	}
	//Vector3D up(0, -1, 0);

	/*Vector3D pVel;
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
		
		//std::cout << "curr veloc: " << collider->getAngularVelocity()->dotProduct(up) << "\n";
		/*Vector3D pVel;
		collider->getVelocityOfPoint(*colPoint, &pVel);
		Vector3D pRot;
		Vector3D comP(*(collider->getCenterOfMass()), *colPoint);
		comP.getUnitVector(&comP);
		comP.crossProduct(pVel, &pRot);
		std::cout << "pRotVel: " << pRot.dotProduct(up) << "\n";*/

		//double angVelBefore = collider->getAngularVelocity()->getMagnitudeSquared();
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
	for (RigidBody* body : rigidBodies) {
		//std::cout << body->getVelocity()->x << ", " << body->getVelocity()->y << ", " << body->getVelocity()->z << "\n";
		//std::cout << body->getAngularVelocity()->getMagnitude() << " " << body->getVelocity()->getMagnitude() << "\n";
	}
	Vector3D gravityAcceleration(gravity.x * timestep, gravity.y * timestep, gravity.z * timestep);
	for (RigidBody* body : rigidBodies) {
		body->acclerateLineraly(gravityAcceleration);
		body->moveInTime(timestep);
	}
	for (int i = 0; i < rigidBodies.size(); i++) {
		for (int j = i + 1; j < rigidBodies.size(); j++) {
			RigidBody* body1 = rigidBodies.at(i);
			RigidBody* body2 = rigidBodies.at(j);
			if (!body1->bodiesInCollisionRange(*body2))
				continue;

			std::vector<RigidBody::ColPointInfo*> b1ColInfo;
			std::vector<RigidBody::ColPointInfo*> b2ColInfo;

			bool collisionsOccuring = true;

			int count = 0;
			int max = 3;
			while (collisionsOccuring && count < max) {
				count++;
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
				else if (b2ColInfo.size() == 0) {
					b1Collider = true;
				}
				else if (b1ColInfo.at(0)->edgeCollision && !b2ColInfo.at(0)->edgeCollision) {
					b1Collider = false;
				}
				else if (!(!b1ColInfo.at(0)->edgeCollision && b2ColInfo.at(0)->edgeCollision) &&
					(b1ColInfo.at(0)->penDepth < b2ColInfo.at(0)->edgeCollision)) {
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

					int maxCollisions = 16 * colInfo->size();
					//double restitutionMultiplier = 1;
					//double restitutionReductionFactor = 0.5;
					int colCount = 0;
					bool contactCollision = false;
					if (colInfo->size() != 1) {
						Vector3D vCldrP0;
						collider->getVelocityOfPoint(*colInfo->at(0)->point, &vCldrP0);
						Vector3D vCldeP0;
						collidee->getVelocityOfPoint(*colInfo->at(0)->point, &vCldeP0);
						Vector3D velRel;
						vCldeP0.sub(vCldrP0, &velRel);
						double normVel = colInfo->at(0)->colNormVector->dotProduct(velRel);
						contactCollision = normVel < reductionVel;
					}
					bool dampenCollision = contactCollision;
					//static bool iterateForwards = true;

					int k = colInfo->size();

					if (colInfo->size() == 1) {
						resolveImpulses(collider, collidee, *colInfo->at(0)->colNormVector, colInfo->at(0)->point, (collidee->getRestitution() + collider->getRestitution()) / 2.0, false);
						pushBodiesApart(collider, collidee, *colInfo->at(0)->colNormVector, colInfo->at(0)->penDepth);
					}
					else {
						while (colCount < maxCollisions) {
							bool colTriggered = false;
							for (int i = 1; colCount < maxCollisions && i < colInfo->size(); i++) {
								if (collider->verifyCollisionPointNotExiting(*collidee, *colInfo->at(i)->colNormVector, *colInfo->at(i)->point)) {
									colTriggered = true;
									colCount++;
									//RigidBody::ColPointInfo* pInfo = iterateForwards ? colInfo->at(i) : colInfo->at(colInfo->size() - i);
									RigidBody::ColPointInfo* pInfo = colInfo->at(i);
									//resolveImpulses(collider, collidee, *colInfo->at(i)->colNormVector, colInfo->at(i)->point, -0.3);
									resolveImpulses(collider, collidee, *colInfo->at(i)->colNormVector, colInfo->at(i)->point, dampenCollision? -0.5 : collidee->getRestitution(), contactCollision);
									//restitutionMultiplier *= restitutionReductionFactor;
								}
							}
							if (!colTriggered) {
								if (dampenCollision) {
									dampenCollision = false;
									colCount = 0;
									continue;
								}
								break;
							}
						}

						/*colCount = 0;
						while (colCount < maxCollisions) {
							bool colTriggered = false;
							for (int i = 1; colCount < maxCollisions && i < colInfo->size(); i++) {
								if (collider->verifyCollisionPointNotExiting(*collidee, *colInfo->at(i)->colNormVector, *colInfo->at(i)->point)) {
									colTriggered = true;
									colCount++;
									//RigidBody::ColPointInfo* pInfo = iterateForwards ? colInfo->at(i) : colInfo->at(colInfo->size() - i);
									RigidBody::ColPointInfo* pInfo = colInfo->at(i);
									//resolveImpulses(collider, collidee, *colInfo->at(i)->colNormVector, colInfo->at(i)->point, -0.3);
									resolveImpulses(collider, collidee, *pInfo->colNormVector, pInfo->point, collidee->getRestitution());
									//restitutionMultiplier *= restitutionReductionFactor;
								}
							}
							if (!colTriggered)
								break;
						}*/
						//resolveImpulses(collider, collidee, *colInfo->at(0)->colNormVector, colInfo->at(0)->point, restitutionMultiplier);
						pushBodiesApart(collider, collidee, *colInfo->at(0)->colNormVector, colInfo->at(0)->penDepth);
						//iterateForwards = !iterateForwards;
					}

				}

				for (int i = 1; i < b1ColInfo.size(); i++) {
					delete b1ColInfo.at(i);
				}
				if (b1ColInfo.size() == 1) {
					delete(b1ColInfo.at(0));
				}
				for (int i = 1; i < b2ColInfo.size(); i++) {
					delete b2ColInfo.at(i);
				}
				if (b2ColInfo.size() == 1) {
					delete(b2ColInfo.at(0));
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
