#include "PhysicsEngine.h"
#include <iostream>


const double GRAV_DEFAULT = 20;
const double TIMESTEP_DEFAULT = 0.035;

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
	Vector3D cldeTransform = nV.multiply(-dCldee);
	Vector3D cldrTransform = nV.multiply(dCldr);
	collider->translate(cldrTransform);
	collidee->translate(cldeTransform);
}

void PhysicsEngine::resolveImpulses(RigidBody* collider, RigidBody* collidee, const Vector3D nV, Point3D colPoint, const std::vector<ConvexHull::ColPointInfo> supPoints, double restitution, double colDepth, bool faceCol) {

	Vector3D vCldrP0 = collider->getVelocityOfPoint(colPoint);
	Vector3D vCldeP0 = collidee->getVelocityOfPoint(colPoint);
	Vector3D velRel = vCldeP0.sub(vCldrP0);
	Vector3D rClde(collidee->getCenterOfMass(), colPoint);
	Vector3D rCldr(collider->getCenterOfMass(), colPoint);
	Vector3D jRotAxisCldr = rCldr.crossProduct(nV);
	Vector3D jRotAxisClde = rClde.crossProduct(nV);
	double invIClde = collidee->findInverseInertiaOfAxis(jRotAxisClde);
	double invICldr = collider->findInverseInertiaOfAxis(jRotAxisCldr);
	double normVel = nV.dotProduct(velRel);
	Vector3D rCldeXn = rClde.crossProduct(nV);
	Vector3D rCldrXn = rCldr.crossProduct(nV);

	double numerator = -(restitution + 1) * normVel;
	double denomenator = (collider->getInverseMass() + collidee->getInverseMass() + rCldrXn.getMagnitudeSquared() * invICldr + rCldeXn.getMagnitudeSquared() * invIClde);

	double impulseMagnitude = abs(numerator / denomenator);
	Vector3D impulse = nV.multiply(impulseMagnitude);

	Vector3D velParralel = nV.multiply(normVel);
	Vector3D k = velRel.sub(velParralel);
	Vector3D frictionImpulse;
	if (k.notZero() && !faceCol) {
		k = k.getUnitVector();
		jRotAxisCldr = rCldr.crossProduct(k);
		jRotAxisClde = rClde.crossProduct(k);
		invIClde = collidee->findInverseInertiaOfAxis(jRotAxisClde);
		invICldr = collider->findInverseInertiaOfAxis(jRotAxisCldr);
		numerator = k.dotProduct(velRel);
		Vector3D rCldrXk = rCldr.crossProduct(k);
		Vector3D rCldeXk = rCldr.crossProduct(k);
		denomenator = (collider->getInverseMass() + collidee->getInverseMass() + rCldrXk.getMagnitudeSquared() * invICldr + rCldeXk.getMagnitudeSquared() * invIClde);
		double frictionImpMag = abs(numerator / denomenator);
		//std::cout << contactCollision << ", " << (velParralel.getMagnitudeSquared() < staticVel) << "\n";
		double maxFriction = impulseMagnitude * collidee->getFriction();
		if (frictionImpMag > maxFriction) {
			frictionImpMag = maxFriction;
		}
		frictionImpulse = k.multiply(frictionImpMag);
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

	if (impulse.notZero()) {
		collider->applyImpulseAtPosition(impulse, colPoint);

		//std::cout << "veloc after: " << collider->getAngularVelocity()->dotProduct(up) << "\n\n";

		impulse = impulse.multiply(-1);
		collidee->applyImpulseAtPosition(impulse, colPoint);
	}
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
		collider->applyImpulseAtPosition(frictionImpulse, colPoint);

		
		
		//Vector3D rotAxis;
		//comP.crossProduct(frictionImpulse, &rotAxis);
		
		//std::cout << "dot product: " << rotAxis.dotProduct(up) << "\n";
		//std::cout << "veloc after: " << collider->getAngularVelocity()->dotProduct(up) << "\n";

		//std::cout << "vel rel: " << k.x << " " << k.y << " " << k.z << "\n";
		//Vector3D frictionUnit;
		//frictionImpulse.getUnitVector(&frictionUnit);
		//std::cout << "impulse: " << frictionUnit.x << " " << frictionUnit.y << " " << frictionUnit.z << "\n";

		//std::cout << "/t/t-------------------\n";

		frictionImpulse = frictionImpulse.multiply(-1);
		collidee->applyImpulseAtPosition(frictionImpulse, colPoint);
		double angVelAfter = collider->getAngularVelocity().getMagnitudeSquared();
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

			std::vector<ConvexHull::ColPointInfo> supPoints;
			Vector3D norm;
			Point3D colPoint;
			double colDepth = -1;
			bool separatingAxis;
			bool isFaceCollision = true;

			RigidBody* collider = body2;
			RigidBody* collidee = body1;

			for (ConvexHull* hullA : *body1->getHulls()) {
				for (ConvexHull* hullB : *body2->getHulls()) {

					if (!hullA->hullsInCollisionRange(*hullB))
						continue;

					std::vector<ConvexHull::ColPointInfo> supPointsA;
					Vector3D normA;
					Point3D colPointA;
					double colDepthA;
					bool separatingAxisA;

					bool faceCollision = true;

					std::vector<ConvexHull::ColPointInfo> supPointsB;
					Vector3D normB;
					Point3D colPointB;
					double colDepthB;
					bool separatingAxisB;

					Vector3D normE;
					Point3D colPointE;
					double colDepthE;
					bool separatingAxisE;

					RigidBody* potCollider = body2;
					RigidBody* potCollidee = body1;

					bool aCol = hullA->SATColliderDetect(hullB, &supPointsA, &colPointA, &normA, &colDepthA, &separatingAxisA);
					bool bCol = hullB->SATColliderDetect(hullA, &supPointsB, &colPointB, &normB, &colDepthB, &separatingAxisB);
					bool eCol = hullA->SATEdgeCol(hullB, &colPointE, &normE, &colDepthE, &separatingAxisE);

					if ((aCol || bCol || eCol) && !(separatingAxisA || separatingAxisB || separatingAxisE)) {
						if (!aCol || (eCol && colDepthE < colDepthA)) {
							normA = normE;
							colPointA = colPointE;
							colDepthA = colDepthE;
							faceCollision = false;
						}
						if (!(aCol || eCol) || (bCol && colDepthB < colDepthA)) {
							normA = normB;
							colPointA = colPointB;
							colDepthA = colDepthB;
							potCollidee = body2;
							potCollider = body1;
							faceCollision = true;
							supPointsA = supPointsB;
						}

						if (colDepthA < colDepth || colDepth == -1) {
							supPoints = supPointsA;
							norm = normA;
							colPoint = colPointA;
							colDepth = colDepthA;
							separatingAxis = separatingAxisA;
							isFaceCollision = faceCollision;
							collidee = potCollidee;
							collider = potCollider;

							//bool aCol = hullA->SATColliderDetect(hullB, &supPointsA, &colPointA, &normA, &colDepthA, &separatingAxisA);
						}
					}
				}
			}
			//bool eCol = body1->SATEdgeCol(body2, &colPointE, &normE, &colDepthE, &separatingAxisE);
			//printf("%f, %f, %f, %f, %f\n", norm.x, norm.y, norm.z, colDepth, collidee->getInverseMass());
			if (colDepth != -1) {

				//printf("%f, %f, %f, %f, %f, %f, %f, %f, %d\n", norm.x, norm.y, norm.z, colPoint.x, colPoint.y, colPoint.z, colDepth, collidee->getInverseMass(), isFaceCollision);

				bool contactColl = false;

				if (isFaceCollision) {
					Vector3D vCldrP0 = collider->getVelocityOfPoint(colPoint);
					Vector3D vCldeP0 = collidee->getVelocityOfPoint(colPoint);
					Vector3D velRel = vCldeP0.sub(vCldrP0);
					double normVel = norm.dotProduct(velRel);
					if (true || normVel < gravity.getMagnitude() / (2.25)) {
						contactColl = true;
						Point3D average(0, 0, 0);
						double depth = 0;
						for (ConvexHull::ColPointInfo supPoint : supPoints) {
							average.x += supPoint.point.x;
							average.y += supPoint.point.y;
							average.z += supPoint.point.z;
							depth += supPoint.penDepth;
						}
						average.x /= supPoints.size();
						average.y /= supPoints.size();
						average.z /= supPoints.size();
						depth /= supPoints.size();
						//printf("%f, %f, %f, %f, hmm yes\n", average.x, average.y, average.z, depth);
						//printf("exiting I guess %d\n", supPoints.size());
						if (collider->verifyCollisionPointNotExiting(*collidee, norm, average)) {
							resolveImpulses(collider, collidee, norm, average, supPoints, (normVel < gravity.getMagnitude() / (2.25)) ? 0 : collidee->getRestitution(), depth, false);
						}
						/*for (RigidBody::ColPointInfo supPoint : supPoints) {
							printf("exiting I guess %d\n", supPoints.size());
							if (collider->verifyCollisionPointNotExiting(*collidee, norm, supPoint.point)) {
								printf("penDepth: %f\n", supPoint.penDepth);

								resolveImpulses(collider, collidee, norm, supPoint.point, supPoints, -0.5 * supPoint.penDepth, colDepth, false);
							}
						}*/
					}
				}

				if (!contactColl && !collidee->verifyCollisionPointNotExiting(*collider, norm, colPoint)) {
					resolveImpulses(collider, collidee, norm, colPoint, supPoints, collidee->getRestitution(), colDepth, false);
				}

				pushBodiesApart(collider, collidee, norm, (contactColl) ? 0.15 * colDepth : colDepth);

			}
					
		}
			/*int maxItr = 5;
			int eps = 0.1;
			double dist = -1;
			double tDelta = timestep / 2.0;
			int itr = 0;
			
			while (!((dist = body1->GJK(*body2, &norm, &colPoint)) < 0 && -dist < eps) && itr < maxItr) {
				itr++;
				double t = (dist < 0)? -tDelta : tDelta;
				body1->moveInTime(t);
				body2->moveInTime(t);
				tDelta /= 2.0;
			}

			resolveImpulses(body1, body2, norm.getInverse(), colPoint,
				(body1->getRestitution() + body2->getRestitution()) / 2.0, false);
			pushBodiesApart(collider, collidee, colInfo->at(0)->colNormVector, colInfo->at(0)->penDepth);

			*/

			/*std::vector<RigidBody::ColPointInfo*> b1ColInfo;
			std::vector<RigidBody::ColPointInfo*> b2ColInfo;

			bool collisionsOccuring = true;

			int count = 0;
			int max = 3;
			while (collisionsOccuring && count < max) {
				count++;
				body1->findCollisionInformationAsCollider(&b1ColInfo, *body2, timestep);
				body2->findCollisionInformationAsCollider(&b2ColInfo, *body1, timestep);

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
				else if (b2ColInfo.size() == 0 || b1ColInfo.at(0)->penDepth > b2ColInfo.at(0)->penDepth) {
					b1Collider = true;
				}
				else {
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
						Vector3D vCldrP0 = collider->getVelocityOfPoint(colInfo->at(0)->point);
						Vector3D vCldeP0 = collidee->getVelocityOfPoint(colInfo->at(0)->point);
						Vector3D velRel = vCldeP0.sub(vCldrP0);
						double normVel = colInfo->at(0)->colNormVector.dotProduct(velRel);
						contactCollision = normVel < reductionVel;
					}
					bool dampenCollision = contactCollision;
					//static bool iterateForwards = true;

					int k = colInfo->size();

					while (colCount < maxCollisions) {
						bool colTriggered = false;
						for (int i = 0; colCount < maxCollisions && i < colInfo->size(); i++) {
							if (collider->verifyCollisionPointNotExiting(*collidee, colInfo->at(i)->colNormVector, colInfo->at(i)->point)) {
								colTriggered = true;
								colCount++;
								RigidBody::ColPointInfo* pInfo = colInfo->at(i);
								resolveImpulses(collider, collidee, colInfo->at(i)->colNormVector, &colInfo->at(i)->point,
									dampenCollision? -0.5 : collidee->getRestitution(), contactCollision);
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
					pushBodiesApart(collider, collidee, colInfo->at(0)->colNormVector, colInfo->at(0)->penDepth);				
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
			*/

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
