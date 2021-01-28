#include "PhysicsEngine.h"
#include <iostream>
#include <chrono>

PhysicsEngine::PhysicsEngine(double timestep) {
	this->gravity = Vector3D(0, 20, 0);//default
	this->timestep = timestep;
	reductionVel = 2 * timestep * timestep * gravity.getMagnitudeSquared();
	staticVel = 0.1;
	setOctree(true);
	fpsCap = 1;
	root = nullptr;
}

void PhysicsEngine::setGravity(Vector3D gravity) {
	this->gravity = gravity;
}

uint16_t PhysicsEngine::giveID() {
	static uint16_t id = 0;
	return id++;
}

void PhysicsEngine::setOctree(bool useOctree, Vector3D octreeOrigin, double octreeSize, double octreeMinSize) {
	this->useOctree = useOctree;
	this->octreeOrigin = octreeOrigin.add(Vector3D(-octreeSize / 2.0, -octreeSize / 2.0, -octreeSize / 2.0));
	this->octreeSize = octreeSize;
	this->octreeMin = octreeMinSize;
}

OctreeNode* PhysicsEngine::getOctreeRoot() {
	return root;
}

void PhysicsEngine::addRigidBody(RigidBody* body) {
	rigidBodies.push_back(body);
	body->setID(giveID());
}

bool PhysicsEngine::removeRigidBody(RigidBody* body) {
	for (int i = 0; i < rigidBodies.size(); i++) {
		if (rigidBodies.at(i) == body) {		
			rigidBodies.erase(rigidBodies.begin() + i);
			return true;
		}
	}
	return false;
}

void PhysicsEngine::removeAllBodies() {
	rigidBodies.clear();
}

double PhysicsEngine::getTimestep() {
	return timestep;
}

void PhysicsEngine::pushBodiesApart(RigidBody* collider, RigidBody* collidee, const Vector3D nV, double colDepth) {
	double pushDist = colDepth + 0.1;
	double dCldr = pushDist / (1 + collider->getMass() * collidee->getInverseMass());
	if (collider->getFixed()) {
		dCldr = 0;
	}
	double dCldee = pushDist - dCldr;
	Vector3D cldeTransform = nV.multiply(-dCldee);
	Vector3D cldrTransform = nV.multiply(dCldr);
	collider->translate(cldrTransform);
	collidee->translate(cldeTransform);
}

void PhysicsEngine::resolveImpulses(RigidBody* collider, RigidBody* collidee, const Vector3D nV, Vector3D colPoint, const std::vector<ConvexHull::ColPointInfo> supPoints, double restitution) {

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
	if (k.notZero()) {
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
		double maxFriction = impulseMagnitude * collidee->getFriction();
		if (frictionImpMag > maxFriction) {
			frictionImpMag = maxFriction;
		}
		frictionImpulse = k.multiply(frictionImpMag);
	}

	if (impulse.notZero()) {
		collider->applyImpulseAtPosition(impulse, colPoint);
		impulse = impulse.multiply(-1);
		collidee->applyImpulseAtPosition(impulse, colPoint);

		//collision history is meant to be used by outside classes to be able to know what physics
		//stuff has taken place since their last checked
		if (collider->trackingCollHistory()) {
			collider->addToColHistory(collidee->getID(), impulseMagnitude);
		}
		if (collidee->trackingCollHistory()) {
			collidee->addToColHistory(collider->getID(), impulseMagnitude);
		}
	}
	if (frictionImpulse.notZero()) {
		collider->applyImpulseAtPosition(frictionImpulse, colPoint);
		frictionImpulse = frictionImpulse.multiply(-1);
		collidee->applyImpulseAtPosition(frictionImpulse, colPoint);
		double angVelAfter = collider->getAngularVelocity().getMagnitudeSquared();
	}
}

bool PhysicsEngine::getColDetectInfo(RigidBody* body1, RigidBody* body2, std::vector<ConvexHull::ColPointInfo>* supPoints, Vector3D* norm,
	Vector3D* colPoint, double* colDepthOut, bool* isFaceCollision, RigidBody** collider, RigidBody** collidee) {

	bool colFound = false;
	double colDepth = -1;
	for (ConvexHull* hullA : *body1->getHulls()) {
		for (ConvexHull* hullB : *body2->getHulls()) {

			if (!hullA->hullsInCollisionRange(hullB)) {
				continue;
			}

			int winningCol = -1; //, 0 is A, 1 B, 2 E

			//collision info if hullA is collidee
			std::vector<ConvexHull::ColPointInfo> supPointsA;
			Vector3D normA;
			Vector3D colPointA;
			double colDepthA;
			bool separatingAxisA;

			//collisionInfo if hullB is collidee
			std::vector<ConvexHull::ColPointInfo> supPointsB;
			Vector3D normB;
			Vector3D colPointB;
			double colDepthB;
			bool separatingAxisB;

			//collision info in case of edge on edge collision
			Vector3D normE;
			Vector3D colPointE;
			double colDepthE;
			bool separatingAxisE;

			RigidBody* potCollider = body2;
			RigidBody* potCollidee = body1;

			bool aCol = hullA->SATColliderDetect(hullB, &supPointsA, &colPointA, &normA, &colDepthA, &separatingAxisA);
			bool bCol = hullB->SATColliderDetect(hullA, &supPointsB, &colPointB, &normB, &colDepthB, &separatingAxisB);
			bool eCol = hullA->SATEdgeCol(hullB, &colPointE, &normE, &colDepthE, &separatingAxisE);

			if ((aCol || bCol || eCol) && !(separatingAxisA || separatingAxisB || separatingAxisE)) {
				double winningDepth = 0;

				if (aCol && (!bCol || (bCol && colDepthB > colDepthA))) {
					winningCol = 0;
					winningDepth = colDepthA;
					if (eCol && colDepthE < colDepthA) {
						winningCol = 2;
						winningDepth = colDepthE;
					}
				}
				else if (bCol) {
					winningCol = 1;
					winningDepth = colDepthB;
					if (eCol && colDepthE < colDepthB) {
						winningCol = 2;
						winningDepth = colDepthE;
					}
				}
				else {
					winningCol = 2;
					winningDepth = colDepthE;
				}

				if (colDepth == -1 || winningDepth > colDepth) {
					colFound = true;

					switch (winningCol) {
					case 0:
						*collider = body2;
						*collidee = body1;
						*supPoints = supPointsA;
						*norm = normA;
						*colPoint = colPointA;
						colDepth = colDepthA;
						*isFaceCollision = true;
						break;
					case 1:
						*collider = body1;
						*collidee = body2;
						*supPoints = supPointsB;
						*norm = normB;
						*colPoint = colPointB;
						colDepth = colDepthB;
						*isFaceCollision = true;
						break;
					case 2:
						*norm = normE;
						*colPoint = colPointE;
						colDepth = colDepthE;
						*isFaceCollision = false;
						break;
					}
				}
			}
		}
	}
	*colDepthOut = colDepth;
	return colFound;
}

void PhysicsEngine::detectAndResolveCollisions(RigidBody* body1, RigidBody* body2) {
	static double narrowTests = 0;
	static double satTests = 0;
	static double collApply = 0;

	//checks to avoid unnecessary/unwanted physics
	if (body1->getFixed() && body2->getFixed()) {
		return;
	}
	if (body1->onNoColList(body2->getID()) || body2->onNoColList(body1->getID())) {
		return;
	}
	
	if (body1->alreadyTestedAgainst(body2->getID())) {
		return;
	}
	else {
		body1->addTestedAgainst(body2->getID());
		body2->addTestedAgainst(body1->getID());
	}

	auto t1 = std::chrono::system_clock::now();
	if (!body1->bodiesInCollisionRange(body2)) {
		auto t2 = std::chrono::system_clock::now();
		std::chrono::duration<float> t = t2 - t1;
		narrowTests += t.count();
		return;
	}
	auto t2 = std::chrono::system_clock::now();

	std::vector<ConvexHull::ColPointInfo> supPoints;
	Vector3D norm;
	Vector3D colPoint;
	double colDepth;
	bool isFaceCollision = true;

	RigidBody* collider = body2;
	RigidBody* collidee = body1;

	//find the intersection between hulls of each body with greatest penetration,
	//and resolve that collision
	bool colFound = getColDetectInfo(body1, body2, &supPoints, &norm, &colPoint, &colDepth, &isFaceCollision, &collider, &collidee);
	double ogDepth = colDepth;
	if (colFound && false) {
		double netT = 0;
		int maxItr = 5;
		double eps = 1;
		double t = -timestep / 2.0;
		for (int i = 0; i < maxItr && colDepth > eps; i++) {
			body1->moveInTime(t);
			body2->moveInTime(t);
			netT += t;
			if (getColDetectInfo(body1, body2, &supPoints, &norm, &colPoint, &colDepth, &isFaceCollision, &collider, &collidee)) {
				t = -abs(t) / 2.0;
			}
			else {
				t = abs(t) / 2.0;
			}
		}
		body1->moveInTime(-netT);
		body2->moveInTime(-netT);
	}

	auto t3 = std::chrono::system_clock::now();
	if (colFound) {

		bool contactColl = false;

		if (isFaceCollision) {
			//using an average of the points collided is a rough solution to poor contact behavior,
			//has some issues like not picking up torque from friction as well
			contactColl = true;
			Vector3D average(0, 0, 0);
			double depth = 0;
			/*for (ConvexHull::ColPointInfo supPoint : supPoints) {
				average.x += supPoint.point.x;
				average.y += supPoint.point.y;
				average.z += supPoint.point.z;
				depth += supPoint.penDepth;
			}
			average.x /= supPoints.size();
			average.y /= supPoints.size();
			average.z /= supPoints.size();
			depth /= supPoints.size();*/
			average = colPoint;

			if (collider->verifyCollisionPointNotExiting(collidee, norm, average)) {
				Vector3D vCldrP0 = collider->getVelocityOfPoint(colPoint);
				Vector3D vCldeP0 = collidee->getVelocityOfPoint(colPoint);
				Vector3D velRel = vCldeP0.sub(vCldrP0);
				double normVel = norm.dotProduct(velRel);
				resolveImpulses(collider, collidee, norm, average, supPoints, (normVel < gravity.getMagnitude() / (2.25)) ? 0 : collidee->getRestitution());
			}

		}
		else if (!collidee->verifyCollisionPointNotExiting(collider, norm, colPoint)) {
			resolveImpulses(collider, collidee, norm, colPoint, supPoints, (collidee->getRestitution() + collider->getRestitution()) / 2.0);
		}

		pushBodiesApart(collider, collidee, norm, (contactColl) ? 1/*0.15*/ * colDepth : colDepth);
		auto t4 = std::chrono::system_clock::now();

		std::chrono::duration<float> narrow = t2 - t1;
		std::chrono::duration<float> sat = t3 - t2;
		std::chrono::duration<float> resolve = t4 - t3;

		narrowTests += narrow.count();
		satTests += sat.count();
		collApply += resolve.count();

		//printf("narrowTest: %f, satTest: %f, collApply: %f\n", narrowTests, satTests, collApply);
	}
}

bool PhysicsEngine::bodyInUse(uint16_t bodyID) {
	for (RigidBody* b : rigidBodies) {
		if (b->getID() == bodyID) {
			return true;
		}
	}
	return false;
}

void PhysicsEngine::iterateEngine(double secondsPassed) {
	static double timeBuff = 0;
	timeBuff += secondsPassed;
	double maxBuff = std::fmax(0.1, timestep);
	timeBuff = std::fmin(maxBuff, timeBuff);

	//timestep = secondsPassed;
	while (timeBuff >= timestep) {
		timeBuff -= timestep;
		iterateEngineTimestep();
	}
}

void PhysicsEngine::iterateEngineTimestep() {
	auto t1 = std::chrono::system_clock::now();
	Vector3D gravityAcceleration(gravity.x * timestep, gravity.y * timestep, gravity.z * timestep);

	for (RigidBody* body : rigidBodies) {
		body->acclerateLineraly(gravityAcceleration);
		body->moveInTime(timestep);
		body->clearTestedList();
	}
	auto t2 = std::chrono::system_clock::now();
	double octreeTime = 0;

	if (useOctree) {

		if (root != nullptr) {
			delete root;
		}
		auto t3 = std::chrono::system_clock::now();
		root = new OctreeNode(octreeOrigin, octreeSize, octreeMin);
		auto t4 = std::chrono::system_clock::now();
		for (RigidBody* b : rigidBodies) {
			root->addBody(b);
		}
		auto t5 = std::chrono::system_clock::now();
		root->expandNode();
		auto t6 = std::chrono::system_clock::now();
		std::vector<OctreeNode*> octreeLeafs;
		root->getCollisionLeafs(&octreeLeafs);
		auto t7 = std::chrono::system_clock::now();

		std::chrono::duration<float> del = t3 - t2;
		std::chrono::duration<float> create = t4 - t3;
		std::chrono::duration<float> push = t5 - t4;
		std::chrono::duration<float> expand = t6 - t5;
		std::chrono::duration<float> findleaf = t7 - t6;

		//printf("del: %f, cre: %f, push: %f, expa: %f, leaf: %f\n", del.count(), create.count(), push.count(), expand.count(), findleaf.count());

		for (OctreeNode* leaf : octreeLeafs) {
			for (int i = 0; i < leaf->getBodies()->size(); i++) {
				for (int j = i + 1; j < leaf->getBodies()->size(); j++) {
					RigidBody* body1 = leaf->getBodies()->at(i);
					RigidBody* body2 = leaf->getBodies()->at(j);

					detectAndResolveCollisions(body1, body2);
				}
			}
		}
	}
	else {

		for (int i = 0; i < rigidBodies.size(); i++) {
			for (int j = i + 1; j < rigidBodies.size(); j++) {
				RigidBody* body1 = rigidBodies.at(i);
				RigidBody* body2 = rigidBodies.at(j);

				detectAndResolveCollisions(body1, body2);
			}
		}
	}

	auto t3 = std::chrono::system_clock::now();

	std::chrono::duration<float> update = t2 - t1;
	std::chrono::duration<float> checks = t3 - t2;

	//printf("Update time: %f, Octree time: %f, other: %f\n", update.count(), octreeTime, checks.count() - octreeTime);
}
