#include "RigidBody.h"
#include "../Math/transformation3D.h"
#include <stdio.h>

RigidBody::RigidBody(const std::vector<ConvexHull*>& hulls, double density, double friction, double restitution, bool fixed, bool custumCOM, Vector3D com) {
	this->fixed = fixed;
	this->friction = friction;
	this->restitution = restitution;
	this->hulls = hulls;
	ID = -1; //unitialized
	trackHistory = false;

	findBodyMassAndInertia(density, custumCOM, com);

	for (ConvexHull* hull : hulls) {
		for (RigidSurface* s : *hull->getSurfaces()) {
			for (Vector3D* p : *s->getPoints()) {
				pointsToTransform.push_back(p);
			}
		}
		pointsToTransform.push_back(hull->getCOMPointer());
	}
	
	if (fixed) {
		findDimensions();
	}

	findCollisionRadius();

	copyPoints();
}

RigidBody::RigidBody(const RigidBody& body) {
	this->fixed = body.fixed;
	this->friction = body.friction;
	this->restitution = body.restitution;

	ID = -1; //unitialized
	trackHistory = false;

	centerOfMass = body.centerOfMass;
	mass = body.mass;
	inverseMass = body.inverseMass;
	inertiaTensor = body.inertiaTensor;

	for (ConvexHull* h : body.hulls) {
		hulls.push_back(new ConvexHull(*h));
	}

	for (ConvexHull* hull : hulls) {
		for (RigidSurface* s : *hull->getSurfaces()) {
			for (Vector3D* p : *s->getPoints()) {
				pointsToTransform.push_back(p);
			}
		}
		pointsToTransform.push_back(hull->getCOMPointer());
	}

	if (fixed) {
		findDimensions();
	}

	findCollisionRadius();

	copyPoints();
}

RigidBody::RigidBody() {

}

RigidBody::~RigidBody() {
	for (ConvexHull* h : hulls) {
		delete h;
	}
}

void RigidBody::addNoCol(uint16_t ID) {
	noColList.push_back(ID);
}

bool RigidBody::onNoColList(uint16_t ID) {
	for (int i : noColList) {
		if (i == ID) {
			return true;
		}
	}
	return false;
}

void RigidBody::findDimensions() {
	double maxX = -INFINITY;
	double minX = INFINITY;
	double maxY = -INFINITY;
	double minY = INFINITY;
	double maxZ = -INFINITY;
	double minZ = INFINITY;

	for (Vector3D* p : pointsToTransform) {
		maxX = std::fmax(p->x, maxX);
		minX = std::fmin(p->x, minX);
		maxY = std::fmax(p->y, maxY);
		minY = std::fmin(p->y, minY);
		maxZ = std::fmax(p->z, maxZ);
		minZ = std::fmin(p->z, minZ);
	}

	
	fixedDimensions = Vector3D(maxX - minX, maxY - minY, maxZ - minZ);
	dimCenter = Vector3D(maxX + minX, maxY + minY, maxZ + minZ).multiply(0.5);
}

Vector3D RigidBody::getDimCenter() {
	return dimCenter;
}

Rotor RigidBody::getOrientation() {
	return orientation;
}

void RigidBody::copyPoints() {
	for (Vector3D* p : pointsToTransform) {
		pointsOG.push_back(p->sub(centerOfMass));
	}
}

uint16_t RigidBody::getID() {
	return ID;
}

void RigidBody::setID(uint16_t ID) {
	this->ID = ID;
}

std::vector<CollisionInfo>* RigidBody::getCollHistory() {
	return &collHistory;
}

void RigidBody::clearCollHistory() {
	collHistory.clear();
}

void RigidBody::addToColHistory(uint16_t collider, double magnitude) {
	collHistory.push_back(CollisionInfo(collider, magnitude));
}

bool RigidBody::trackingCollHistory() {
	return trackHistory;
}

void RigidBody::setTrackCollHistory(bool b) {
	trackHistory = b;
}

bool RigidBody::alreadyTestedAgainst(uint16_t colliderID) {
	for (uint16_t id : testedList) {
		if (id == colliderID) {
			return true;
		}
	}
	return false;
}

void RigidBody::addTestedAgainst(uint16_t colliderID) {
	testedList.push_back(colliderID);
}

void RigidBody::clearTestedList() {
	testedList.clear();
}

std::vector<ConvexHull*>* RigidBody::getHulls() {
	return &hulls;
}

void RigidBody::findBodyMassAndInertia(double density, bool useCustumCOM, Vector3D custumCOM) {

	mass = 0;

	for (ConvexHull* hull : hulls) {
		mass += hull->getMass();
		Vector3D hullCOM = hull->getCenterOfMass();
		centerOfMass.x += hullCOM.x * hull->getMass();
		centerOfMass.y += hullCOM.y * hull->getMass();
		centerOfMass.z += hullCOM.z * hull->getMass();
	}

	inverseMass = 1.0 / mass;
	centerOfMass.x /= mass;
	centerOfMass.y /= mass;
	centerOfMass.z /= mass;

	if (useCustumCOM) {
		centerOfMass = custumCOM;
	}

	for (ConvexHull* hull : hulls) {
		Vector3D hullCOM = hull->getCenterOfMass();
		Vector3D hullRel;
		hullRel.x = hullCOM.x - centerOfMass.x;
		hullRel.y = hullCOM.y - centerOfMass.y;
		hullRel.z = hullCOM.z - centerOfMass.z;
		Matrix33* hTens = hull->getInertia();

		inertiaTensor.elements[0][0] += hTens->elements[0][0] + hull->getMass() * (hullRel.y * hullRel.y + hullRel.z * hullRel.z);
		inertiaTensor.elements[0][1] += hTens->elements[0][1] - hull->getMass() * (hullRel.x * hullRel.y);
		inertiaTensor.elements[0][2] += hTens->elements[0][2] - hull->getMass() * (hullRel.x * hullRel.z);
		inertiaTensor.elements[1][0] += hTens->elements[1][0] - hull->getMass() * (hullRel.x * hullRel.y);
		inertiaTensor.elements[1][1] += hTens->elements[1][1] + hull->getMass() * (hullRel.x * hullRel.x + hullRel.z * hullRel.z);
		inertiaTensor.elements[1][2] += hTens->elements[1][2] - hull->getMass() * (hullRel.z * hullRel.y);
		inertiaTensor.elements[2][0] += hTens->elements[2][0] - hull->getMass() * (hullRel.x * hullRel.z);
		inertiaTensor.elements[2][1] += hTens->elements[2][1] - hull->getMass() * (hullRel.z * hullRel.y);
		inertiaTensor.elements[2][2] += hTens->elements[2][2] + hull->getMass() * (hullRel.x * hullRel.x + hullRel.y * hullRel.y);
	}

	tensorInverse = inertiaTensor.invert();

	/*for (int i = 0; i < 9; i++) {
		printf("%f, ", inertiaTensor[i]);
		if ((i + 1) % 3 == 0)
			printf("\n");
	}

	printf("%f, %f, %f\n", centerOfMass.x, centerOfMass.y, centerOfMass.z);

	printf("%f\n", mass);
	printf("____________________________________\n");*/
}

void RigidBody::setVelocity(Vector3D vel) {
	this->velocity = vel;
}

void RigidBody::setAngVelocity(Vector3D aVel) {
	this->angularVelocity = aVel;
}

void RigidBody::acclerateLineraly(const Vector3D changeInVelocity) {
	if (fixed)
		return;
	velocity = velocity.add(changeInVelocity);
}

void RigidBody::translate(const Vector3D translation) {
	centerOfMass = centerOfMass.add(translation);
	transformation3D::translatePoints(&pointsToTransform, translation);
}

Vector3D RigidBody::getDimensions() {
	return fixedDimensions;
}

void RigidBody::setToOrientation(Rotor rotor) {
	orientation = rotor;
	for (int i = 0; i < pointsOG.size(); i++) {
		*pointsToTransform.at(i) = centerOfMass.add(orientation.rotate(pointsOG.at(i)));
	}
	if (fixed) {
		findDimensions();
	}
}

//Method from talk by Erin Catto, "Physics for Game Programmers: Numberical Methods"
//can view here: https://www.gdcvault.com/play/1022196/Physics-for-Game-Programmers-Numerical
//
Vector3D RigidBody::gyroAccel(double time) {
	Vector3D wB = orientation.getInverse().rotate(angularVelocity);
	Vector3D w2B = wB;

	int nItr = 1;
	for (int i = 0; i < nItr; i++) {
		Vector3D funcW = (inertiaTensor * w2B.sub(wB)).add(w2B.crossProduct(inertiaTensor * w2B).multiply(time));
		Matrix33 jacobian = inertiaTensor + (((Matrix33::skew(w2B) * inertiaTensor) + (Matrix33::skew(inertiaTensor * w2B) * -1)) * time);
		w2B = Matrix33::newtonSolve(w2B, funcW, jacobian);
	}

	return orientation.rotate(w2B);
}

std::vector<Vector3D>* RigidBody::getAllPoints() {
	return &pointsOG;
}

void RigidBody::moveInTime(double time) {
	Vector3D linMove = velocity.multiply(time);
	centerOfMass = centerOfMass.add(linMove);

	if (angularVelocity.notZero()) {

		angularVelocity = gyroAccel(time);

		double rotationMagnitude = angularVelocity.getMagnitude() * time;
		Vector3D rotationAxis = angularVelocity.getUnitVector();
	
		setToOrientation(orientation.applyRotor(Rotor(rotationAxis, rotationMagnitude)));
	}
	else {
		transformation3D::translatePoints(&pointsToTransform, linMove);
	}
	
}

bool RigidBody::getFixed() {
	return fixed;
}

double iExp(double base, int x) {
	double prod = 1;
	for (int i = 0; i < x; i++) {
		prod *= base;
	}
	return prod;
}

Vector3D RigidBody::findVectorRelativeToBodyFrame(const Vector3D vector) {
	return orientation.getInverse().rotate(vector);
}

double RigidBody::getInertiaOfAxis(const Vector3D axisIn) {
	Vector3D axis = findVectorRelativeToBodyFrame(axisIn).getUnitVector();
	Vector3D iAxis = inertiaTensor * axis;
	return axis.dotProduct(iAxis);
}

double RigidBody::findInverseInertiaOfAxis(const Vector3D inputAxis) {	
	if (fixed || !inputAxis.notZero())
		return 0;
	
	return 1.0 / getInertiaOfAxis(inputAxis);
}

void RigidBody::applyImpulseAtPosition(const Vector3D impulse, const Vector3D position) {
	if (fixed)
		return;
	velocity.x += impulse.x * inverseMass;
	velocity.y += impulse.y * inverseMass;
	velocity.z += impulse.z * inverseMass;

	Vector3D comToPoint(centerOfMass, position);
	Vector3D deltaW = comToPoint.crossProduct(impulse);
	deltaW = deltaW.multiply(findInverseInertiaOfAxis(deltaW));
	angularVelocity = angularVelocity.add(deltaW);
}

void RigidBody::findCollisionRadius() {
	double radius = 0;
	for (Vector3D* p : pointsToTransform) {
		double dist = p->sub(centerOfMass).getMagnitude();
		if (dist > radius)
			radius = dist;
	}
	
	collisionRadiusSquared = radius * radius;
	collisionRadius = radius;
	
}

double RigidBody::getFriction() const {
	return friction;
}

double RigidBody::getRestitution() const {
	return restitution;
}

double RigidBody::getCollisionRadius() const {
	return collisionRadius;
}

Vector3D RigidBody::getAngularVelocity() {
	return angularVelocity;
}

double RigidBody::getMass() const {
	return mass;
}

double RigidBody::getInverseMass() const {
	if (fixed)
		return 0;
	return inverseMass;
}

double RigidBody::getCollisionRadiusSquared() const {
	return collisionRadiusSquared;
}

Vector3D RigidBody::getCenterOfMass() {
 	return centerOfMass;
}

bool RigidBody::bodiesInCollisionRange(RigidBody* body) {
	double xDist = centerOfMass.x - body->getCenterOfMass().x;
	double yDist = centerOfMass.y - body->getCenterOfMass().y;
	double zDist = centerOfMass.z - body->getCenterOfMass().z;

	double sqrDist = xDist * xDist + yDist * yDist + zDist * zDist;
	double colDist = (collisionRadius + body->getCollisionRadius()) * (collisionRadius + body->getCollisionRadius());
	return sqrDist <= colDist;
}

Vector3D RigidBody::getVelocityOfPointDueToAngularVelocity(const Vector3D point) const {
	Vector3D centerOfMassToPoint(centerOfMass, point);
	return angularVelocity.crossProduct(centerOfMassToPoint);
}

Vector3D RigidBody::getVelocityOfPoint(const Vector3D point) const {
	return getVelocityOfPointDueToAngularVelocity(point).add(velocity);
}

Vector3D RigidBody::getVelocity() {
	return velocity;
}

double RigidBody::getRadialDistanceOfPoint(const Vector3D point) {
	double dx = centerOfMass.x - point.x;
	double dy = centerOfMass.y - point.y;
	double dz = centerOfMass.z - point.z;
	return dx * dx + dy * dy + dz * dz;
}

bool RigidBody::verifyCollisionPointNotExiting(RigidBody* body, const Vector3D normalVector, const Vector3D p) {
	Vector3D vPThisBody = getVelocityOfPoint(p);
	Vector3D vPOtherBody = body->getVelocityOfPoint(p);
	Vector3D vPRelative = vPThisBody.sub(vPOtherBody);
	return vPRelative.dotProduct(normalVector) < 0;
}
