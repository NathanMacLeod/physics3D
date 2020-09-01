#include "RigidBody.h"
#include "transformation3D.h"
#include <stdio.h>

RigidBody::RigidBody(const std::vector<ConvexHull*>& hulls, double density, double friction, double restitution, bool fixed) {
	this->fixed = fixed;
	this->friction = friction;
	this->restitution = restitution;
	this->hulls = hulls;

	findBodyMassAndInertia(density);

	for (ConvexHull* hull : hulls) {
		for (Vector3D* p : *hull->getColPoints()) {
			pointsToTransform.push_back(p);
		}
	}

	findCollisionRadius();
	
	if (!fixed)
		velocity = Vector3D(0, 0, -15);

	orientationPoint1 = Vector3D(centerOfMass.x, centerOfMass.y - 1, centerOfMass.z);
	orientationPoint2 = Vector3D(centerOfMass.x + 1, centerOfMass.y, centerOfMass.z);

	pointsToTransform.push_back(&orientationPoint1);
	pointsToTransform.push_back(&orientationPoint2);
	pointsToTransform.push_back(&centerOfMass);
}

RigidBody::~RigidBody() {
	for (ConvexHull* h : hulls) {
		delete h;
	}
}

std::vector<ConvexHull*>* RigidBody::getHulls() {
	return &hulls;
}

void RigidBody::findBodyMassAndInertia(double density) {

	mass = 0;
	inertiaTensor = new double[9]{ 0, 0, 0, 0, 0, 0, 0, 0, 0 };

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

	for (ConvexHull* hull : hulls) {
		Vector3D hullCOM = hull->getCenterOfMass();
		Vector3D hullRel;
		hullRel.x = hullCOM.x - centerOfMass.x;
		hullRel.y = hullCOM.y - centerOfMass.y;
		hullRel.z = hullCOM.z - centerOfMass.z;
		double* hTens = hull->getInertia();

		inertiaTensor[0] += hTens[0] + hull->getMass() * (hullRel.y * hullRel.y + hullRel.z * hullRel.z);
		inertiaTensor[1] += hTens[1] - hull->getMass() * (hullRel.x * hullRel.y);
		inertiaTensor[2] += hTens[2] - hull->getMass() * (hullRel.x * hullRel.z);
		inertiaTensor[3] += hTens[3] - hull->getMass() * (hullRel.x * hullRel.y);
		inertiaTensor[4] += hTens[4] + hull->getMass() * (hullRel.x * hullRel.x + hullRel.z * hullRel.z);
		inertiaTensor[5] += hTens[5] - hull->getMass() * (hullRel.z * hullRel.y);
		inertiaTensor[6] += hTens[6] - hull->getMass() * (hullRel.x * hullRel.z);
		inertiaTensor[7] += hTens[7] - hull->getMass() * (hullRel.z * hullRel.y);
		inertiaTensor[8] += hTens[8] + hull->getMass() * (hullRel.x * hullRel.x + hullRel.y * hullRel.y);

	}
//	for (int i = 0; i < 9; i++) {
//		printf("%f\n", inertiaTensor[i]);
//	}
}

void RigidBody::acclerateLineraly(const Vector3D changeInVelocity) {
	if (fixed)
		return;
	velocity = velocity.add(changeInVelocity);
}

void RigidBody::translate(const Vector3D translation) {
	transformation3D::translatePoints(&pointsToTransform, translation);
}

void RigidBody::moveInTime(double time) {
	if (fixed && false)
		return;
	double rotationMagnitude = angularVelocity.getMagnitude() * time;
	Vector3D translation = velocity.multiply(time);
	Vector3D rotationAxis = angularVelocity.getUnitVector();
	transformation3D::translatePoints(&pointsToTransform, translation);
	if(rotationMagnitude != 0)
		transformation3D::rotatePointsAroundArbitraryAxis(&pointsToTransform, rotationAxis, centerOfMass.x, centerOfMass.y, centerOfMass.z, rotationMagnitude);
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
	Vector3D orientationVector1(centerOfMass, orientationPoint1);
	Vector3D orientationVector2(centerOfMass, orientationPoint2);
	Vector3D orientationVector3 = orientationVector1.crossProduct(orientationVector2);

	double negYVal = orientationVector1.dotProduct(vector);
	double xVal = orientationVector2.dotProduct(vector);
	double zVal = orientationVector3.dotProduct(vector);

	return Vector3D(xVal, -negYVal, zVal);
}

double RigidBody::findInverseInertiaOfAxis(const Vector3D inputAxis) {	
	if (fixed || !inputAxis.notZero())
		return 0;
	Vector3D axis = findVectorRelativeToBodyFrame(inputAxis).getUnitVector();
	double vec1 = inertiaTensor[0] * axis.x + inertiaTensor[1] * axis.y + inertiaTensor[2] * axis.z;
	double vec2 = inertiaTensor[3] * axis.x + inertiaTensor[4] * axis.y + inertiaTensor[5] * axis.z;
	double vec3 = inertiaTensor[6] * axis.x + inertiaTensor[7] * axis.y + inertiaTensor[8] * axis.z;
	double inertia = axis.x * vec1 + axis.y * vec2 + axis.z * vec3;
	return 1.0 / inertia;
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
	double greatestRadiusSquared = 0;
	for (Vector3D* p : pointsToTransform) {
		double squaredRadius = (centerOfMass.x - p->x) * (centerOfMass.x - p->x) + (centerOfMass.y - p->y) * (centerOfMass.y - p->y)
			+ (centerOfMass.z - p->z) * (centerOfMass.z - p->z);
		if (squaredRadius > greatestRadiusSquared)
			greatestRadiusSquared = squaredRadius;
	}
	
	collisionRadiusSquared = greatestRadiusSquared;
	collisionRadius = sqrt(greatestRadiusSquared);
	
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
