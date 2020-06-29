#include "RigidBody.h"
#include "transformation3D.h"
#include <stdio.h>

RigidBody::RigidBody(const std::vector<RigidSurface*>& surfaces, double density, double friction, double restitution, bool fixed) {
	this->fixed = fixed;
	this->friction = friction;
	this->restitution = restitution;
	this->surfaces = surfaces;	

	findBodyMassAndInertia(density);
	createReferenceCopies();
	findCollisionRadius();
	findColPointsEdges();

	for (RigidSurface* surface : surfaces) {
		pointsToTransform.push_back(surface->getNormalVectorPoint());
	}
	for (Point3D* p : colPoints) {
		pointsToTransform.push_back(p);
	}
	//if (!fixed)
	//	velocity.x = 5;

	orientationPoint1 = Point3D(centerOfMass.x, centerOfMass.y - 1, centerOfMass.z);
	orientationPoint2 = Point3D(centerOfMass.x + 1, centerOfMass.y, centerOfMass.z);

	pointsToTransform.push_back(&orientationPoint1);
	pointsToTransform.push_back(&orientationPoint2);
	pointsToTransform.push_back(&centerOfMass);
}

RigidBody::Edge::Edge(Point3D* p1, Point3D* p2) {
	this->p1 = p1;
	this->p2 = p2;
	inverseMagnitude = 1 / Vector3D(*p1, *p2).getMagnitude();
}

void RigidBody::findColPointsEdges() {
	for (RigidSurface* s : surfaces) {
		for (int i = 0; i < s->getPoints()->size(); i++) {
			Point3D* p = s->getPoints()->at(i);
			bool pAlreadyAdded = false;
			for (Point3D* cp : colPoints) {
				if (cp == p) {
					pAlreadyAdded = true;
					break;
				}
			}
			if (!pAlreadyAdded) {
				colPoints.push_back(p);
			}

			Point3D* p2 = (i + 1 == s->getPoints()->size()) ? s->getPoints()->at(0) : s->getPoints()->at(i + 1);
			bool edgeAlreadyAdded = false;
			//double epsilon = 0.0001;
			//Vector3D nV(*s->getPoints()->at(0), *s->getNormalVectorPoint());
			for (Edge* edge : colEdges) {
				if ((edge->p1 == p && edge->p2 == p2) || (edge->p2 == p && edge->p1 == p2)) {
					edgeAlreadyAdded = true;
					break;
				}
			}
			if (!edgeAlreadyAdded) {
				colEdges.push_back(new Edge(p, p2));
			}
		}
	}
}

void RigidBody::acclerateLineraly(const Vector3D changeInVelocity) {
	//std::cout << velocity.x << " " << velocity.y << " " << velocity.z << "\n";
	if (fixed)
		return;
	velocity = velocity.add(changeInVelocity);
}

void RigidBody::translate(const Vector3D translation) {
	transformation3D::translatePoints(&pointsToTransform, translation);
}

void RigidBody::moveInTime(double time) {
	if (fixed)
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

enum axis { X, Y, Z };

double getAxisVal(Point3D* p, enum axis axis) {
	switch (axis) {
	case X:
		return p->x;
		break;
	case Y:
		return p->y;
		break;
	case Z:
		return p->z;
		break;
	}
}

double iExp(double base, int x) {
	double prod = 1;
	for (int i = 0; i < x; i++) {
		prod *= base;
	}
	return prod;
}

void RigidBody::findBodyMassAndInertia(double density) {
	double v = 0, vX = 0, vY = 0, vZ = 0, vXSqrd = 0, vYSqrd = 0, vZSqrd = 0;
	double vXY = 0, vXZ = 0, vYZ = 0;
	
	for (RigidSurface* s : surfaces) {
		double sXSqrd, sYSqrd, sZSqrd, sXCbd, sYCbd, sZCbd;
		double sXSqrdY, sXSqrdZ, sYSqrdZ, sX, u;

		double* sA, * sB, * sC, * sASqrd, * sBSqrd, * sCSqrd;
		double* sACbd, * sBCbd, * sCCbd, * sASqrdB, * sBSqrdC, * sASqrdC, *sBSqrdA, *sCSqrdA, *sCSqrdB;

		double f = 0, fA = 0, fB = 0, fAB = 0, fASqrd = 0, fBSqrd = 0, fASqrdB = 0, fBSqrdA = 0;
		double fACbd = 0, fBCbd = 0;

		enum axis A; //horz axis
		enum axis B; //vert axis
		enum axis C; //normal axis

		double nA;
		double nB;
		double nC;
		double invC;
		
		Vector3D normV(*s->getPoints()->at(0), *s->getNormalVectorPoint());

		if (abs(normV.x) > abs(normV.y) && abs(normV.x) > abs(normV.z)) {
			C = X;
			A = Y;
			B = Z;
			nC = normV.x;
			nA = normV.y;
			nB = normV.z;

			sA = &u;
			sB = &u;
			sC = &sX;
			sASqrd = &sYSqrd;
			sBSqrd = &sZSqrd;
			sCSqrd = &sXSqrd;
			sACbd = &sYCbd;
			sBCbd = &sZCbd;
			sCCbd = &sXCbd;
			sASqrdB = &sYSqrdZ;
			sCSqrdB = &sXSqrdZ;
			sCSqrdA = &sXSqrdY;
			sBSqrdA = &u;
			sBSqrdC = &u;
			sASqrdC = &u;
			sASqrd = &sYSqrd;
			sBSqrd = &sZSqrd;
			sCSqrd = &sXSqrd;
		}
		else if (abs(normV.y) > abs(normV.x) && abs(normV.y) > abs(normV.z)) {
			C = Y;
			A = Z;
			B = X;
			nC = normV.y;
			nA = normV.z;
			nB = normV.x;

			sA = &u;
			sB = &sX;
			sC = &u;
			sASqrd = &sZSqrd;
			sBSqrd = &sXSqrd;
			sCSqrd = &sYSqrd;
			sACbd = &sZCbd;
			sBCbd = &sXCbd;
			sCCbd = &sYCbd;
			sBSqrdA = &sXSqrdZ;
			sBSqrdC = &sXSqrdY;
			sCSqrdA = &sYSqrdZ;
			sASqrdB = &u;
			sASqrdC = &u;
			sCSqrdB = &u;
			sASqrd = &sZSqrd;
			sBSqrd = &sXSqrd;
			sCSqrd = &sYSqrd;
		}
		else {
			C = Z;
			A = X;
			B = Y;
			nC = normV.z;
			nA = normV.x;
			nB = normV.y;

			sA = &sX;
			sB = &u;
			sC = &u;
			sASqrd = &sXSqrd;
			sBSqrd = &sYSqrd;
			sCSqrd = &sZSqrd;
			sACbd = &sXCbd;
			sBCbd = &sYCbd;
			sCCbd = &sZCbd;
			sASqrdB = &sXSqrdY;
			sBSqrdC = &sYSqrdZ;
			sASqrdC = &sXSqrdZ;
			sBSqrdA = &u;
			sCSqrdA = &u;
			sCSqrdB = &u;
			sASqrd = &sXSqrd;
			sBSqrd = &sYSqrd;
			sCSqrd = &sZSqrd;
		}

		for (int i = 0; i < s->getPoints()->size(); i++) {
			Point3D* p1 = s->getPoints()->at(i);
			Point3D* p2 = i + 1 == s->getPoints()->size() ? s->getPoints()->at(0) : s->getPoints()->at(i + 1);

			double a1 = getAxisVal(p1, A);
			double b1 = getAxisVal(p1, B);
			double a2 = getAxisVal(p2, A);
			double b2 = getAxisVal(p2, B);

			double a21 = a2 - a1;
			double b21 = b2 - b1;

			f += b21 * (a1 + a21 / 2);
			fA += 0.5 * b21 * (a1 * a1 + a1 * a21 + a21 * a21 / 3);
			fB += -0.5 * a21 * (b1 * b1 + b1 * b21 + b21 * b21 / 3);
			fAB += 0.5 * b21 * (b1 * (a1 * a1 + a1 * a21 + a21 * a21 / 3) + b21 * (a1 * a1 / 2.0 + 2 * a1 * a21 / 3.0 + a21 * a21 / 4.0));
			fASqrd += (1 / 3.0) * b21 * (a1 * a1 * a1 + 3 * a1 * a1 * a21 / 2.0 + a1 * a21 * a21 + a21 * a21 * a21 / 4.0);
			fBSqrd += -(1/3.0) * a21 * (b1 * b1 * b1 + 3 * b1 * b1 * b21 / 2.0 + b1 * b21 * b21 + b21 * b21 * b21 / 4.0);
			fASqrdB += (1 / 3.0) * b21 * (b1 * (a1 * a1 * a1 + 3 * a1 * a1 * a21 / 2.0 + a1 * a21 * a21 + a21 * a21 * a21 / 4.0)
				+ b21 * (a1 * a1 * a1 / 2.0 + a1 * a1 * a21 + 3 * a1 * a21 * a21 / 4.0 + a21 * a21 * a21 / 5.0));
			fBSqrdA += -(1 / 3.0) * a21 * (a1 * (b1 * b1 * b1 + 3 * b1 * b1 * b21 / 2.0 + b1 * b21 * b21 + b21 * b21 * b21 / 4.0)
				+ a21 * (b1 * b1 * b1 / 2.0 + b1 * b1 * b21 + 3 * b1 * b21 * b21 / 4.0 + b21 * b21 * b21 / 5.0));
			fACbd += 0.25 * b21 * (a1 * a1 * a1 * a1 + 2 * a1 * a1 * a1 * a21 + 2 * a1 * a1 * a21 * a21 + a1 * a21 * a21 * a21 + a21 * a21 * a21 * a21 / 5.0);
			fBCbd += -0.25 * a21 * (b1 * b1 * b1 * b1 + 2 * b1 * b1 * b1 * b21 + 2 * b1 * b1 * b21 * b21 + b1 * b21 * b21 * b21 + b21 * b21 * b21 * b21 / 5.0);
		}

		invC = 1 / nC;
		//double absInvC = abs(invC);
		Point3D* p = s->getPoints()->at(0);
		double k = -(normV.x * p->x + normV.y * p->y + normV.z * p->z);

		*sA = invC * fA;
		*sB = invC * fB;
		*sC = -invC * invC * (f * k + nA * fA + nB * fB);
		*sASqrd = invC * fASqrd;
		*sBSqrd = invC * fBSqrd;
		*sCSqrd = invC * invC * invC * (k * k * f + nA * nA * fASqrd + nB * nB * fBSqrd + 2 * k * nA * fA
			+ 2 * k * nB * fB + 2 * nA * nB * fAB);
		*sASqrdB = invC * fASqrdB;
		*sBSqrdA = invC * fBSqrdA;
		*sASqrdC = -invC * invC * (k * fASqrd + nA * fACbd + nB * fASqrdB);
		*sBSqrdC = -invC * invC * (k * fBSqrd + nB * fBCbd + nA * fBSqrdA);
		*sCSqrdA = invC * invC * invC * (k * k * fA + nA * nA * fACbd + nB * nB * fBSqrdA + 2 * k * nA * fASqrd + 2 * k * nB * fAB + 2 * nA * nB * fASqrdB);
		*sCSqrdB = invC * invC * invC * (k * k * fB + nA * nA * fASqrdB + nB * nB * fBCbd + 2 * k * nA * fAB + 2 * k * nB * fBSqrd + 2 * nA * nB * fBSqrdA);
		*sACbd = invC * fACbd;
		*sBCbd = invC * fBCbd;
		*sCCbd = -invC * invC * invC * invC * (k * k * k * f + nA * nA * nA * fACbd + nB * nB * nB * fBCbd + 3 * k * nA * nA * fASqrd + 3 * k * nB * nB * fBSqrd +
			3 * nA * nA * nB * fASqrdB + 3 * nA * nB * nB * fBSqrdA + 3 * k * k * nA * fA + 3 * k * k * nB * fB + 6 * k * nA * nB * fAB);

		//printf("sZSqrd: %f, sASqrd: %f, sBSqrd: %f, sCSqrd: %f\n", sZSqrd, *sASqrd, *sBSqrd, *sCSqrd);

		v += normV.x * sX;
		vX += 0.5 * normV.x * sXSqrd;
		vY += 0.5 * normV.y * sYSqrd;
		vZ += 0.5 * normV.z * sZSqrd;
		vXSqrd += (1.0 / 3.0) * normV.x * sXCbd;
		vYSqrd += (1.0 / 3.0) * normV.y * sYCbd;
		vZSqrd += (1.0 / 3.0) * normV.z * sZCbd;
		vXY += 0.5 * normV.x * sXSqrdY;
		vXZ += 0.5 * normV.x * sXSqrdZ;
		vYZ += 0.5 * normV.y * sYSqrdZ;

	}

	mass = v * density;
	centerOfMass.x = vX / v;
	centerOfMass.y = vY / v;
	centerOfMass.z = vZ / v;

	inverseMass = 1.0 / mass;
	inertiaTensor = new double[9] { 0, 0, 0, 0, 0, 0, 0, 0, 0};

	inertiaTensor[0] = density * (vYSqrd + vZSqrd - v * (centerOfMass.y * centerOfMass.y + centerOfMass.z * centerOfMass.z));
	inertiaTensor[1] = -density * (vXY - v * centerOfMass.x * centerOfMass.y);
	inertiaTensor[2] = -density * (vXZ - v * centerOfMass.x * centerOfMass.z);
	inertiaTensor[4] = density * (vXSqrd + vZSqrd - v * (centerOfMass.x * centerOfMass.x + centerOfMass.z * centerOfMass.z));
	inertiaTensor[5] = -density * (vYZ - v *centerOfMass.y * centerOfMass.z);
	inertiaTensor[8] = density * (vXSqrd + vYSqrd - v * (centerOfMass.x * centerOfMass.x + centerOfMass.y * centerOfMass.y));
	inertiaTensor[3] = inertiaTensor[1];
	inertiaTensor[6] = inertiaTensor[2];
	inertiaTensor[7] = inertiaTensor[5];

	
	/*for (int i = 0; i < 9; i++) {
		printf("%f, ", inertiaTensor[i]);
		if ((i + 1) % 3 == 0)
			printf("\n");
	}

	printf("%f, %f, %f\n", centerOfMass.x, centerOfMass.y, centerOfMass.z);

	printf("%f\n", mass);
	printf("____________________________________\n");*/
	
}

Vector3D RigidBody::findVectorRelativeToBodyFrame(const Vector3D vector) {
	Vector3D output;

	Vector3D orientationVector1(centerOfMass, orientationPoint1);
	Vector3D orientationVector2(centerOfMass, orientationPoint2);
	Vector3D orientationVector3 = orientationVector1.crossProduct(orientationVector2);

	double negYVal = orientationVector1.dotProduct(vector);
	double xVal = orientationVector2.dotProduct(vector);
	double zVal = orientationVector3.dotProduct(vector);

	output.x = xVal;
	output.y = -negYVal;
	output.z = zVal;
	return output;
}

double RigidBody::findInverseInertiaOfAxis(const Vector3D inputAxis) {
	if (fixed)
		return 0;
	Vector3D axis = findVectorRelativeToBodyFrame(inputAxis).getUnitVector();
	double vec1 = inertiaTensor[0] * axis.x + inertiaTensor[1] * axis.y + inertiaTensor[2] * axis.z;
	double vec2 = inertiaTensor[3] * axis.x + inertiaTensor[4] * axis.y + inertiaTensor[5] * axis.z;
	double vec3 = inertiaTensor[6] * axis.x + inertiaTensor[7] * axis.y + inertiaTensor[8] * axis.z;
	double inertia = axis.x * vec1 + axis.y * vec2 + axis.z * vec3;
	return 1.0 / inertia;
}

void RigidBody::applyImpulseAtPosition(const Vector3D impulse, const Point3D position) {
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

void RigidBody::createReferenceCopies() {
	std::vector<Point3D*> referencePoints;
	for (RigidSurface* surface : surfaces) {
		RigidSurface* referenceCopy = new RigidSurface(*surface);
		surfacesReferenceCopy.push_back(referenceCopy);
		for (Point3D* p : *(referenceCopy->getPoints())) {
			referencePoints.push_back(p);
		}
		referencePoints.push_back(referenceCopy->getNormalVectorPoint());
	}
	Vector3D translation(-centerOfMass.x, -centerOfMass.y, -centerOfMass.z);
	transformation3D::translatePoints(&referencePoints, translation);
}

void RigidBody::findCollisionRadius() {
	double greatestRadiusSquared = 0;
	for (RigidSurface* surface : surfacesReferenceCopy) {
		for (Point3D* p : *(surface->getPoints())) {
			double squaredRadius = p->x * p->x + p->y * p->y + p->z * p->z;
			if (squaredRadius > greatestRadiusSquared)
				greatestRadiusSquared = squaredRadius;
		}
	}
	collisionRadiusSquared = greatestRadiusSquared;
	collisionRadius = sqrt(greatestRadiusSquared);
	
}

/*void RigidBody::recalibrateFromReference() {
	//reference point 1 sits 1 unit above center of mass
	//reference point 2 sits 1 positive x direction to center of mass
	Vector3D reference1Vector(0, -1, 0);
	Vector3D reference2Vector(1, 0, 0);

	Vector3D reference1Pos(centerOfMass, orientationPoint1);

	//find vector and angle to define transformation from default orientation such that the negative y axis aligns with reference point 1 on the current orientation
	double reference1Angle = acos(reference1Vector.dotProduct(reference1Pos));
	Vector3D p1RotationVector = reference1Vector.crossProduct(reference1Pos).getUnitVector();

	//transform p1 so that reference point 2 sits on the xz plane
	Point3D orientationPoint2Relative;
	transformation3D::rotatePointAroundArbitraryAxis(&orientationPoint2, &orientationPoint2Relative, *p1RotationVector, centerOfMass.x, centerOfMass.y, centerOfMass.z, -reference1Angle);

	double reference2Z = orientationPoint2Relative.z - centerOfMass.z;
	double reference2X = orientationPoint2Relative.x - centerOfMass.x;

	//replace point values with values of reference copy, then apply transformations to return to orientation
	for (int i = 0; i < surfacesReferenceCopy.size(); i++) {
		RigidSurface* surfaceReferenceCopy = surfacesReferenceCopy.at(i);
		RigidSurface* surface = surfaces.at(i);
		for (int j = 0; j < surfaceReferenceCopy->getPoints()->size(); j++) {
			Point3D* surfacePoint = surface->getPoints()->at(j);
			Point3D* pointReferenceCopy = surfaceReferenceCopy->getPoints()->at(j);

			surfacePoint->x = pointReferenceCopy->x;
			surfacePoint->y = pointReferenceCopy->y;
			surfacePoint->z = pointReferenceCopy->z;
		}
		Point3D* surfaceNormalPoint = surface->getNormalVectorPoint();
		Point3D* surfaceNormalPointReferenceCopy = surfaceReferenceCopy->getNormalVectorPoint();
		surfaceNormalPoint->x = surfaceNormalPointReferenceCopy->x;
		surfaceNormalPoint->y = surfaceNormalPointReferenceCopy->y;
		surfaceNormalPoint->z = surfaceNormalPointReferenceCopy->z;
	}
	orientationPoint1 = Point3D(0, -1, 0);
	orientationPoint2 = Point3D(0, 0, 1);

	//transform points
	transformation3D::rotatePointsAroundYParralelAxis(&pointsToTransform, -1, 0, 0, reference2X, reference2Z);
	transformation3D::rotatePointsAroundArbitraryAxis(&pointsToTransform, *p1RotationVector, 0, 0, 0, reference1Angle, 0, 0);
	Vector3D translation(centerOfMass.x, centerOfMass.y, centerOfMass.z);
	transformation3D::translatePoints(&pointsToTransform, translation);
}*/

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

Point3D* RigidBody::getCenterOfMass() {
	return &centerOfMass;
}

bool RigidBody::getPointInsideBody(const Point3D point, const std::vector<Point3D*> vectorPositions, const std::vector<Vector3D*> normalVectors) {
	for (int i = 0; i < vectorPositions.size(); i++) {
		Vector3D* normalVector = normalVectors.at(i);
		Point3D* vectorPos = vectorPositions.at(i);
		Vector3D posToPoint(*vectorPos, point);
		if (posToPoint.dotProduct(*normalVector) > 0)
			return false;
	}
	return true;
}

bool RigidBody::getPointInsideBody(const Point3D point) {
	for (RigidSurface* surface : surfaces) {
		Vector3D normalVector(*(surface->getPoints()->at(0)), *(surface->getNormalVectorPoint()));
		Vector3D posToPoint(*(surface->getPoints()->at(0)), point);
		if (normalVector.dotProduct(posToPoint) > 0)
			return false;
	}
	return true;
}

bool RigidBody::bodiesInCollisionRange(RigidBody& body) {
	double xDist = abs(centerOfMass.x - body.getCenterOfMass()->x);
	double yDist = abs(centerOfMass.y - body.getCenterOfMass()->y);
	double zDist = abs(centerOfMass.z - body.getCenterOfMass()->z);

	//if (xDist > collisionRadius + body.getCollisionRadius() || yDist > collisionRadius + body.getCollisionRadius() || zDist > collisionRadius + body.getCollisionRadius())
	//	return false;
	if (xDist * xDist + yDist * yDist + zDist * zDist > (collisionRadius + body.getCollisionRadius()) * (collisionRadius + body.getCollisionRadius()))
		return false;
	return true;
}

std::vector<RigidSurface*>* RigidBody::getSurfaces() {
	return &surfaces;
}

Vector3D RigidBody::getVelocityOfPointDueToAngularVelocity(const Point3D point) const {
	Vector3D centerOfMassToPoint(centerOfMass, point);
	return angularVelocity.crossProduct(centerOfMassToPoint);
}

Vector3D RigidBody::getVelocityOfPoint(const Point3D point) const {
	return getVelocityOfPointDueToAngularVelocity(point).add(velocity);
}

Vector3D RigidBody::getVelocity() {
	return velocity;
}

double RigidBody::getRadialDistanceOfPoint(const Point3D point) {
	double dx = centerOfMass.x - point.x;
	double dy = centerOfMass.y - point.y;
	double dz = centerOfMass.z - point.z;
	return dx * dx + dy * dy + dz * dz;
}

bool RigidBody::verifyCollisionPointNotExiting(const RigidBody body, const Vector3D normalVector, const Point3D p) {
	Vector3D vPThisBody = getVelocityOfPoint(p);
	Vector3D vPOtherBody = body.getVelocityOfPoint(p);
	Vector3D vPRelative = vPThisBody.sub(vPOtherBody);
	return vPRelative.dotProduct(normalVector) < 0;
}

RigidBody::ColPointInfo::ColPointInfo(double x, double y, double z, Vector3D* colNormVector, double penDepth) {
	this->point.x = x;
	this->point.y = y;
	this->point.z = z;
	this->colNormVector.x = colNormVector->x;
	this->colNormVector.y = colNormVector->y;
	this->colNormVector.z = colNormVector->z;
	this->penDepth = penDepth;
}


void RigidBody::findCollisionInformationAsCollider(std::vector<ColPointInfo*>* colOutputs, RigidBody& body, double timestep) {
	
	for (Point3D* p : colPoints) {
		//check if point is within collision radius to ignore points that cant possibly be inside
		double distToBodySquared = (p->x - body.getCenterOfMass()->x) * (p->x - body.getCenterOfMass()->x) +
			(p->y - body.getCenterOfMass()->y) * (p->y - body.getCenterOfMass()->y) +
			(p->z - body.getCenterOfMass()->z) * (p->z - body.getCenterOfMass()->z);
		if (distToBodySquared > body.getCollisionRadiusSquared())
			continue;
		if (!body.getPointInsideBody(*p)) {
			continue;
		}
		//point can potentially be a collision point, now check to see if the ray from the center of mass to the point
		//intersects any surfaces on body
		RigidSurface* nearestPenetratedSurface = nullptr;
		double nearestPenetratingSurfaceDistSquared;
		for (RigidSurface* potentialCollisionSurface : *(body.getSurfaces())) {
			Point3D* surfaceP1 = potentialCollisionSurface->getPoints()->at(0);
			//normal vector of plane
			Vector3D normalVector(*surfaceP1, *(potentialCollisionSurface->getNormalVectorPoint()));
			//check that p and center of mass are on opposite sides of the plane defined by surface
			Vector3D p1P(*surfaceP1, *p);
			Vector3D p1COM(*surfaceP1, centerOfMass);
			//if both dot products are positive or negative they are on the same side of the plane
			if (p1P.dotProduct(normalVector) * p1COM.dotProduct(normalVector) >= 0)
				continue;
			//useing parametrics to fine the intersection between pToCOM and the plane
			//n . (pToP1 - pToCom * t) = 0
			// >>>
			// t = n . pToP1 / n . pToCom
			Vector3D pToCOM(*p, centerOfMass);
			Vector3D pToP1(*p, *surfaceP1);
			double t = normalVector.dotProduct(pToP1) / normalVector.dotProduct(pToCOM);
			pToCOM = pToCOM.multiply(t); //point p + pToCOM now gives intersection with plane
			Point3D planeIntersection(p->x + pToCOM.x, p->y + pToCOM.y, p->z + pToCOM.z);
			//check that the intersection point lies in the actual surface defined
			Vector3D planeAxisI(*surfaceP1, planeIntersection);
			double p1ToIntersectDistSquared = planeAxisI.getMagnitudeSquared();
			planeAxisI = planeAxisI.getUnitVector();
			Vector3D planeAxisJ = planeAxisI.crossProduct(normalVector);
			//axis I and J define unit vectors on the plane defined by surface. if intersection point is within
			//the surface, it now sits on the i axis, and if its within there should be one edge on the polygon that intersects
			// the i axis, that should be behind the intersection point in its i value
			bool pointInSurface = false;
			for (int i = 1; i < potentialCollisionSurface->getPoints()->size() - 1; i++) {
				Vector3D point1(*surfaceP1, *(potentialCollisionSurface->getPoints()->at(i)));
				Vector3D point2(*surfaceP1, *(potentialCollisionSurface->getPoints()->at(i + 1)));
				double p1J = point1.dotProduct(planeAxisJ);
				double p2J = point2.dotProduct(planeAxisJ);
				//check that p1 and p2 cross the i axis
				if (p1J * p2J > 0)
					continue;
				double p1I = point1.dotProduct(planeAxisI);
				double p2I = point2.dotProduct(planeAxisI);
				if (p1J == 0) {
					pointInSurface = p1I * p1I > p1ToIntersectDistSquared;
					break;
				}
				if (p2J == 0) {
					pointInSurface = p2I * p2I > p1ToIntersectDistSquared;
					break;
				}
				double iIntercept = (p2I == p1I) ? p2I : p1I - p1J * (p2I - p1I) / (p2J - p1J);
				pointInSurface = iIntercept * iIntercept > p1ToIntersectDistSquared;
				break;
			}
			if (!pointInSurface)
				continue;

			double distX = (planeIntersection.x - centerOfMass.x) * (planeIntersection.x - centerOfMass.x);
			double distY = (planeIntersection.y - centerOfMass.y) * (planeIntersection.y - centerOfMass.y);
			double distZ = (planeIntersection.z - centerOfMass.z) * (planeIntersection.z - centerOfMass.z);
			double distFromCenterOfMass = distX + distY + distZ;
			if (nearestPenetratedSurface == nullptr || distFromCenterOfMass < nearestPenetratingSurfaceDistSquared) {
				nearestPenetratedSurface = potentialCollisionSurface;
				nearestPenetratingSurfaceDistSquared = distFromCenterOfMass;
			}
		}
		if (nearestPenetratedSurface == nullptr) {
			continue;
		}

		Point3D* surfaceP1 = nearestPenetratedSurface->getPoints()->at(0);
		Vector3D p1ToP(*surfaceP1, *p);
		Vector3D normalVector(*surfaceP1, *(nearestPenetratedSurface->getNormalVectorPoint()));
		
		bool pointNotExiting = verifyCollisionPointNotExiting(body, normalVector, *p);
		if (!pointNotExiting) {
			continue;
		}
		double penetrationDepth = abs(p1ToP.dotProduct(normalVector));
		bool deepestPenPoint = colOutputs->size() == 0 || penetrationDepth > colOutputs->at(0)->penDepth;

		ColPointInfo* info = new ColPointInfo(p->x, p->y, p->z, &normalVector, penetrationDepth);
		colOutputs->push_back(info);
	}
	
	//check for edge collisions
	if (!fixed) {

		ColPointInfo* edgeCol = nullptr;

		for (Edge* edge : colEdges) {
			Point3D* p1 = edge->p1;
			Point3D* p2 = edge->p2;

			if (body.getPointInsideBody(*p1) || body.getPointInsideBody(*p2))
				continue;

			Vector3D p1p2(*p1, *p2);
			Vector3D p1p2Unit = p1p2.multiply(edge->inverseMagnitude);
			Vector3D p1Com(*p1, *(body.getCenterOfMass()));
			Vector3D parralelComp = p1p2Unit.multiply(p1Com.dotProduct(p1p2Unit));
			Vector3D distComp = p1Com.sub(parralelComp);
			if (distComp.getMagnitudeSquared() > body.getCollisionRadiusSquared())
				continue;
			double leastDepthPenetration = -1;
			for (RigidSurface* potColSurface : *(body.getSurfaces())) {
				Point3D* surfP1 = potColSurface->getPoints()->at(0);
				Vector3D normalVector(*surfP1, *(potColSurface->getNormalVectorPoint()));
				Vector3D p1Sp1(*p1, *surfP1);
				Vector3D p2Sp1(*p2, *surfP1);

				if (p1Sp1.dotProduct(normalVector) * p2Sp1.dotProduct(normalVector) > 0)
					continue;

				Vector3D perpComp = p1p2Unit.multiply(p1p2Unit.dotProduct(p1Sp1));
				Vector3D axisJ = p1Sp1.sub(perpComp);
				Vector3D axisI;
				axisI = axisJ.crossProduct(p1p2Unit);
				int pointsAbove = 1;
				int pointsBelow = 0;
				for (int i = 1; i < potColSurface->getPoints()->size() - 1; i++) {
					Vector3D lineP1(*p1, *(potColSurface->getPoints()->at(i)));
					Vector3D lineP2(*p1, *(potColSurface->getPoints()->at(i + 1)));

					double p1I = axisI.dotProduct(lineP1);
					double p2I = axisI.dotProduct(lineP2);
					//i coords dont intersect j axis
					if (p1I * p2I > 0)
						continue;
					double p1J = axisJ.dotProduct(lineP1);
					double p2J = axisJ.dotProduct(lineP2);
					double jInt = p1J - p1I * (p2J - p1J) / (p2I - p1I);
					if (jInt > 0) {
						pointsAbove++;
					}
					else {
						pointsBelow++;
					}
				}
				if (pointsAbove % 2 == 0 || pointsBelow % 2 == 0)
					continue;
				for (int i = 0; i < potColSurface->getPoints()->size(); i++) {
					Point3D* lp1 = potColSurface->getPoints()->at(i);
					Point3D* lp2 = (i == potColSurface->getPoints()->size() - 1) ? potColSurface->getPoints()->at(0) : potColSurface->getPoints()->at(i + 1);
					Vector3D lp1lp2(*lp1, *lp2);
					Vector3D perpDirection = lp1lp2.crossProduct(p1p2).getUnitVector();
					Vector3D p1Lp1(*p1, *lp1);
					double penDepth = p1Lp1.dotProduct(perpDirection);
					if (penDepth < 0) {
						perpDirection = perpDirection.multiply(-1);
						penDepth *= -1;
					}
					//choice of 5 arbitrary, tries to avoid improper collisions
					if ((leastDepthPenetration != -1 && penDepth >= leastDepthPenetration) || penDepth > 1.5)
						continue;
					double lp1I = p1Lp1.dotProduct(p1p2Unit);
					Vector3D p1Lp2(*p1, *lp2);
					double lp2I = p1Lp2.dotProduct(p1p2Unit);
					Vector3D vertAxis = perpDirection.crossProduct(p1p2Unit);
					double lp1J = vertAxis.dotProduct(p1Lp1);
					double lp2J = vertAxis.dotProduct(p1Lp2);
					double iInt = lp1I - lp1J * (lp2I - lp1I) / (lp2J - lp1J);
					if (iInt < 0 || iInt > p1p2.getMagnitude())
						continue;
					Vector3D p1ToCol = p1p2Unit.multiply(iInt);
					Vector3D perpComp = perpDirection.multiply(penDepth);
					p1ToCol = p1ToCol.add(perpComp);

					Point3D colPoint(p1->x + p1ToCol.x, p1->y + p1ToCol.y, p1->z + p1ToCol.z);
					if (!verifyCollisionPointNotExiting(body, perpDirection, colPoint)) {
						continue;
					}

					Vector3D colNV(perpDirection.x, perpDirection.y, perpDirection.z);
					leastDepthPenetration = penDepth;
					
					ColPointInfo* info = new ColPointInfo(colPoint.x, colPoint.y, colPoint.z, &colNV, penDepth);
					if (edgeCol != nullptr)
						delete edgeCol;
					edgeCol = info;
				}
			}
		}

		if (edgeCol != nullptr)
			colOutputs->push_back(edgeCol);

	}
	//sorting the colPoints with bubble sort, deepest penning points are resolved first.
	if (colOutputs->size() > 1) {
		int k = 1;
		bool swp = false;
		do {
			bool swp = false;
			for (int i = 0; i < colOutputs->size() - k; i++) {
				if (colOutputs->at(i + 1)->penDepth > colOutputs->at(i)->penDepth) {
					ColPointInfo* tmp = colOutputs->at(i + 1);
					colOutputs->at(i + 1) = colOutputs->at(i);
					colOutputs->at(i) = tmp;
					swp = true;
				}
			}
			k--;
		} while (swp);
	}
}
