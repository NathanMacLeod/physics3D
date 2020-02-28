#include "RigidBody.h"
#include "transformation3D.h"

const double DEFAULT_PARTICLE_DENSITY = 0.25;

RigidBody::RigidBody(const std::vector<RigidSurface*>& surfaces, double density, double friction, double restitution, bool fixed) {
	this->fixed = fixed;
	this->friction = friction;
	this->restitution = restitution;
	this->surfaces = surfaces;
	for (RigidSurface* surface : surfaces) {
		for (Point3D* p : *(surface->getPoints())) {
			pointsToTransform.push_back(p);
		}
		pointsToTransform.push_back(surface->getNormalVectorPoint());
	}

	findBodyMassAndInertia(density, DEFAULT_PARTICLE_DENSITY);
	createReferenceCopies();
	findCollisionRadius();

	//if (!fixed)
	//	velocity.x = 5;

	orientationPoint1 = Point3D(centerOfMass.x, centerOfMass.y - 1, centerOfMass.z);
	orientationPoint2 = Point3D(centerOfMass.x + 1, centerOfMass.y, centerOfMass.z);

	pointsToTransform.push_back(&orientationPoint1);
	pointsToTransform.push_back(&orientationPoint2);
	pointsToTransform.push_back(&centerOfMass);
}

void RigidBody::acclerateLineraly(const Vector3D changeInVelocity) {
	//std::cout << velocity.x << " " << velocity.y << " " << velocity.z << "\n";
	if (fixed)
		return;
	velocity.add(changeInVelocity, &velocity);
}

void RigidBody::translate(const Vector3D translation) {
	transformation3D::translatePoints(&pointsToTransform, translation);
}

void RigidBody::moveInTime(double time) {
	if (fixed)
		return;
	Vector3D translation;
	Vector3D rotationAxis;
	double rotationMagnitude = angularVelocity.getMagnitude() * time;
	angularVelocity.getUnitVector(&rotationAxis);
	velocity.multiply(time, &translation);
	transformation3D::translatePoints(&pointsToTransform, translation);
	if(rotationMagnitude != 0)
		transformation3D::rotatePointsAroundArbitraryAxis(&pointsToTransform, rotationAxis, centerOfMass.x, centerOfMass.y, centerOfMass.z, rotationMagnitude);
}

bool RigidBody::getFixed() {
	return fixed;
}


//todo: do this in not dumb way
void RigidBody::findBodyMassAndInertia(double particleMass, double particleSpacing) {
	double lowestX = 0;
	double greatestX = 0;
	double lowestY = 0;
	double greatestY = 0;
	double lowestZ = 0;
	double greatestZ = 0;
	bool firstPass = true;
	for (RigidSurface* surface : surfaces) {
		for (Point3D* p : *(surface->getPoints())) {
			if (firstPass || p->x < lowestX)
				lowestX = p->x;
			if (firstPass || p->x > greatestX)
				greatestX = p->x;

			if (firstPass || p->y < lowestY)
				lowestY = p->y;
			if (firstPass || p->y > greatestY)
				greatestY = p->y;

			if (firstPass || p->z < lowestZ)
				lowestZ = p->z;
			if (firstPass || p->z > greatestZ)
				greatestZ= p->z;

			firstPass = false;
		}
	}
	int numPoints = 0;
	std::vector<Point3D**> integrationLineBoundries;
	//iterate along planes parralel to yz plane, find surfaces that intersect plane
	//next iterate along lines parralel to z axis, find where the line enters and exits the body
	//finally iterate along the line, create point masses
	Vector3D yzPlaneNormalVector(1, 0, 0);
	Vector3D xzPlaneNormalVector(0, 1, 0);

	std::vector<RigidSurface*> intersectingPlanes;
	std::vector<double*> planesYRange;
	int nplanes = 0;
	int nlines = 0;
	int npoints = 0;
	for(double x = lowestX; x < greatestX; x += particleSpacing) {
		nplanes++;
		intersectingPlanes.clear();
		for (double* bounds : planesYRange) {
			delete[] bounds;
		}
		planesYRange.clear();
		for (RigidSurface* surface : surfaces) {
			double bound1;
			double bound2;
			bool intersectFound = false;
			bool intersect2Found = false;
			for (int i = 0; i < surface->getPoints()->size(); i++) {
				Point3D* p1 = surface->getPoints()->at(i);
				Point3D* p2 = ((i == surface->getPoints()->size() - 1)? surface->getPoints()->at(0) : surface->getPoints()->at(i + 1));
				if ((p1->x > x && p2->x < x) || (p1->x < x && p2->x > x)) {
					double intersectYVal = p1->y - (p1->x - x) * (p2->y - p1->y) / (p2->x - p1->x);
					if (intersectFound) {
						bound2 = intersectYVal;
						intersect2Found = true;
						break;
					}
					else {
						intersectFound = true;
						bound1 = intersectYVal;
					}
				}
			}
			if (!intersectFound || !intersect2Found)
				continue;

			if (bound1 == bound2)
				continue;

			double* bounds = new double[2];
			if (bound1 > bound2) {
				bounds[0] = bound2;
				bounds[1] = bound1;
			}
			else {
				bounds[0] = bound1;
				bounds[1] = bound2;
			}
			//std::cout << "   Y Bounds:" << bounds[0] << " " << bounds[1] << "\n";
			intersectingPlanes.push_back(surface);
			planesYRange.push_back(bounds);
		}
		for (double y = lowestY; y < greatestY; y += particleSpacing) {
			double zMin = 0;
			double zMax = 0;
			bool firstMatch = true;
			for (int i = 0; i < planesYRange.size(); i++) {
				if (y < planesYRange.at(i)[0] || y > planesYRange.at(i)[1])
					continue;
				Point3D* p1 = intersectingPlanes.at(i)->getPoints()->at(0);
				Vector3D planeNV(*p1, *(intersectingPlanes.at(i)->getNormalVectorPoint()));
				//std::cout << "      nV:" << planeNV.x << ", " << planeNV.y << ", " << planeNV.z << "\n";
				double zIntersect = p1->z - (planeNV.x * (x - p1->x) + planeNV.y * (y - p1->y)) / planeNV.z;
				if (firstMatch) {
					firstMatch = false;
					zMin = zIntersect;
					zMax = zIntersect;
					continue;
				}
				if (zMin > zIntersect)
					zMin = zIntersect;
				if (zMax < zIntersect)
					zMax = zIntersect;
			}
			nlines++;
			
			//std::cout << "      zMin:" << zMin << " zMax:" << zMax << "\n";
			Point3D* min = new Point3D(x, y, zMin);
			Point3D* max = new Point3D(x, y, zMax);
			Point3D** line = new Point3D*[2]{ min, max };
			integrationLineBoundries.push_back(line);
			for (double z = zMin; z < zMax; z += particleSpacing) {
				numPoints++;
				centerOfMass.x += x;
				centerOfMass.y += y;
				centerOfMass.z += z;
				npoints++;
			}
		}
		//std::cout << "Num for Xcord " << x << ": " << numForPlane << "\n";
	}
	//std::cout << nplanes << " " << nlines << " " << npoints << "\n";
	centerOfMass.x /= numPoints;
	centerOfMass.y /= numPoints;
	centerOfMass.z /= numPoints;
	mass = particleMass * numPoints;
	inverseMass = 1.0 / mass;
	inertiaTensor = new double[9]{ 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	for (Point3D** line : integrationLineBoundries) {
		Point3D* zMin = line[0];
		Point3D* zMax = line[1];
		double rX = zMin->x - centerOfMass.x;
		double rY = zMin->y - centerOfMass.y;
		double rXSquared = rX * rX;
		double rYSquared = rY * rY;
		//std::cout << zMin->z << " " << zMax->z << "\n";
		for (double z = zMin->z; z < zMax->z; z += particleSpacing) {
			double rZ = z - centerOfMass.z;
			double rZSquared = rZ * rZ;
			inertiaTensor[0] += rYSquared + rZSquared;
			inertiaTensor[1] += -rX * rY;
			inertiaTensor[2] += -rX * rZ;
			inertiaTensor[3] += -rY * rX;
			inertiaTensor[4] += rXSquared + rZSquared;
			inertiaTensor[5] += -rY * rZ;
			inertiaTensor[6] += -rZ * rX;
			inertiaTensor[7] += -rZ * rY;
			inertiaTensor[8] += rXSquared + rYSquared;
		}
		delete zMin;
		delete zMax;
		delete[] line;
	}
	for (int i = 0; i < 9; i++) {
		inertiaTensor[i] *= mass / numPoints;
	}
}

Vector3D* RigidBody::findVectorRelativeToBodyFrame(const Vector3D vector, Vector3D* output) {
	Vector3D orientationVector1(centerOfMass, orientationPoint1);
	Vector3D orientationVector2(centerOfMass, orientationPoint2);
	Vector3D orientationVector3;
	orientationVector1.crossProduct(orientationVector2, &orientationVector3);

	double negYVal = orientationVector1.dotProduct(vector);
	double xVal = orientationVector2.dotProduct(vector);
	double zVal = orientationVector3.dotProduct(vector);

	output->x = xVal;
	output->y = -negYVal;
	output->z = zVal;
	return output;
}

double RigidBody::findInverseInertiaOfAxis(const Vector3D inputAxis) {
	if (fixed)
		return 0;
	Vector3D axis;
	findVectorRelativeToBodyFrame(inputAxis, &axis);
	axis.getUnitVector(&axis);
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
	Vector3D deltaW;
	comToPoint.crossProduct(impulse, &deltaW);
	deltaW.multiply(findInverseInertiaOfAxis(deltaW), &deltaW);
	angularVelocity.add(deltaW, &angularVelocity);
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

void RigidBody::recalibrateFromReference() {
	//reference point 1 sits 1 unit above center of mass
	//reference point 2 sits 1 positive x direction to center of mass
	Vector3D reference1Vector(0, -1, 0);
	Vector3D reference2Vector(1, 0, 0);

	Vector3D reference1Pos(centerOfMass, orientationPoint1);

	//find vector and angle to define transformation from default orientation such that the negative y axis aligns with reference point 1 on the current orientation
	Vector3D a;
	double reference1Angle = acos(reference1Vector.dotProduct(reference1Pos));
	Vector3D* p1RotationVector = reference1Vector.crossProduct(reference1Pos, &a);
	p1RotationVector = p1RotationVector->getUnitVector(&a);

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

Vector3D* RigidBody::getAngularVelocity() {
	return &angularVelocity;
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

	if (xDist > collisionRadius + body.getCollisionRadius() || yDist > collisionRadius + body.getCollisionRadius() || zDist > collisionRadius + body.getCollisionRadius())
		return false;
	if (xDist * xDist + yDist * yDist + zDist * zDist > pow(collisionRadius + body.getCollisionRadius(), 2))
		return false;
	return true;
}

std::vector<RigidSurface*>* RigidBody::getSurfaces() {
	return &surfaces;
}

Vector3D* RigidBody::getVelocityOfPointDueToAngularVelocity(const Point3D point, Vector3D* output) const {
	Vector3D centerOfMassToPoint(centerOfMass, point);
	angularVelocity.crossProduct(centerOfMassToPoint, output);
	return output;
}

Vector3D* RigidBody::getVelocityOfPoint(const Point3D point, Vector3D* output) const {
	getVelocityOfPointDueToAngularVelocity(point, output);
	velocity.add(*output, output);
	return output;
}

double RigidBody::getRadialDistanceOfPoint(const Point3D point) {
	double dx = centerOfMass.x - point.x;
	double dy = centerOfMass.y - point.y;
	double dz = centerOfMass.z - point.z;
	return dx * dx + dy * dy + dz * dz;
}

bool RigidBody::verifyCollisionPointNotExiting(const RigidBody body, const Vector3D normalVector, const Point3D p) {
	Vector3D vPThisBody;
	Vector3D vPOtherBody;
	getVelocityOfPoint(p, &vPThisBody);
	body.getVelocityOfPoint(p, &vPOtherBody);
	Vector3D vPRelative;
	vPThisBody.sub(vPOtherBody, &vPRelative);
	return vPRelative.dotProduct(normalVector) < 0;
}

RigidBody::ColPointInfo::ColPointInfo(Point3D* point, Vector3D* colNormVector, bool edgeCollision, double penDepth) {
	this->point = point;
	this->colNormVector = colNormVector;
	this->edgeCollision = edgeCollision;
	this->penDepth = penDepth;
}

RigidBody::ColPointInfo::~ColPointInfo() {
	delete colNormVector;
	if (edgeCollision) {
		delete point;
	}
}

void RigidBody::findCollisionInformationAsCollider(std::vector<ColPointInfo*>* colOutputs, RigidBody& body) {
	bool pointInside = false;
	for (RigidSurface* surface : surfaces) {
		for (Point3D* p : *(surface->getPoints())) {
			//check if point is within collision radius to ignore points that cant possibly be inside
			double distToBodySquared = pow(p->x - body.getCenterOfMass()->x, 2) + pow(p->y - body.getCenterOfMass()->y, 2) + pow(p->z - body.getCenterOfMass()->z, 2);
			if (distToBodySquared > body.getCollisionRadiusSquared())
				continue;
			if (!body.getPointInsideBody(*p)) {
				continue;
			}
			pointInside = true;

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
				pToCOM.multiply(t, &pToCOM); //point p + pToCOM now gives intersection with plane
				Point3D planeIntersection(p->x + pToCOM.x, p->y + pToCOM.y, p->z + pToCOM.z);
				//check that the intersection point lies in the actual surface defined
				Vector3D planeAxisI(*surfaceP1, planeIntersection);
				double p1ToIntersectDistSquared = planeAxisI.getMagnitudeSquared();
				planeAxisI.getUnitVector(&planeAxisI);
				Vector3D planeAxisJ;
				planeAxisI.crossProduct(normalVector, &planeAxisJ);
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
			Vector3D* normalVector = new Vector3D(*surfaceP1, *(nearestPenetratedSurface->getNormalVectorPoint()));

			bool pointNotExiting = verifyCollisionPointNotExiting(body, *normalVector, *p);

			double penetrationDepth = abs(p1ToP.dotProduct(*normalVector));

			bool deepestPenPoint = colOutputs->size() == 0 || penetrationDepth > colOutputs->at(0)->penDepth;

			ColPointInfo* info = new ColPointInfo(p, normalVector, false, penetrationDepth);

			if (deepestPenPoint) {
				if (colOutputs->size() != 0) {
					colOutputs->at(0) = info;
				}
				else {
					colOutputs->push_back(info);
				}
			}
			colOutputs->push_back(info);
		}
	}
	if (getInverseMass() != 0 && !pointInside) {
		//check for edge collisions
		for (RigidSurface* surface : surfaces) {
			for (int i = 0; i < surface->getPoints()->size(); i++) {
				Point3D* p1 = surface->getPoints()->at(i);
				Point3D* p2 = (i == surface->getPoints()->size() - 1)? surface->getPoints()->at(0) : surface->getPoints()->at(i + 1);
				Vector3D p1p2(*p1, *p2);
				Vector3D p1p2Unit;
				p1p2.multiply(surface->getInverseSegmentMagnitude(i), &p1p2Unit);
				Vector3D p1Com(*p1, *(body.getCenterOfMass()));
				Vector3D parralelComp;
				p1p2Unit.multiply(p1Com.dotProduct(p1p2Unit), &parralelComp);
				Vector3D distComp;
				p1Com.sub(parralelComp, &distComp);
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

					Vector3D axisJ;
					Vector3D perpComp;
					p1p2Unit.multiply(p1p2Unit.dotProduct(p1Sp1), &perpComp);
					p1Sp1.sub(perpComp, &axisJ);
					Vector3D axisI;
					axisJ.crossProduct(p1p2Unit, &axisI);
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
						Vector3D perpDirection;
						lp1lp2.crossProduct(p1p2, &perpDirection);
						perpDirection.getUnitVector(&perpDirection);
						Vector3D p1Lp1(*p1, *lp1);
						double penDepth = p1Lp1.dotProduct(perpDirection);
						if (penDepth < 0) {
							perpDirection.multiply(-1, &perpDirection);
							penDepth *= -1;
						}
						//choice of 1.5 arbitrary, tries to avoid improper collisions
						if ((leastDepthPenetration != -1 && penDepth >= leastDepthPenetration) || penDepth > 1.5)
							continue;
						double lp1I = p1Lp1.dotProduct(p1p2Unit);
						Vector3D p1Lp2(*p1, *lp2);
						double lp2I = p1Lp2.dotProduct(p1p2Unit);
						Vector3D vertAxis;
						perpDirection.crossProduct(p1p2Unit, &vertAxis);
						double lp1J = vertAxis.dotProduct(p1Lp1);
						double lp2J = vertAxis.dotProduct(p1Lp2);
						double iInt = lp1I - lp1J * (lp2I - lp1I) / (lp2J - lp1J);
						if (iInt < 0 || iInt > p1p2.getMagnitude())
							continue;
						Vector3D p1ToCol;
						p1p2Unit.multiply(iInt, &p1ToCol);
						Vector3D perpComp;
						perpDirection.multiply(penDepth, &perpComp);
						p1ToCol.add(perpComp, &p1ToCol);

						Point3D* colPoint = new Point3D(p1->x + p1ToCol.x, p1->y + p1ToCol.y, p1->z + p1ToCol.z);
						if (!verifyCollisionPointNotExiting(body, perpDirection, *colPoint))
							continue;
						leastDepthPenetration = penDepth;
						Vector3D* colNV = new Vector3D(perpDirection.x, perpDirection.y, perpDirection.z);
						ColPointInfo* info = new ColPointInfo(colPoint, colNV, true, penDepth);
						colOutputs->push_back(info);
						return;
					}
				}
			}
		}
	}

	//sorting the colPoints with bubble sort, deepest penning points are resolved first.
	if (colOutputs->size() > 1) {
		int k = 1;
		bool swp = false;
		do {
			bool swp = false;
			for (int i = 1; i < colOutputs->size() - k; i++) {
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
