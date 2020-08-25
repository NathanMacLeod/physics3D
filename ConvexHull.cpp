#include "ConvexHull.h"

ConvexHull::ConvexHull(const std::vector<RigidSurface*>& surfaces, double density) {
	this->surfaces = surfaces;	

	findBodyMassAndInertia(density);
	findColPointsEdges();
	findCollisionRadius();

}

ConvexHull::Edge::Edge(Point3D* p1, Point3D* p2) {
	this->p1 = p1;
	this->p2 = p2;
	inverseMagnitude = 1.0 / Vector3D(*p1, *p2).getMagnitude();
}

void ConvexHull::findColPointsEdges() {
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

//Fast and Accurate Computation of Polyhedral Mass Properties (Brian Miritch)
void ConvexHull::findBodyMassAndInertia(double density) {
	double v = 0, vX = 0, vY = 0, vZ = 0, vXSqrd = 0, vYSqrd = 0, vZSqrd = 0;
	double vXY = 0, vXZ = 0, vYZ = 0;

	for (RigidSurface* s : surfaces) {
		double sXSqrd, sYSqrd, sZSqrd, sXCbd, sYCbd, sZCbd;
		double sXSqrdY, sXSqrdZ, sYSqrdZ, sX, u;

		double* sA, * sB, * sC, * sASqrd, * sBSqrd, * sCSqrd;
		double* sACbd, * sBCbd, * sCCbd, * sASqrdB, * sBSqrdC, * sASqrdC, * sBSqrdA, * sCSqrdA, * sCSqrdB;

		double f = 0, fA = 0, fB = 0, fAB = 0, fASqrd = 0, fBSqrd = 0, fASqrdB = 0, fBSqrdA = 0;
		double fACbd = 0, fBCbd = 0;

		enum axis A; //horz axis
		enum axis B; //vert axis
		enum axis C; //normal axis

		double nA;
		double nB;
		double nC;
		double invC;

		Vector3D normV = s->getUnitNorm();

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
			fBSqrd += -(1 / 3.0) * a21 * (b1 * b1 * b1 + 3 * b1 * b1 * b21 / 2.0 + b1 * b21 * b21 + b21 * b21 * b21 / 4.0);
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

	inertiaTensor = new double[9]{ 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	inertiaTensor[0] = density * (vYSqrd + vZSqrd - v * (centerOfMass.y * centerOfMass.y + centerOfMass.z * centerOfMass.z));
	inertiaTensor[1] = -density * (vXY - v * centerOfMass.x * centerOfMass.y);
	inertiaTensor[2] = -density * (vXZ - v * centerOfMass.x * centerOfMass.z);
	inertiaTensor[4] = density * (vXSqrd + vZSqrd - v * (centerOfMass.x * centerOfMass.x + centerOfMass.z * centerOfMass.z));
	inertiaTensor[5] = -density * (vYZ - v * centerOfMass.y * centerOfMass.z);
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

void ConvexHull::findCollisionRadius() {
	double greatestRadiusSquared = 0;
	for (Point3D* p : colPoints) {
		double squaredRadius = (centerOfMass.x - p->x) * (centerOfMass.x - p->x) + (centerOfMass.y - p->y) * (centerOfMass.y - p->y)
			+ (centerOfMass.z - p->z) * (centerOfMass.z - p->z);
		if (squaredRadius > greatestRadiusSquared)
			greatestRadiusSquared = squaredRadius;
	}

	collisionRadiusSquared = greatestRadiusSquared;
	collisionRadius = sqrt(greatestRadiusSquared);

}

Point3D ConvexHull::getCenterOfMass() {
	return centerOfMass;
}

double* ConvexHull::getInertia() {
	return inertiaTensor;
}

double ConvexHull::getMass() {
	return mass;
}

std::vector<Point3D*>* ConvexHull::getColPoints() {
	return &colPoints;
}

bool ConvexHull::getPointInsideBody(const Point3D point) {
	for (RigidSurface* surface : surfaces) {
		Vector3D normalVector = surface->getUnitNorm();
		Vector3D posToPoint(*(surface->getPoints()->at(0)), point);
		if (normalVector.dotProduct(posToPoint) > 0)
			return false;
	}
	return true;
}

double ConvexHull::getCollisionRadius() {
	return collisionRadius;
}

bool ConvexHull::hullsInCollisionRange(ConvexHull& hull) {
	double xDist = centerOfMass.x - hull.centerOfMass.x;
	double yDist = centerOfMass.y - hull.centerOfMass.y;
	double zDist = centerOfMass.z - hull.centerOfMass.z;

	double sqrDist = xDist * xDist + yDist * yDist + zDist * zDist;
	double colDist = (collisionRadius + hull.getCollisionRadius()) * (collisionRadius + hull.getCollisionRadius());
	return sqrDist <= colDist;
}

ConvexHull::ColPointInfo::ColPointInfo(Point3D p, double penDepth) {
	this->point = p;
	this->penDepth = penDepth;
}

void ConvexHull::findMaxMin(Vector3D n, double* max, double* min, Point3D* maxP, Point3D* minP) {
	Point3D zero(0, 0, 0);

	double nMax, nMin;
	bool init = true;
	Point3D* minPoint = nullptr;
	Point3D* maxPoint = nullptr;

	for (Point3D* p : colPoints) {
		double nDotVal = Vector3D(zero, *p).dotProduct(n);
		if (init) {
			nMin = nDotVal;
			nMax = nDotVal;
			minPoint = p;
			maxPoint = p;
			init = false;
		}
		else {
			if (nDotVal < nMin) {
				nMin = nDotVal;
				minPoint = p;
			}
			else if (nDotVal > nMax) {
				nMax = nDotVal;
				maxPoint = p;
			}
		}
	}

	*max = nMax;
	*min = nMin;
	if (maxP != nullptr) {
		*maxP = *maxPoint;
	}
	if (minP != nullptr) {
		*minP = *minPoint;
	}
}

bool ConvexHull::SATColliderDetect(ConvexHull* potCollider, std::vector<ColPointInfo>* colSupPoints, Point3D* collisionPoint, Vector3D* nVect, double* colDepth, bool* separatingAxis) {
	std::vector<Vector3D> testedDirs;

	Point3D colPoint;
	Vector3D colVector;
	double lowestColDepth = -1;
	double winningCollideeMax;

	for (RigidSurface* s : surfaces) {
		Vector3D n = s->getUnitNorm();

		bool alreadyTested = false;
		for (Vector3D dir : testedDirs) {
			if (abs(n.dotProduct(dir)) > 0.99) {
				alreadyTested = true;
			}
		}
		if (alreadyTested) {
			continue;
		}
		testedDirs.push_back(n);

		double colliderMax, colliderMin, collideeMax, collideeMin;
		

		Point3D minPoint;
		Point3D maxPoint;

		findMaxMin(n, &collideeMax, &collideeMin, nullptr, nullptr);
		potCollider->findMaxMin(n, &colliderMax, &colliderMin, &maxPoint, &minPoint);

		if (!(colliderMin > collideeMax || colliderMax < collideeMin)) { //check for intersection
			double colDepth = collideeMax - colliderMin;
			Point3D potColPoint = minPoint;
			bool flipped = false;
			if (colliderMax - collideeMin < colDepth) {
				flipped = true;
				colDepth = colliderMax - collideeMin;
				n = n.getInverse();
				potColPoint = maxPoint;
			}

			if (getPointInsideBody(potColPoint) && (colDepth < lowestColDepth || lowestColDepth == -1)) {
				lowestColDepth = colDepth;
				colPoint = potColPoint;
				colVector = n;
				winningCollideeMax = (flipped) ? -collideeMin : collideeMax;
			}
		}
		else {
			//existense of separating axis => no colliding
			*separatingAxis = true;
			return false;
		}
	}

	if (lowestColDepth != -1) {
		Point3D zero(0, 0, 0);
		for (Point3D* p : potCollider->colPoints) {
			if (getPointInsideBody(*p)) {
				double nDotVal = Vector3D(zero, *p).dotProduct(colVector);
				colSupPoints->push_back(ColPointInfo(*p, winningCollideeMax - nDotVal));
			}
		}
	}

	*collisionPoint = colPoint;
	*nVect = colVector;
	*colDepth = lowestColDepth;
	*separatingAxis = false;
	return lowestColDepth != -1;
}

bool ConvexHull::SATEdgeCol(ConvexHull* potCollider, Point3D* collisionPoint, Vector3D* nVect, double* collisionDepth, bool* separatingAxis) {
	std::vector<Vector3D> testedDirs;

	Point3D colPoint;
	Vector3D colVector;
	double lowestColDepth = -1;
	Point3D zero(0, 0, 0);

	bool separtingAxisFound = false;

	for (Edge* edge1 : colEdges) {

		for (Edge* edge2 : potCollider->colEdges) {

			Vector3D n = Vector3D(*edge1->p1, *edge1->p2).crossProduct(Vector3D(*edge2->p1, *edge2->p2));
			if (!n.notZero()) {
				continue;// vectors parralel, cant find normal
			}
			n = n.getUnitVector();


			double colliderMax, colliderMin, collideeMax, collideeMin;
			bool colliderFirst = true;
			bool collideeFirst = true;

			findMaxMin(n, &collideeMax, &collideeMin, nullptr, nullptr);
			potCollider->findMaxMin(n, &colliderMax, &colliderMin, nullptr, nullptr);

			if (!(colliderMin > collideeMax || colliderMax < collideeMin)) { //check for intersection
				double colDepth = collideeMax - colliderMin;
				if (colliderMax - collideeMin < colDepth) {
					colDepth = colliderMax - collideeMin;
					n = n.getInverse();
				}

				if (colDepth < lowestColDepth || lowestColDepth == -1) {
					Vector3D e1Axis = Vector3D(*edge1->p1, *edge1->p2).multiply(edge1->inverseMagnitude);
					Vector3D e1Norm = e1Axis.crossProduct(n);

					Vector3D e2p1Rel = Vector3D(*edge1->p1, *edge2->p1);
					Vector3D e2p2Rel = Vector3D(*edge1->p1, *edge2->p2);

					double p1e1Val = e2p1Rel.dotProduct(e1Axis);
					double p2e1Val = e2p2Rel.dotProduct(e1Axis);
					double p1NormVal = e2p1Rel.dotProduct(e1Norm);
					double p2NormVal = e2p2Rel.dotProduct(e1Norm);

					if (p2NormVal - p1NormVal == 0) {
						continue; //edges are parralel, collision can't occur
					}

					double p1ToP2e1Slope = (p2e1Val - p1e1Val) / (p2NormVal - p1NormVal);
					double intersectE1AxisVal = p1e1Val + p1ToP2e1Slope * (-p1NormVal);
					if (intersectE1AxisVal < 0 || intersectE1AxisVal >(1.0 / edge1->inverseMagnitude)) {
						continue; //edges are offset and couldnt collide
					}

					Point3D* p1 = edge1->p1;
					Vector3D e1ToColP = e1Axis.multiply(intersectE1AxisVal);
					colPoint = Point3D(p1->x + e1ToColP.x, p1->y + e1ToColP.y, p1->z + e1ToColP.z);
					colVector = n;
					lowestColDepth = colDepth;
				}
			}
			else {
				*separatingAxis = true;
				//existense of separating axis => no colliding
				return false;
			}
		}
	}

	*collisionPoint = colPoint;
	*nVect = colVector;
	*collisionDepth = lowestColDepth;
	*separatingAxis = false;
	return lowestColDepth != -1;
}